# scripts/ems_priority.py
import os, sys, csv
from pathlib import Path
from collections import defaultdict

# SUMO tools
if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))
import traci  # type: ignore

ROOT = Path(__file__).resolve().parents[1]
CFG  = str(ROOT / "data" / "city" / "city.sumocfg")
SUMO_BIN = os.environ.get("SUMO_BIN", "sumo-gui")  # set SUMO_BIN=sumo for headless

# --- Tunables (balanced for a tiny city) ---
EMS_DIST        = 75.0    # m: EMS must be close to claim the next TLS
TRAIN_WARN_DIST = 180.0   # m: manage trains early to avoid hard braking
TRAIN_HOLD_MAX  = 12      # s: per-train hold budget while EMS is preempting
PULSE           = 0.8     # s: short forced-state pulses so base cycle keeps running
# -------------------------------------------

# --- Logging setup ---
OUT_DIR = ROOT / "data" / "out"
OUT_DIR.mkdir(parents=True, exist_ok=True)

STEP_LOG  = OUT_DIR / "step_log.csv"
TRIP_LOG  = OUT_DIR / "trip_log.csv"
EVENT_LOG = OUT_DIR / "events_log.csv"

# initialise CSV headers once
if not STEP_LOG.exists():
    with STEP_LOG.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time", "veh_id", "type", "speed", "edge", "lane", "pos", "waiting"])

if not TRIP_LOG.exists():
    with TRIP_LOG.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["veh_id", "type", "route", "depart_time", "arrival_time", "travel_time"])

if not EVENT_LOG.exists():
    with EVENT_LOG.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time", "event_type", "source", "tls_id", "state"])
# ---------------------


def is_ems(v):   return traci.vehicle.getTypeID(v) == "emergency"
def is_train(v): return traci.vehicle.getTypeID(v) == "train"

def analyze_tls(tls_id):
    """
    For each TLS, collect:
      - n: number of controlled links
      - rail_idxs: set(linkIdx) for rail approaches (edge id starts with 'rail_')
      - road_idxs: set(linkIdx) for road approaches (the complement)
      - inc_edge_to_link_idxs: map incoming edge -> [linkIdx,...]
      - is_crossing: True if this TLS controls any rail links
    """
    ctrl = traci.trafficlight.getControlledLinks(tls_id)
    n = len(ctrl)
    rail_idxs, road_idxs = set(), set()
    inc_edge_to_link_idxs = defaultdict(list)
    for idx, triples in enumerate(ctrl):
        inc_lane = triples[0][0]
        inc_edge = traci.lane.getEdgeID(inc_lane)
        if inc_edge.startswith("rail_"):
            rail_idxs.add(idx)
        else:
            road_idxs.add(idx)
            inc_edge_to_link_idxs[inc_edge].append(idx)
    return {
        "n": n,
        "rail_idxs": rail_idxs,
        "road_idxs": road_idxs,
        "inc_edge_to_link_idxs": dict(inc_edge_to_link_idxs),
        "is_crossing": len(rail_idxs) > 0
    }

def build_state(n, greens, force_red=None):
    s = ["r"] * n
    for i in greens: s[i] = "G"
    if force_red:
        for i in force_red: s[i] = "r"
    return "".join(s)

def main():
    if not Path(CFG).exists():
        raise SystemExit("Build first: python scripts/build_city_net.py")

    traci.start([SUMO_BIN, "-c", CFG,
                 "--quit-on-end", "true",
                 "--time-to-teleport", "-1",
                 "--collision.action", "teleport",
                 "--collision.mingap-factor", "0.5"])

    print("[SUMO] started:", SUMO_BIN)
    tls_meta = {tls: analyze_tls(tls) for tls in traci.trafficlight.getIDList()}

    preempt_until = {}                       # tls_id -> time
    train_hold_left = defaultdict(int)       # (tls_id, train_id) -> seconds left

    # trip tracking for logging
    depart_times = {}        # veh_id -> first seen time
    veh_types    = {}        # veh_id -> type
    veh_routes   = {}        # veh_id -> route id
    arrived      = set()     # veh_ids we've already logged as finished

    try:
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            t = traci.simulation.getTime()

            # --- logging: per-step + trip depart/arrival ---
            veh_ids = list(traci.vehicle.getIDList())
            veh_set = set(veh_ids)

            # step-level log
            with STEP_LOG.open("a", newline="") as f_step:
                w_step = csv.writer(f_step)
                for vid in veh_ids:
                    try:
                        vtype   = traci.vehicle.getTypeID(vid)
                        speed   = traci.vehicle.getSpeed(vid)
                        edge    = traci.vehicle.getRoadID(vid)
                        lane    = traci.vehicle.getLaneID(vid)
                        pos     = traci.vehicle.getLanePosition(vid)
                        waiting = traci.vehicle.getWaitingTime(vid)
                    except traci.TraCIException:
                        continue
                    w_step.writerow([t, vid, vtype, speed, edge, lane, pos, waiting])

            # first time we see a vehicle -> record depart time and metadata
            for vid in veh_ids:
                if vid not in depart_times:
                    depart_times[vid] = t
                    try:
                        veh_types[vid] = traci.vehicle.getTypeID(vid)
                    except traci.TraCIException:
                        veh_types[vid] = "unknown"
                    try:
                        veh_routes[vid] = traci.vehicle.getRouteID(vid)
                    except traci.TraCIException:
                        veh_routes[vid] = "unknown"

            # vehicles that disappeared since last step -> log trip
            with TRIP_LOG.open("a", newline="") as f_trip:
                w_trip = csv.writer(f_trip)
                for vid in list(depart_times.keys()):
                    if vid not in veh_set and vid not in arrived:
                        arrived.add(vid)
                        dep = depart_times.get(vid, 0.0)
                        arr = t
                        vtype = veh_types.get(vid, "unknown")
                        route = veh_routes.get(vid, "unknown")
                        w_trip.writerow([vid, vtype, route, dep, arr, arr - dep])

            # --- 1) EMS claims: green only on its incoming link; rail kept red
            ems_green = {}     # tls_id -> set(linkIdx)
            blocked_tls = set()
            for vid in traci.vehicle.getIDList():
                if not is_ems(vid):
                    continue
                nxt = traci.vehicle.getNextTLS(vid)
                if not nxt:
                    continue
                tls_id, link_idx, dist, _ = nxt[0]
                if dist <= EMS_DIST:
                    ems_green.setdefault(tls_id, set()).add(link_idx)
                    blocked_tls.add(tls_id)
                    traci.vehicle.setLaneChangeMode(vid, 0)
                    traci.vehicle.setSpeedMode(vid, 0)

            # --- 2) Trains near crossings: hold/slow during EMS, else allow
            train_near = set()               # tls_id with a train in warn zone
            train_green = {}                 # tls_id -> set(linkIdx)
            for vid in traci.vehicle.getIDList():
                if not is_train(vid):
                    continue
                nxt = traci.vehicle.getNextTLS(vid)
                if not nxt:
                    continue
                tls_id, link_idx, dist, _ = nxt[0]
                meta = tls_meta.get(tls_id)
                if not meta or not meta["is_crossing"]:
                    continue

                if dist < TRAIN_WARN_DIST:
                    train_near.add(tls_id)

                key = (tls_id, vid)
                if tls_id in blocked_tls and dist < TRAIN_WARN_DIST:
                    # EMS is preempting this crossing -> hold/slow with budget
                    left = train_hold_left.get(key, TRAIN_HOLD_MAX)
                    if left > 0:
                        if dist < 25:
                            traci.vehicle.slowDown(vid, 0.1, 3.0)
                        elif dist < 70:
                            traci.vehicle.slowDown(vid, 3.0, 3.0)
                        else:
                            traci.vehicle.slowDown(vid, 6.0, 3.0)
                        train_hold_left[key] = left - 1
                    else:
                        # release once budget exhausted
                        traci.vehicle.setSpeed(vid, -1)
                        train_green.setdefault(tls_id, set()).add(link_idx)
                else:
                    # no EMS preemption -> let trains proceed when near
                    if dist < TRAIN_WARN_DIST:
                        traci.vehicle.setSpeed(vid, -1)
                        train_green.setdefault(tls_id, set()).add(link_idx)

            # --- 3) Anti-starvation at crossings:
            # If no EMS and no train is near a crossing, force ROAD green.
            crossing_road_green = {}
            for tls_id, meta in tls_meta.items():
                if not meta["is_crossing"]:
                    continue
                if (tls_id not in blocked_tls) and (tls_id not in train_near):
                    # keep roads moving across the rail row
                    crossing_road_green[tls_id] = set(meta["road_idxs"])

            # --- 4) Apply forced states (short pulses so base plan resumes quickly)
            touched = set(ems_green) | set(train_green) | set(crossing_road_green)
            with EVENT_LOG.open("a", newline="") as f_event:
                w_event = csv.writer(f_event)

                for tls_id in touched:
                    meta = tls_meta[tls_id]
                    greens = set()
                    # default crossing road-green if eligible
                    if tls_id in crossing_road_green:
                        greens |= crossing_road_green[tls_id]
                    # allow trains (when not blocked by EMS)
                    if tls_id in train_green:
                        greens |= train_green[tls_id]
                    # EMS wins and rail must stay red
                    if tls_id in ems_green:
                        greens = set(ems_green[tls_id])      # only the EMS link(s)
                        force_red = meta["rail_idxs"]
                        event_type = "ems_preempt"
                        source = "ems"
                    elif tls_id in train_green:
                        force_red = set()
                        event_type = "train_request"
                        source = "train"
                    elif tls_id in crossing_road_green:
                        force_red = set()
                        event_type = "road_keep_green"
                        source = "controller"
                    else:
                        force_red = set()
                        event_type = "other"
                        source = "controller"

                    state = build_state(meta["n"], greens, force_red)

                    # log the TLS override event
                    w_event.writerow([t, event_type, source, tls_id, state])

                    traci.trafficlight.setRedYellowGreenState(tls_id, state)
                    traci.trafficlight.setPhaseDuration(tls_id, PULSE)
                    preempt_until[tls_id] = t + PULSE + 0.1

            # --- 5) Clear expired entries
            for tls_id in list(preempt_until.keys()):
                if t >= preempt_until[tls_id]:
                    preempt_until.pop(tls_id, None)

            # Clean train budgets for trains that left/do not target TLS anymore
            for key in list(train_hold_left.keys()):
                tls_id, train_id = key
                try:
                    nxt = traci.vehicle.getNextTLS(train_id)
                    if not nxt:
                        train_hold_left.pop(key, None)
                except traci.TraCIException:
                    train_hold_left.pop(key, None)

    finally:
        traci.close()
        print("[SUMO] closed")


if __name__ == "__main__":
    main()
