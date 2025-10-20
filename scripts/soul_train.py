# scripts/soul_train.py
# Soul-Train controller with proper non-conflicting signal states.
# Road and rail movements are exclusive (no simultaneous green).

import os
from pathlib import Path
import traci

CFG = "rail.sumocfg"
SUMO_BIN = os.environ.get("SUMO_BIN", "sumo-gui")  # set SUMO_BIN=sumo for headless

# Tunables
EMS_DIST = 120.0          # EMS considered "close" (m)
EMS_NEAR = 15.0           # very close to stop line (m)
TRAIN_WARN_DIST = 80.0    # intervene when train is within this distance (m)
HOLD_ROAD_GREEN = 10      # s to keep road green to let EMS clear
TRAIN_HOLD_MAX = 12       # total seconds we will hold/slow the train

def is_train(vid: str) -> bool:
    try: return traci.vehicle.getTypeID(vid) == "train"
    except traci.TraCIException: return False

def is_ems(vid: str) -> bool:
    try: return traci.vehicle.getTypeID(vid) == "emergency"
    except traci.TraCIException: return False

def build_state(base_len: int, road_idxs, rail_idxs, road_green: bool, rail_green: bool) -> str:
    """
    Construct a safe state string:
      - road links -> 'G' if road_green else 'r'
      - rail links -> 'G' if rail_green else 'r'
      - everything else -> 'r'
    """
    s = ['r'] * base_len
    for i in road_idxs:
        s[i] = 'G' if road_green else 'r'
    for i in rail_idxs:
        s[i] = 'G' if rail_green else 'r'
    return ''.join(s)

def main():
    Path("data/out").mkdir(parents=True, exist_ok=True)

    traci.start([
        SUMO_BIN, "-c", CFG,
        "--quit-on-end", "true",
        "--time-to-teleport", "-1",
        "--collision.action", "teleport",
        "--collision.mingap-factor", "0.5",
        "--error-log", "data/out/sumo_err.txt",
        "--log", "data/out/sumo_log.txt",
    ])

    tls_ids = traci.trafficlight.getIDList()
    if not tls_ids:
        raise RuntimeError("No TLS found. Ensure node 'c' is type='traffic_light'.")
    tls = tls_ids[0]

    # --- Map TLS link indices to ROAD vs RAIL once at start ---
    # getControlledLinks returns a list of size N (N = length of signal state).
    # Each entry is a list of (incomingLane, outgoingLane, viaLane) tuples for that signal index.
    ctrl = traci.trafficlight.getControlledLinks(tls)
    n_signals = len(ctrl)

    road_idxs, rail_idxs = [], []
    for idx, triples in enumerate(ctrl):
        # Some indices may control multiple connections; classify as rail if ANY incoming lane comes from a *_rail edge.
        is_rail_here = False
        for (inc, out, via) in triples:
            # inc looks like "edgeID_laneIndex", e.g., "n2c_rail_0" or "w2c_0"
            if "_rail" in inc:
                is_rail_here = True
                break
        if is_rail_here:
            rail_idxs.append(idx)
        else:
            road_idxs.append(idx)

    print(f"[INIT] TLS '{tls}': total={n_signals}, road_idxs={road_idxs}, rail_idxs={rail_idxs}")

    def force(road_green: bool, rail_green: bool, hold_s: int):
        state = build_state(n_signals, road_idxs, rail_idxs, road_green, rail_green)
        traci.trafficlight.setRedYellowGreenState(tls, state)
        traci.trafficlight.setPhaseDuration(tls, max(1, int(hold_s)))

    # Remaining hold seconds per train id
    train_hold_left = {}

    try:
        # Default: ROAD green, RAIL red
        force(True, False, 5)

        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            t = traci.simulation.getTime()

            # Find closest train approaching the TLS
            closest_train_id, closest_train_dist = None, float("inf")
            for vid in traci.vehicle.getIDList():
                if not is_train(vid): continue
                nxt = traci.vehicle.getNextTLS(vid)
                if not nxt: continue
                tls_id, _idx, dist_m, _ = nxt[0]
                if tls_id == tls and dist_m < closest_train_dist:
                    closest_train_id, closest_train_dist = vid, dist_m

            # Is an EMS close to the TLS?
            ems_close = False
            for vid in traci.vehicle.getIDList():
                if not is_ems(vid): continue
                nxt = traci.vehicle.getNextTLS(vid)
                if not nxt: continue
                tls_id, _idx, dist_m, _ = nxt[0]
                if tls_id != tls: continue
                spd = traci.vehicle.getSpeed(vid)
                if (dist_m <= EMS_DIST) or (dist_m <= EMS_NEAR and spd < 1.0):
                    # Encourage assertive EMS behavior near stop line
                    traci.vehicle.setLaneChangeMode(vid, 0)
                    traci.vehicle.setSpeedMode(vid, 0)
                    ems_close = True

            # Debug
            print(f"[DEBUG] t={t:5.1f}s  train_dist={closest_train_dist:6.1f}  ems_close={ems_close}")

            # Decision logic with MUTUAL EXCLUSION of movements
            if closest_train_id is not None and closest_train_dist < TRAIN_WARN_DIST:
                if ems_close:
                    # Keep ROAD green for a short window so EMS clears; RAIL stays red.
                    force(True, False, HOLD_ROAD_GREEN)
                    # Slow/hold the train briefly
                    left = train_hold_left.get(closest_train_id, TRAIN_HOLD_MAX)
                    if left > 0:
                        if closest_train_dist < 25:
                            traci.vehicle.setSpeed(closest_train_id, 0)      # stop if very close
                        else:
                            traci.vehicle.slowDown(closest_train_id, 3.0, 3)  # slow otherwise
                        train_hold_left[closest_train_id] = left - 1
                        print(f"[SOUL-TRAIN] Holding train {closest_train_id} ({left-1}s left); ROAD GREEN for EMS")
                    else:
                        # Safety cap reached: close road for train
                        force(False, True, 6)  # rail green now, road red
                        traci.vehicle.setSpeed(closest_train_id, -1)  # release control
                        print("[SOUL-TRAIN] Max hold reached; ROAD RED / RAIL GREEN for train")
                else:
                    # No EMS: close ROAD, open RAIL for train
                    force(False, True, 8)
                    if closest_train_id in train_hold_left:
                        del train_hold_left[closest_train_id]
                    traci.vehicle.setSpeed(closest_train_id, -1)
                    print("[SOUL-TRAIN] No EMS; ROAD RED / RAIL GREEN – train proceeds")
            else:
                # No train nearby: default to ROAD green, RAIL red
                force(True, False, 5)

    finally:
        traci.close()

if __name__ == "__main__":
    main()
