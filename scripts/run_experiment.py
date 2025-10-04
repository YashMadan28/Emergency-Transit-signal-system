import argparse, csv
from collections import defaultdict
import traci

# ----- TUNING -----
APPROACH_M = 120.0       # when we start tracking an EMS for the next TLS
NEAR_STOP_M = 15.0       # "near the stop line"
SPEED_STOP = 0.5         # below this => considered stopped
HOLD_GREEN_S = 12        # how long we hold a forced green (when ON)
# -------------------

def eta(d, v): return d / max(v, 0.3)

def set_only_links_green(tls_id, link_indices):
    state = list(traci.trafficlight.getRedYellowGreenState(tls_id))
    for i in range(len(state)):
        state[i] = 'r'
    for k in link_indices:
        if 0 <= k < len(state):
            state[k] = 'G'
    traci.trafficlight.setRedYellowGreenState(tls_id, ''.join(state))
    traci.trafficlight.setPhaseDuration(tls_id, HOLD_GREEN_S) 
def run(cfg, preempt_on, out_csv):
    traci.start([
        "sumo-gui", "-c", cfg,
        "--time-to-teleport", "-1",
        "--collision.action", "teleport",
        "--collision.mingap-factor", "0.5",
    ])

    # Per-(tls_id, ems_id) tracking
    active = {}   # key=(tls,vid) -> dict(state)
    results = []  # finished rows

    try:
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            t = traci.simulation.getTime()

            # Build requests per TLS (for ON mode)
            requests = defaultdict(set)

            for vid in traci.vehicle.getIDList():
                if traci.vehicle.getTypeID(vid) != "emergency":
                    continue

                nxt = traci.vehicle.getNextTLS(vid)
                if not nxt:
                    # If we were tracking this EMS/TLS and now it has no TLS, close all open records for this vid
                    for (tls2, v2), st in list(active.items()):
                        if v2 == vid:
                            # close without clear distance signal (fallback)
                            st["clear_time_sec"] = t - st["start_t"]
                            results.append(st)
                            del active[(tls2, v2)]
                    continue

                tls_id, link_idx, dist_m, link_state = nxt[0]
                spd = traci.vehicle.getSpeed(vid)

                key = (tls_id, vid)
                st = active.get(key)

                # Create tracking when entering approach window
                if st is None and dist_m <= APPROACH_M:
                    st = {
                        "ems_id": vid,
                        "tls_id": tls_id,
                        "start_t": t,
                        "stopped_last_step": False,
                        "stops": 0,
                        "stop_time_accum": 0.0,
                        "last_t": t,
                        "last_tls": tls_id,
                    }
                    active[key] = st

                if st is None:
                    continue  # not yet inside approach window

                # Count stop-time and stops within NEAR_STOP_M
                dt = t - st["last_t"]
                st["last_t"] = t
                if dist_m <= NEAR_STOP_M and spd < SPEED_STOP:
                    st["stop_time_accum"] += dt
                    if not st["stopped_last_step"]:
                        st["stops"] += 1
                    st["stopped_last_step"] = True
                else:
                    st["stopped_last_step"] = False

                # Preemption (ON): ask to force green on required link(s)
                if preempt_on and (dist_m <= APPROACH_M):
                    # demo aggressiveness so EMS doesn't "flinch"
                    traci.vehicle.setLaneChangeMode(vid, 0)
                    traci.vehicle.setSpeedMode(vid, 0)
                    requests[tls_id].add(link_idx)

                # Detect "cleared" – once next-TLS changed (new tls id) OR very large dist (passed)
                nxt2 = traci.vehicle.getNextTLS(vid)
                cleared = False
                if not nxt2:
                    cleared = True
                else:
                    tls2, link2, dist2, _ = nxt2[0]
                    # If TLS id changed, we passed the junction we were heading to
                    if tls2 != st["last_tls"]:
                        cleared = True
                    # If distance jumped back up while last loop we were very close, assume passed
                    elif dist_m <= 2.0 and dist2 > dist_m + 2.0:
                        cleared = True

                if cleared:
                    st["clear_time_sec"] = t - st["start_t"]
                    results.append(st)
                    del active[key]

            # Apply forced greens for all requested TLS (ON mode)
            if preempt_on:
                for tls_id, link_set in requests.items():
                    set_only_links_green(tls_id, list(link_set))

    finally:
        traci.close()

    # Write CSV
    with open(out_csv, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=[
            "tls_id", "ems_id", "stops", "stop_time_sec", "clear_time_sec"
        ])
        w.writeheader()
        for r in results:
            w.writerow({
                "tls_id": r["tls_id"],
                "ems_id": r["ems_id"],
                "stops": r["stops"],
                "stop_time_sec": round(r["stop_time_accum"], 2),
                "clear_time_sec": round(r.get("clear_time_sec", 0.0), 2),
            })
    print(f"[OK] wrote {out_csv}")

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--cfg", default="simulation.sumocfg")
    ap.add_argument("--mode", choices=["on", "off"], default="on",
                    help="'on' = preemption enabled, 'off' = baseline")
    ap.add_argument("--out", default=None, help="output CSV path")
    args = ap.parse_args()
    out_csv = args.out or (f"data/out/metrics_{args.mode}.csv")
    run(args.cfg, preempt_on=(args.mode=="on"), out_csv=out_csv)
