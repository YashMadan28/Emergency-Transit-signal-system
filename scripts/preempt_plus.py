# scripts/preempt_plus_strict.py
import traci

CFG = "simulation.sumocfg"
SUMO = "sumo-gui"

EMERGENCY_DIST = 160.0      # start preempting earlier
ETA_MAX_SEC    = 15.0
HOLD_GREEN_SEC = 15         # keep green longer
SPEED_FLOOR    = 0.5

def eta(d, v): return d / (v if v > SPEED_FLOOR else SPEED_FLOOR)

def force_only_link_green(tls_id, link_idx, hold_s):
    cur = traci.trafficlight.getRedYellowGreenState(tls_id)
    n = len(cur)
    state = ['r'] * n
    if 0 <= link_idx < n:
        state[link_idx] = 'G'
    traci.trafficlight.setRedYellowGreenState(tls_id, ''.join(state))
    traci.trafficlight.setPhaseDuration(tls_id, max(1, hold_s))

traci.start([
    SUMO, "-c", CFG,
    "--time-to-teleport", "-1",          # do NOT teleport stuck vehicles
    "--collision.action", "teleport",    # avoid deadlocks for the rest
    "--collision.mingap-factor", "0.5"
])

try:
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        t = traci.simulation.getTime()

        for vid in traci.vehicle.getIDList():
            if traci.vehicle.getTypeID(vid) != "emergency":
                continue

            nxt = traci.vehicle.getNextTLS(vid)
            if not nxt:
                continue

            tls_id, link_idx, dist_m, _ = nxt[0]
            spd = traci.vehicle.getSpeed(vid)
            t_eta = eta(dist_m, spd)

            # DEMO MODE: make EMS aggressive while preempting
            if dist_m < EMERGENCY_DIST:
                traci.vehicle.setLaneChangeMode(vid, 0)  # ignore lane-change safety checks
                traci.vehicle.setSpeedMode(vid, 0)       # ignore speed/right-of-way checks

            # Trigger OR keep reasserting while EMS remains near the stop line
            if (dist_m < EMERGENCY_DIST and t_eta < ETA_MAX_SEC) or (dist_m < 15):
                force_only_link_green(tls_id, link_idx, HOLD_GREEN_SEC)
                print(f"[PREEMPT] t={t:.1f}s tls={tls_id} link={link_idx} "
                      f"vid={vid} dist={dist_m:.1f} eta={t_eta:.1f} v={spd:.1f}")
finally:
    traci.close()
