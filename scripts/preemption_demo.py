import traci

CFG = "simulation.sumocfg"
SUMO_BIN = "sumo-gui"

# Tunables
EMERGENCY_DIST = 180.0     # start acting earlier
ETA_MAX_SEC    = 12.0
HOLD_GREEN_SEC = 8
REARM_SEC      = 10
SPEED_FLOOR    = 0.3

last_preempt_at = {}  # tlsID -> sim time

def eta(dist, speed):
    v = max(speed, SPEED_FLOOR)
    return dist / v

def force_only_this_link_green(tls_id, link_index, hold_s):
    # Build an "all red" state and set EMS movement to 'G'
    current = traci.trafficlight.getRedYellowGreenState(tls_id)
    n = len(current)
    state = ['r'] * n
    if 0 <= link_index < n:
        state[link_index] = 'G'
    # Apply and hold
    traci.trafficlight.setRedYellowGreenState(tls_id, ''.join(state))
    traci.trafficlight.setPhaseDuration(tls_id, max(1, hold_s))

def main():
    traci.start([SUMO_BIN, "-c", CFG])
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

                tls_id, link_index, dist_m, cur_link_state = nxt[0]
                spd = traci.vehicle.getSpeed(vid)
                t_eta = eta(dist_m, spd)

                # Debounce
                if tls_id in last_preempt_at and (t - last_preempt_at[tls_id]) < REARM_SEC:
                    continue

                # Trigger if close or already stuck near stop line
                if (dist_m < EMERGENCY_DIST and t_eta < ETA_MAX_SEC) or (dist_m < 15 and spd < 0.5):
                    force_only_this_link_green(tls_id, link_index, HOLD_GREEN_SEC)
                    last_preempt_at[tls_id] = t
                    print(f"[PREEMPT] t={t:.1f}s tls={tls_id} link={link_index} "
                          f"vid={vid} dist={dist_m:.1f}m eta={t_eta:.1f}s spd={spd:.1f}")
    finally:
        traci.close()

if __name__ == "__main__":
    main()
