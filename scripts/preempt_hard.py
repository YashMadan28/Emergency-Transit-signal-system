import traci

CFG = "simulation.sumocfg"
SUMO_BIN = "sumo-gui"

# Trigger thresholds (tune live if needed)
EMERGENCY_DIST = 150.0    # m
ETA_MAX_SEC    = 12.0     # s
HOLD_GREEN_SEC = 8        # s
REARM_SEC      = 8        # s
SPEED_FLOOR    = 0.3      # m/s

last_preempt_at = {}  # tlsID -> sim time

def eta(dist, speed):
    v = speed if speed > SPEED_FLOOR else SPEED_FLOOR
    return dist / v

def force_only_link_green(tls_id, link_index, hold_s):
    # all red except target link = 'G'
    state_now = traci.trafficlight.getRedYellowGreenState(tls_id)
    n = len(state_now)
    new_state = ['r'] * n
    if 0 <= link_index < n:
        new_state[link_index] = 'G'
    traci.trafficlight.setRedYellowGreenState(tls_id, ''.join(new_state))
    traci.trafficlight.setPhaseDuration(tls_id, max(1, hold_s))

def main():
    traci.start([SUMO_BIN, "-c", CFG])
    try:
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            t = traci.simulation.getTime()

            # scan EMS only
            for vid in traci.vehicle.getIDList():
                if traci.vehicle.getTypeID(vid) != "emergency":
                    continue

                nxt = traci.vehicle.getNextTLS(vid)
                if not nxt:
                    continue

                tls_id, link_idx, dist_m, link_state = nxt[0]
                spd = traci.vehicle.getSpeed(vid)
                t_eta = eta(dist_m, spd)

                # debounce per TLS
                if tls_id in last_preempt_at and (t - last_preempt_at[tls_id]) < REARM_SEC:
                    continue

                # trigger if close & arriving soon OR stuck at stop line
                if (dist_m < EMERGENCY_DIST and t_eta < ETA_MAX_SEC) or (dist_m < 12 and spd < 0.5):
                    force_only_link_green(tls_id, link_idx, HOLD_GREEN_SEC)
                    last_preempt_at[tls_id] = t
                    print(f"[PREEMPT] t={t:.1f}s tls={tls_id} link={link_idx} vid={vid} "
                          f"dist={dist_m:.1f}m eta={t_eta:.1f}s spd={spd:.1f}")
    finally:
        traci.close()

if __name__ == "__main__":
    main()
