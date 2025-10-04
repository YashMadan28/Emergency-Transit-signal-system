import traci
from collections import defaultdict

CFG = "simulation.sumocfg"
SUMO = "sumo-gui"

# --- knobs that can tune live ---
EMERGENCY_DIST = 170.0        # start preempting if EMS within this distance to next TLS
ETA_MAX_SEC    = 15.0         # or ETA below this (distance / speed)
NEAR_STOP_M    = 15.0         # also preempt if EMS is within 15m regardless of ETA
HOLD_GREEN_S   = 12           # keep holding green while requests exist
RESTORE_COOLDOWN_S = 2        # after last request clears, wait then restore normal program
SPEED_FLOOR    = 0.5
AGGRESSIVE_EMS = True         # make EMS “bulldozer” during preemption
# --------------------------------

def eta(d, v):
    return d / (v if v > SPEED_FLOOR else SPEED_FLOOR)

# Memory per TLS to restore normal operation
saved_prog  = {}   # tlsID -> programID
saved_phase = {}   # tlsID -> phase index
last_request_t = {}  # tlsID -> last sim time we had any EMS request here
preempting = set()   # tlsIDs currently in preemption

def begin_preempt_if_needed(tls_id):
    """Save normal plan once when we first preempt this TLS."""
    if tls_id in preempting:
        return
    try:
        saved_prog[tls_id]  = traci.trafficlight.getProgram(tls_id)
        saved_phase[tls_id] = traci.trafficlight.getPhase(tls_id)
    except Exception:
        # Fallback if program/phase not available
        saved_prog[tls_id]  = None
        saved_phase[tls_id] = None
    preempting.add(tls_id)

def force_links_green(tls_id, link_indices, hold_s=HOLD_GREEN_S):
    """Set all links RED except requested link indices = GREEN (supports multiple EMS)."""
    cur = traci.trafficlight.getRedYellowGreenState(tls_id)
    n = len(cur)
    state = ['r'] * n
    for k in link_indices:
        if 0 <= k < n:
            state[k] = 'G'
    traci.trafficlight.setRedYellowGreenState(tls_id, ''.join(state))
    traci.trafficlight.setPhaseDuration(tls_id, max(1, hold_s))

def restore_normal_if_idle(tls_id):
    """When no EMS wants this TLS for a short cooldown, restore saved program/phase."""
    if tls_id not in preempting:
        return
    prog = saved_prog.get(tls_id)
    ph   = saved_phase.get(tls_id)
    try:
        if prog is not None:
            traci.trafficlight.setProgram(tls_id, prog)
        if ph is not None:
            traci.trafficlight.setPhase(tls_id, ph)
        traci.trafficlight.setPhaseDuration(1)
    except Exception:
        pass
    preempting.discard(tls_id)

def main():
    traci.start([
        SUMO, "-c", CFG,
        "--time-to-teleport", "-1",            # don't delete waiting EMS
        "--collision.action", "teleport",
        "--collision.mingap-factor", "0.5"
    ])

    try:
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            t = traci.simulation.getTime()

            # Collect preemption requests per TLS
            requests = defaultdict(set)  # tlsID -> set of link indices to set GREEN

            for vid in traci.vehicle.getIDList():
                if traci.vehicle.getTypeID(vid) != "emergency":
                    continue

                nxt = traci.vehicle.getNextTLS(vid)
                if not nxt:
                    continue

                tls_id, link_idx, dist_m, _ = nxt[0]
                spd = traci.vehicle.getSpeed(vid)
                t_eta = eta(dist_m, spd)

                # be assertive during approach so EMS doesn’t stall
                if AGGRESSIVE_EMS and dist_m < EMERGENCY_DIST:
                    traci.vehicle.setLaneChangeMode(vid, 0)  # ignore LC rules
                    traci.vehicle.setSpeedMode(vid, 0)       # ignore right-of-way penalties

                # trigger if close & soon, or already at stop line
                if (dist_m < EMERGENCY_DIST and t_eta < ETA_MAX_SEC) or (dist_m < NEAR_STOP_M):
                    requests[tls_id].add(link_idx)

            # Apply preemption TLS by TLS (supports multiple EMS per TLS)
            for tls_id, links in requests.items():
                begin_preempt_if_needed(tls_id)
                force_links_green(tls_id, links, HOLD_GREEN_S)
                last_request_t[tls_id] = t
                # log once per step
                print(f"[PREEMPT] t={t:.1f} tls={tls_id} links={sorted(list(links))}")

            # Handle restore for TLS without requests
            for tls_id in traci.trafficlight.getIDList():
                if tls_id in requests:
                    continue
                last = last_request_t.get(tls_id, -1e9)
                if (t - last) >= RESTORE_COOLDOWN_S:
                    restore_normal_if_idle(tls_id)

    finally:
        traci.close()

if __name__ == "__main__":
    main()
