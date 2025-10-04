import csv
from pathlib import Path
import traci

CFG = "simulation.sumocfg"
OUT = Path("data/out")
OUT.mkdir(parents=True, exist_ok=True)
CSV_PATH = OUT / "sumo_detections.csv"

traci.start(["sumo", "-c", CFG])   # use "sumo-gui" for GUI

with open(CSV_PATH, "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=[
        "time", "vehicle_id", "type", "x", "y", "speed", "lane", "edge"
    ])
    writer.writeheader()

    t = 0
    try:
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            veh_ids = traci.vehicle.getIDList()
            for vid in veh_ids:
                vtype = traci.vehicle.getTypeID(vid)
                x, y = traci.vehicle.getPosition(vid)
                speed = traci.vehicle.getSpeed(vid)
                lane = traci.vehicle.getLaneID(vid)
                edge = traci.vehicle.getRoadID(vid)
                writer.writerow({
                    "time": t, "vehicle_id": vid, "type": vtype,
                    "x": x, "y": y, "speed": speed, "lane": lane, "edge": edge
                })
            t += 1
    finally:
        traci.close()

print(f"[OK] Wrote {CSV_PATH}")
