# dashboard.py
import streamlit as st
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

st.set_page_config(
    page_title="EMS Transit Priority – Data Science Dashboard",
    layout="wide",
)

OUT_DIR = Path("data/out")
STEP_LOG = OUT_DIR / "step_log.csv"
TRIP_LOG = OUT_DIR / "trip_log.csv"
EVENT_LOG = OUT_DIR / "events_log.csv"


@st.cache_data
def load_data():
    step_df = pd.read_csv(STEP_LOG) if STEP_LOG.exists() else None
    trip_df = pd.read_csv(TRIP_LOG) if TRIP_LOG.exists() else None
    event_df = pd.read_csv(EVENT_LOG) if EVENT_LOG.exists() else None
    return step_df, trip_df, event_df


def main():
    st.title("🚑 EMS Transit Priority – Smart City Analytics")

    if not OUT_DIR.exists() or not any(OUT_DIR.glob("*.csv")):
        st.warning(
            "No log files found in `data/out`. "
            "Run one simulation first:\n\n"
            "```bash\n"
            "python3 scripts/build_city_net.py\n"
            "export SUMO_BIN=sumo-gui   # or 'sumo'\n"
            "python3 scripts/ems_priority.py\n"
            "```"
        )
        return

    step_df, trip_df, event_df = load_data()

    if trip_df is None or trip_df.empty:
        st.error("`trip_log.csv` is missing or empty – run a simulation first.")
        return

    # --- Sidebar controls ---
    st.sidebar.header("Filters")

    # Vehicle type filter for per-step view
    all_types = sorted(trip_df["type"].dropna().unique().tolist())
    sel_type = st.sidebar.selectbox(
        "Vehicle type focus (for detailed plots)",
        options=all_types,
        index=all_types.index("emergency") if "emergency" in all_types else 0,
    )

    st.sidebar.markdown("---")
    st.sidebar.markdown("**How to use:**")
    st.sidebar.markdown(
        "- Run your SUMO + EMS simulation\n"
        "- This dashboard reads logs from `data/out`\n"
        "- Explore EMS travel times, delays, and preemption events"
    )

    # --- Top-level trip stats ---
    st.subheader("📊 Trip-level Performance")

    ems_trips = trip_df[trip_df["type"] == "emergency"].copy()
    car_trips = trip_df[trip_df["type"] == "car"].copy()
    train_trips = trip_df[trip_df["type"] == "train"].copy()

    c1, c2, c3 = st.columns(3)
    with c1:
        st.metric("EMS trips (emergency)", len(ems_trips))
    with c2:
        st.metric("Car trips", len(car_trips))
    with c3:
        st.metric("Train trips", len(train_trips))

    if not ems_trips.empty:
        ems_trips = ems_trips[ems_trips["travel_time"] >= 0]
        avg_ems = ems_trips["travel_time"].mean()
        p95_ems = ems_trips["travel_time"].quantile(0.95)
        min_ems = ems_trips["travel_time"].min()
        max_ems = ems_trips["travel_time"].max()

        st.markdown("### 🚑 EMS Travel Times")

        c1, c2, c3, c4 = st.columns(4)
        with c1:
            st.metric("Avg EMS travel time (s)", f"{avg_ems:.1f}")
        with c2:
            st.metric("95th percentile EMS time (s)", f"{p95_ems:.1f}")
        with c3:
            st.metric("Min EMS time (s)", f"{min_ems:.1f}")
        with c4:
            st.metric("Max EMS time (s)", f"{max_ems:.1f}")

        fig, ax = plt.subplots()
        ax.hist(ems_trips["travel_time"], bins=20)
        ax.set_xlabel("EMS travel time (s)")
        ax.set_ylabel("Count")
        ax.set_title("EMS travel time distribution")
        st.pyplot(fig)

    if not car_trips.empty:
        car_trips = car_trips[car_trips["travel_time"] >= 0]
        avg_car = car_trips["travel_time"].mean()
        st.markdown("### 🚗 Car Travel Times (Context)")
        c1, c2 = st.columns(2)
        with c1:
            st.metric("Avg car travel time (s)", f"{avg_car:.1f}")
        with c2:
            st.metric(
                "Cars vs EMS (avg ratio)",
                f"{avg_car / avg_ems:.2f}" if "avg_ems" in locals() and avg_ems > 0 else "N/A",
            )

    # --- Events: preemptions and train requests ---
    st.subheader("🚦 Signal Events: EMS Preemption & Train Requests")

    if event_df is not None and not event_df.empty:
        st.markdown("Sample of signal events:")
        st.dataframe(event_df.head(20))

        event_df["time_bucket"] = (event_df["time"] // 60).astype(int)  # per minute
        ems_events = event_df[event_df["event_type"] == "ems_preempt"]
        train_events = event_df[event_df["event_type"] == "train_request"]

        ems_counts = ems_events.groupby("time_bucket")["event_type"].count()
        train_counts = train_events.groupby("time_bucket")["event_type"].count()

        fig2, ax2 = plt.subplots()
        if not ems_counts.empty:
            ax2.plot(ems_counts.index, ems_counts.values, marker="o", label="EMS preemptions")
        if not train_counts.empty:
            ax2.plot(train_counts.index, train_counts.values, marker="x", label="Train requests")
        ax2.set_xlabel("Time (minutes)")
        ax2.set_ylabel("Count")
        ax2.set_title("Preemptions & train requests per minute")
        ax2.legend()
        st.pyplot(fig2)
    else:
        st.info("No `events_log.csv` found or it's empty. Preemption events won't be shown.")

    # --- Detailed per-step view for selected vehicle type ---
    st.subheader(f"⏱ Detailed dynamics for '{sel_type}' vehicles")

    if step_df is None or step_df.empty:
        st.info("No `step_log.csv` found – step-by-step data not available.")
        return

    # Join step log with trip info for selected type
    step_sel = step_df[step_df["type"] == sel_type].copy()
    if step_sel.empty:
        st.info(f"No step data for type '{sel_type}'. Try another type in the sidebar.")
        return

    # Allow user to choose a specific vehicle ID of that type
    unique_vids = sorted(step_sel["veh_id"].unique().tolist())
    chosen_vid = st.selectbox(f"Choose {sel_type} vehicle ID to inspect", unique_vids)

    veh_steps = step_sel[step_sel["veh_id"] == chosen_vid].copy()
    veh_steps = veh_steps.sort_values("time")

    if veh_steps.empty:
        st.info("No steps for that vehicle yet (maybe it hasn't spawned?).")
        return

    st.markdown(f"#### Trajectory for vehicle ID `{chosen_vid}`")

    c1, c2 = st.columns(2)

    with c1:
        fig3, ax3 = plt.subplots()
        ax3.plot(veh_steps["time"], veh_steps["speed"])
        ax3.set_xlabel("Time (s)")
        ax3.set_ylabel("Speed (m/s)")
        ax3.set_title("Speed profile")
        st.pyplot(fig3)

    with c2:
        fig4, ax4 = plt.subplots()
        ax4.plot(veh_steps["time"], veh_steps["waiting"])
        ax4.set_xlabel("Time (s)")
        ax4.set_ylabel("Waiting time (s)")
        ax4.set_title("Accumulated waiting time")
        st.pyplot(fig4)

    st.markdown("#### Raw step data for this vehicle")
    st.dataframe(veh_steps.reset_index(drop=True))


if __name__ == "__main__":
    main()
