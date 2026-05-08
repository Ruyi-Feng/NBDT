"""
SSM Event Extractor
Extract ego and target vehicle trajectories around risk events
based on one or more SSM indicators and thresholds from the SSM results JSON.

Usage:
  # single metric
  python ssm_event_extractor.py --json_path r.json --csv_path t.csv \
      --metrics TTC:2.0:below

  # multiple metrics
  python ssm_event_extractor.py --json_path r.json --csv_path t.csv \
      --metrics TTC:2.0:below DRAC:3.0:above PET:1.5:below
"""

import json
import os
import argparse
import pandas as pd


def load_json(json_path):
    with open(json_path, "r") as f:
        return json.load(f)


def parse_metric_spec(spec):
    """
    Parse a metric spec string like 'TTC:2.0:below' into a dict.
    """
    parts = spec.split(":")
    if len(parts) != 3:
        raise ValueError(
            f"Invalid metric spec '{spec}'. Expected format: NAME:THRESHOLD:CONDITION "
            f"(e.g. TTC:2.0:below)")
    name, threshold, condition = parts
    if condition not in ("below", "above"):
        raise ValueError(f"Condition must be 'below' or 'above', got '{condition}'")
    return {"name": name, "threshold": float(threshold), "condition": condition}


def find_risk_events(data, metric, threshold, condition):
    """
    Scan all ego-target pairs in the JSON and find risk events.

    A risk event is a continuous sequence of frames where the specified metric
    crosses the threshold. Consecutive triggering frames are merged into one event.

    Returns a list of dicts:
        {ego_id, target_id, start_frame, end_frame, extreme_value}
    """
    events = []
    vehicles = data.get("vehicles", {})

    for ego_id, ego_data in vehicles.items():
        targets = ego_data.get("targets", {})
        for target_id, target_data in targets.items():
            frames = target_data.get("instant_metrics", [])
            in_event = False
            event_start = None
            event_end = None
            event_values = []

            for frame_data in frames:
                frame_num = frame_data.get("frame")
                value = frame_data.get(metric)

                triggered = False
                if value is not None:
                    if condition == "below":
                        triggered = value < threshold
                    elif condition == "above":
                        triggered = value > threshold

                if triggered:
                    if not in_event:
                        event_start = frame_num
                        event_values = []
                        in_event = True
                    event_end = frame_num
                    event_values.append(value)
                else:
                    if in_event:
                        extreme = min(event_values) if condition == "below" else max(event_values)
                        events.append({
                            "ego_id": ego_id,
                            "target_id": target_id,
                            "start_frame": event_start,
                            "end_frame": event_end,
                            "extreme_value": extreme,
                        })
                        in_event = False

            # close last event if still open
            if in_event:
                extreme = min(event_values) if condition == "below" else max(event_values)
                events.append({
                    "ego_id": ego_id,
                    "target_id": target_id,
                    "start_frame": event_start,
                    "end_frame": event_end,
                    "extreme_value": extreme,
                })

    return events


def extract_trajectories(csv_path, all_metric_events, period, csv_name):
    """
    For each event from each metric, extract ego and target trajectories within
    [start_frame - period, end_frame + period] from the CSV,
    and assign a unique event_id and the event's extreme value column(s).

    all_metric_events: list of (metric_name, condition, events_list)
    """
    df = pd.read_csv(csv_path)
    all_segments = []

    for metric_name, condition, events in all_metric_events:
        extreme_col = f"{metric_name}_{'min' if condition == 'below' else 'max'}"

        for event in events:
            ego_id = event["ego_id"]
            target_id = event["target_id"]
            event_start = event["start_frame"]
            event_end = event["end_frame"]
            extreme_value = event["extreme_value"]

            # try to match id types with the CSV
            if df["carId"].dtype in ("int64", "float64"):
                try:
                    ego_id_csv = int(ego_id)
                    target_id_csv = int(target_id)
                except (ValueError, TypeError):
                    ego_id_csv = ego_id
                    target_id_csv = target_id
            else:
                ego_id_csv = ego_id
                target_id_csv = target_id

            frame_min = event_start - period
            frame_max = event_end + period

            mask = (
                df["carId"].isin([ego_id_csv, target_id_csv])
                & (df["frameNum"] >= frame_min)
                & (df["frameNum"] <= frame_max)
            )
            segment = df.loc[mask].copy()

            event_id = f"{csv_name}_{metric_name}_{ego_id}_{target_id}_{event_start}"
            segment.insert(0, "event_id", event_id)
            segment["trigger_metric"] = metric_name
            segment[extreme_col] = extreme_value
            all_segments.append(segment)

    if not all_segments:
        return pd.DataFrame()

    result = pd.concat(all_segments, ignore_index=True)
    result.sort_values(by=["event_id", "frameNum", "carId"], inplace=True)
    result.reset_index(drop=True, inplace=True)
    return result


def main():
    parser = argparse.ArgumentParser(
        description="Extract vehicle trajectories around SSM risk events."
    )
    parser.add_argument(
        "--json_path", type=str, required=True,
        help="Path to the SSM results JSON file."
    )
    parser.add_argument(
        "--csv_path", type=str, required=True,
        help="Path to the original trajectory CSV file."
    )
    parser.add_argument(
        "--metrics", type=str, nargs="+", required=True,
        help="One or more metric specs in NAME:THRESHOLD:CONDITION format. "
             "E.g. TTC:2.0:below DRAC:3.0:above PET:1.5:below"
    )
    parser.add_argument(
        "--period", type=int, default=50,
        help="Number of frames before/after the event to extract (default: 50)."
    )
    parser.add_argument(
        "--output_path", type=str, default=None,
        help="Output CSV path. Default: {json_name}_events.csv"
    )
    args = parser.parse_args()

    metric_specs = [parse_metric_spec(s) for s in args.metrics]
    csv_name = os.path.splitext(os.path.basename(args.csv_path))[0]

    data = load_json(args.json_path)

    all_metric_events = []
    total_events = 0
    for spec in metric_specs:
        events = find_risk_events(data, spec["name"], spec["threshold"], spec["condition"])
        all_metric_events.append((spec["name"], spec["condition"], events))
        total_events += len(events)
        print(f"  {spec['name']} (threshold={spec['threshold']}, "
              f"condition={spec['condition']}): {len(events)} events")

    print(f"Total: {total_events} risk events across {len(metric_specs)} metric(s).")

    if total_events == 0:
        print("No risk events found. No output file generated.")
        return

    result = extract_trajectories(args.csv_path, all_metric_events, args.period, csv_name)

    if result.empty:
        print("No trajectory data matched the events. No output file generated.")
        return

    output_path = args.output_path
    if output_path is None:
        json_name = os.path.splitext(os.path.basename(args.json_path))[0]
        output_path = os.path.join(os.path.dirname(args.json_path), f"{json_name}_events.csv")

    result.to_csv(output_path, index=False)
    print(f"Saved {len(result)} rows to {output_path}")


if __name__ == "__main__":
    main()
