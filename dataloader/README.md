
# Standardized Data Transformation


## Data Transfer Tools

This module converts raw trajectory datasets into the NBDT standard format.

### Supported Datasets

- `highD`
- `inD`
- `NGSIM`
- `CitySim`

Set up your Python environment first.

Install the dependencies required by this project before running any transfer scripts.
```
pip intsall -r requirements.txt
```

### Start transfer current dataset into standard

Follow these steps:

1. Go to the `dataloader` directory.

```bash
cd dataloader
```

2. Run the transfer script with CLI arguments.

```bash
python factory.py --dataset highD --data_folder ./original_data --save_folder ./processed_data
```

- `--dataset` (`str`, default: `highD`)
  - Dataset key (case-insensitive): `highD`, `inD`, `ngsim`, `citysim`
- `--data_folder` (`str`, default: `./original_data`)
  - Folder containing raw input files
- `--save_folder` (`str`, default: `./processed_data`)
  - Folder where converted CSV files are written
- `--use_yml` (`str`, default: `./config/config.yaml`)
  - YAML file loaded after CLI parsing; values in YAML override CLI/default values

3. If you prefer configuration files, use YAML from `config`.

```bash
python factory.py --use_yml ./config/config.yaml
```

- Example Config (`config/config.yaml`)

```yaml
dataset: highD
data_folder: ./original_data
save_folder: ./processed_data
```

# Surrogate Safety Measures Calculation

This module computes various Surrogate Safety Measures (SSM) from vehicle trajectory data. It is designed to work with the NBDT (Naturalistic Driving Behavior & Trajectory) standard format, but can be adapted to any trajectory dataset with minimal changes.

## Features

Preprocessing – Efficiently loads and preprocesses trajectory data (acceleration, angular velocity, lane direction, projection).

### SSM Computation – Calculates per‑frame and per‑pair safety indicators:

- `TTC` – Time‑to‑Collision (constant speed assumption)

- `MTTC` – Modified Time‑to‑Collision (accounts for acceleration)

- `DRAC` – Deceleration Rate to Avoid Crash

- `CAI` – Crash Index (based on MTTC)

- `PET` – Post‑Encroachment Time (for crossing conflicts)

- `2D_TTC` – Two‑dimensional collision time (based on bounding boxes and angular velocity)

### Aggregated Measures 

`TIT` (Time‑Integrated TTC), `TET` (Time‑Exposed TTC), and `CPI` (Crash Potential Index) are computed for each ego‑target interaction.

### Visualization 

Optionally saves images of scenes where any SSM falls below a user‑defined threshold.

### JSON Output

All results are saved in a structured JSON file for further analysis.

## Installation

Ensure you have Python 3.8+ installed. Install the required dependencies:

```
pip install -r requirements.txt
```

## Usage

### Basic Command

Run the main script with your trajectory and metadata files:

```
python main.py
```

All parameters are defined inside the script (see Configuration below).

### Configuration

Open `main.py` and adjust the following user parameters at the top of the `main()` function:

```
EGO_ID = None               # ego vehicle ID (None = process all vehicles)
TARGET_IDS = []             # list of target IDs (empty = use surrounding vehicles)
START_FRAME = None          # first frame (None = first available frame)
END_FRAME = None            # last frame (None = last available frame)
ENABLE_VISUAL = True        # save images for triggered events
ENABLE_OUTPUT = True        # save JSON results
OUTPUT_DIR = "./output"     # output directory for JSON and images
```

Thresholds for event detection are also configurable:

```
TTC_THRESHOLD = 2.0
MTTC_THRESHOLD = 2.0
PET_THRESHOLD = 2.0
TTC2D_THRESHOLD = 2.0
```

### Input Data Format

The module expects two CSV files:

- `Metadata file` – contains global information. Example columns: `frameRate`, `upperLaneMarkings`, `lowerLaneMarkings`.

- `Tracks file` – per‑frame vehicle records.
Required columns:
`frameNum`, `carId`, `carCenterXm`, `carCenterYm`,
`boundingBox1Xm`, `boundingBox1Ym`, `boundingBox2Xm`, `boundingBox2Ym`, 
`boundingBox3Xm`, `boundingBox3Ym`, `boundingBox4Xm`, `boundingBox4Ym`,
`heading`, `speed`, `objClass`, `laneId`.

Additional columns are allowed but ignored.

If your data does not follow the NBDT standard, you can adapt the load_tracks function in `data_loader.py` accordingly.

## Output

### JSON Structure

When `ENABLE_OUTPUT = True`, a JSON file named `{tracks_filename}_results.json` is created in the specified `OUTPUT_DIR`. The structure is as follows:

```
{
  "meta": { ... },  // metadata (frame rate, thresholds, lane markings)
  "vehicles": {
    "ego_id": {
      "targets": {
        "target_id": {
          "relation": "front",  // relative position at start
          "start_frame": 100,
          "end_frame": 150,
          "frames": [           // per‑frame data
            {
              "frame": 101,
              "relation": "front",
              "TTC": 2.5,
              "MTTC": 2.3,
              "DRAC": 1.2,
              "CAI": 15.7,
              "PET": null,
              "2D_TTC": null
            },
            ...
          ],
          "aggregated": {
            "TIT": 0.234,       // Time‑Integrated TTC
            "TET": 0.056        // Time‑Exposed TTC
          }
        }
      },
      "cpi": 0.018              // Crash Potential Index
    }
  }
}
```

- All numeric values are stored as JSON numbers; `null` indicates that the value was undefined or infinite for that frame.

- The `relation` field in each frame reflects the instantaneous relative position (may differ from the overall relation).

## Visualizations

If `ENABLE_VISUAL = True`, images of scenes where any SSM falls below the corresponding threshold are saved under `OUTPUT_DIR/visualization/egoId_targetId/`. Each image contains the bounding boxes, nearest points, and a title listing the triggered indicators.


