
# Standardized Data Transformation


## Data Transfer Tools

This module converts raw trajectory datasets into the NBDT standard format.

### Supported Datasets

- `highD`
- `inD`
- `NGSIM`
- `CitySim`
- `Waymo`

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
  - Dataset key (case-insensitive): `highD`, `inD`, `ngsim`, `citysim`, `Waymo`
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


#### Notes on `Waymo` (Waymo Motion Dataset)

Waymo Motion has no per-frame `lane_id` and no closed lane polygons, so
`laneId` is obtained by **centerline matching**: each vehicle centre is
projected onto every `LaneCenter.polyline` and assigned the nearest lane
within 2 m (else `-1`). Input is one or more `*.tfrecord` shards; requires
`tensorflow` and `waymo-open-dataset`.

Extra columns beyond the standard NBDT schema:

- `scenarioId` — Waymo scenario id (one TFRecord shard contains many scenarios)
- `tfFile` — source TFRecord filename
- `velocity_x`, `velocity_y` — per-frame velocity components (m/s) from Waymo state
- `length`, `width` — per-frame vehicle dimensions (m)
- `obj_type` — Waymo agent type: 0=UNSET, 1=VEHICLE, 2=PEDESTRIAN, 3=CYCLIST, 4=OTHER
- `vtype` — 1=HDV, 2=AV (ego data collector), 3=others
