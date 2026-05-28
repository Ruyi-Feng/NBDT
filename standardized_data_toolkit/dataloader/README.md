
# Standardized Data Transformation


## Data Transfer Tools

This module converts raw trajectory datasets into the NBDT standard format.

### Supported Datasets

- `highD`
- `inD`
- `NGSIM`
- `CitySim`
- `Waymo`
- `Lyft`

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
  - Dataset key (case-insensitive): `highD`, `inD`, `ngsim`, `citysim`, `Waymo`, `Lyft`
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

`objClass` mapping (NBDT standard):

| Waymo `obj_type` | Label       | `objClass` |
|-----------------|-------------|-----------|
| 1               | VEHICLE     | 0 (car / generic vehicle) |
| 2               | PEDESTRIAN  | 5 |
| 3               | CYCLIST     | 4 |
| 0 / 4           | UNSET/OTHER | -1 |

> Waymo does not subdivide VEHICLE into car / taxi / truck / bus, so all
> VEHICLE agents are mapped to `objClass=0`. AV vs HDV is indicated by the
> extra `vtype` column (see **AV / HDV Indicator Columns** below).

Extra columns beyond the standard NBDT schema:

- `scenarioId` — Waymo scenario id (one TFRecord shard contains many scenarios)
- `tfFile` — source TFRecord filename
- `velocity_x`, `velocity_y` — per-frame velocity components (m/s) from Waymo state
- `length`, `width` — per-frame vehicle dimensions (m)
- `obj_type` — raw Waymo agent type: 0=UNSET, 1=VEHICLE, 2=PEDESTRIAN, 3=CYCLIST, 4=OTHER
- `vehicleRole` — unified AV/HDV indicator: 0=non-vehicle agent, 1=HDV, 2=AV/ego (see **AV / HDV Indicator Column** below)

---

#### Notes on `Lyft` (Lyft Level-5 Prediction Dataset)

> **Data availability**: The Lyft Level-5 dataset is no longer available for
> public download from the official source. Raw zarr archives must be obtained
> from parties who already hold them. Requires `l5kit` (`pip install l5kit`).

Expected directory layout under `--data_folder` (= `L5KIT_DATA_FOLDER`):

```
<data_folder>/
    <name>.zarr/          ← one or more zarr scene archives
    semantic_map/
        semantic_map.pb
        meta.json
```

```bash
python factory.py --dataset Lyft --data_folder /path/to/lyft_root --save_folder ./processed_data
```

Optional YAML config keys:

```yaml
dataset: Lyft
data_folder: /path/to/lyft_root
save_folder: ./processed_data
cfg_path: /path/to/lyft_map_config.yaml   # default: <data_folder>/lyft_map_config.yaml
num_scenes: 100                           # default: all scenes in the zarr
```

`objClass` mapping (NBDT standard):

| Lyft perception label | `objClass` |
|-----------------------|-----------|
| CAR, VAN              | 0 |
| BUS                   | 2 |
| TRUCK                 | 3 |
| MOTORCYCLE, MOTORCYCLIST | 4 |
| PEDESTRIAN            | 5 |
| other / unknown       | -1 |

Extra columns beyond the standard NBDT schema:

- `vehicleRole` — unified AV/HDV indicator: 0=surrounding agent, 2=AV/ego (see **AV / HDV Indicator Column** below)

Lyft provides no GPS data; `carCenterLon` and `carCenterLat` are always `-1`.
Pixel coordinates (`carCenterX/Y`, `boundingBox*X/Y`) are also `-1` because
the dataset supplies only metric world coordinates.

---

### objClass Reference (NBDT Standard)

Defined in the NBDT wiki. New types are appended to this table when added.

| `objClass` | Vehicle type | Datasets |
|-----------|-------------|---------|
| -1 | Undifferentiated / unknown | all (fallback) |
| 0  | Car (generic vehicle for Waymo) | highD, inD, NGSIM, Waymo, Lyft |
| 1  | Taxi | *(reserved, no current dataset)* |
| 2  | Bus | UTE, Lyft |
| 3  | Truck | highD, inD, NGSIM, Lyft |
| 4  | Motorcycle / cyclist / bicycle | NGSIM, Waymo, Lyft, inD |
| 5  | Pedestrian | inD, Waymo, Lyft |

### AV / HDV Indicator Column

`objClass` describes vehicle category only. Whether a vehicle is an AV (CAV)
or a human-driven vehicle (HDV) is recorded in the unified `vehicleRole` column,
shared by all datasets that carry this information:

| `vehicleRole` | Meaning | Datasets |
|---|---|---|
| 0 | Surrounding / non-vehicle agent (role unconfirmed or not applicable) | Lyft (all non-ego), Waymo (pedestrian / cyclist) |
| 1 | HDV — confirmed human-driven vehicle | Waymo |
| 2 | AV / ego data collector | Waymo, Lyft |

> **Lyft note**: surrounding agents are not individually labelled as HDV or AV,
> so they all receive `vehicleRole=0`. Value `1` (confirmed HDV) is not
> produced by `LyftTransfer`.
>
> **Waymo note**: Waymo distinguishes ego AV (`vehicleRole=2`), other HDVs
> (`vehicleRole=1`), and non-vehicle agents such as pedestrians and cyclists
> (`vehicleRole=0`).

If a future dataset needs additional role values, append them to this table
and document them here.
