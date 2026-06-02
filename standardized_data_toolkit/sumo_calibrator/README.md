# SUMO Calibrator

`sumo_calibrator` calibrates SUMO car-following and lane-change parameters from trajectory CSV data with a genetic algorithm, and provides tools to:

- calibrate IDM, Krauss, and LC2013 parameters in a single run,
- grid-search GA training parameters to find the best score for a given model,
- emit SUMO-ready `<vType>` XML and JSON summaries for downstream simulation.

## 1. What This Module Does

### SUMO model calibration

Car-following (per-sample acceleration RMSE):

- `IDM` — outputs SUMO `carFollowModel="IDM"` parameters: `v0`, `tau`, `minGap`, `accel`, `decel`, `delta`.
- `Krauss` — outputs SUMO `carFollowModel="Krauss"` parameters: `maxSpeed`, `tau`, `minGap`, `accel`, `decel`, `sigma`.

Lane-change (per-sample logloss against observed lane changes):

- `LC2013` — outputs SUMO `laneChangeModel="LC2013"` parameters: `lcStrategic`, `lcCooperative`, `lcSpeedGain`, `lcKeepRight`, `lcAssertive`, `lcImpatience`.

### Calibration modes

- `calibrate`: run one GA pass per selected model and emit the calibrated parameters.
- `optimize`: grid-search GA training hyperparameters for one specified model and keep the trial with the lowest objective.

## 2. File Overview

- `sumo_calibrator/main.py`: main entry point for SUMO calibration.
- `sumo_calibrator/analyzer.py`: data loading, sample extraction, `Calibrator` and `GridSearchOptimizer` orchestrators.
- `sumo_calibrator/sumo_models.py`: IDM / Krauss / LC2013 evaluators, parameter bounds, GA runner.
- `sumo_calibrator/example_config.json`: config template.

SUMO model parameter reference: <https://gitee.com/itsncut/trafficsimulation/blob/master/sumo/SUMO%E4%B8%AD%E7%9A%84%E8%BD%A6%E8%BE%86%E7%B1%BB%E5%9E%8B%E8%B7%9F%E9%A9%B0%E6%8D%A2%E9%81%93%E6%A8%A1%E5%9E%8B%E8%AE%BE%E7%BD%AE.md>

Trajectory data format reference: <https://github.com/ZhilingResearch/Ozone/wiki/TrajectoryDataFormat>

## 3. Input Data Requirements

Expected trajectory CSV columns (metric fields only — pixel columns `carCenterX/Y` are not used):

- `frameNum`, `carId`
- `carCenterXm`, `carCenterYm`
- `boundingBox1Xm`, `boundingBox1Ym`
- `boundingBox2Xm`, `boundingBox2Ym`
- `boundingBox3Xm`, `boundingBox3Ym`
- `boundingBox4Xm`, `boundingBox4Ym`
- `course`, `speed`, `laneId`

Extra columns are allowed and ignored. The default frame rate is 30 FPS; `frame_step` downsamples by frame index (`frame_step = 6` keeps every 6th frame, i.e. 5 Hz).

Python dependencies: `pandas`, `numpy`, `deap`.

## 4. Run SUMO Calibration

Run from the project root:

```bash
python -m sumo_calibrator.main \
  --config sumo_calibrator/example_config.json \
  --out ./output/calibration
```

If `--out` is omitted, the default output directory is `calibration_output`.

### Config fields

Minimal required fields:

- `mode`: `calibrate` or `optimize`
- `model`: `all` (calibrate mode only), or one of `idm`, `krauss`, `lc2013`

Input fields:

- `input_dir` (default `input`): directory scanned for `*.csv` when `csv_files` is empty
- `csv_files`: optional explicit list of CSV paths

Sample-extraction fields under `sample`:

- `frame_step` (default `6`): use every Nth frame; 6 means 5 Hz for 30 FPS data
- `max_rows_per_file` (default `0`): row cap per file, `0` means all rows
- `max_cf_samples` (default `12000`): cap on car-following samples
- `max_lc_samples` (default `12000`): cap on lane-change samples
- `sample_seed` (default `42`): random seed for sub-sampling

GA fields under `ga` (used in `calibrate` mode):

- `population` (default `24`)
- `generations` (default `18`)
- `mutation_rate` (default `0.25`)
- `seed` (default `42`)

Grid-search fields under `grid_search` (used in `optimize` mode):

- `population_candidates` (default `[12, 24]`)
- `generation_candidates` (default `[10, 18]`)
- `mutation_rate_candidates` (default `[0.15, 0.25, 0.35]`)
- `seed_candidates` (default `[42]`)

## 5. Calibrate Output Structure

In `calibrate` mode, the output directory contains:

- `sumo_vtypes.add.xml`: SUMO additional file with calibrated `<vType>` entries. Only the models selected by `model` are included; the `LC2013` block is merged onto each `<vType>` when both car-following and lane-change models are calibrated.
- `calibrated_params.json`: per-model parameter dicts and a `summary` block.
- `samples_summary.txt`: flat key-value summary (sample counts, errors, runtime).

The `summary` block in `calibrated_params.json` contains:

- `model`, `models`: requested model selection
- `input_files`, `frame_step`, `dt_seconds`
- `rows_read`, `sampled_rows`, `frame_min`, `frame_max`, `lanes`
- `car_following_samples`, `lane_change_samples`
- per-model objective values: `idm_accel_rmse`, `krauss_accel_rmse`, `lc2013_logloss`
- `runtime_seconds`

## 6. Optimize Output Structure

In `optimize` mode, the output directory contains:

- `best_model_params.json`: the best-trial parameters and objective score.
- `training_params.json`: the GA hyperparameters that produced the best trial, the full search space, per-trial scores, sample/baseline metrics, fitness history, and runtime.
- `best_model_params.add.xml`: SUMO additional file containing just the best `<vType>`.

`training_params.json` includes a `baseline_metrics` block useful for diagnosing whether GA search is the bottleneck:

- `zero_accel_rmse`, `mean_accel_rmse` (car-following)
- `lane_change_rate`, `constant_rate_logloss` (lane-change)

Notes:

- LC2013 logloss is calibrated only on the intercept against the observed lane-change rate, so the score evaluates ranking / separation rather than forcing an unrealistically high base lane-change probability.
- The car-following objective is the RMSE between predicted and observed accelerations on the sampled rows; predictions are clipped to `[-10, 6]` m/s² before comparison.

## 7. SUMO Usage Example

```xml
<additional>
  <vType id="ga_idm" carFollowModel="IDM" ... />
  <vType id="ga_krauss" carFollowModel="Krauss" ... />
</additional>
```

The emitted XML can be passed to SUMO via `--additional-files`. When both a car-following model and `LC2013` are calibrated, lane-change attributes are merged onto each car-following `<vType>`.

## 8. Diagnosing Large Errors

- Inspect `baseline_metrics` in `training_params.json`. If the model RMSE is close to `zero_accel_rmse`, the trajectory acceleration itself is noisy and additional GA search will yield little.
- For IDM / Krauss, prefer `frame_step = 12` or `15` over the default `6` to reduce speed-difference noise that inflates observed acceleration.
- Tune `max_rows_per_file`, `max_cf_samples`, `max_lc_samples` to change sample size and coverage.
- For LC2013, the `lane_change_rate` is usually very low; compare model logloss against `constant_rate_logloss`. If the gap is small, single-frame proxy features do not separate lane changes well — couple SUMO closed-loop simulation into the fitness function for further improvement.
- Increasing `population_candidates` and `generation_candidates` only addresses under-searching, not sample noise or proxy mismatch.

## 9. Recommended Workflow

1. Run `main.py` in `calibrate` mode for a first pass across IDM / Krauss / LC2013.
2. Inspect `samples_summary.txt`. If a specific model's score is unsatisfactory, switch to `optimize` mode for that model.
3. Use the resulting `sumo_vtypes.add.xml` or `best_model_params.add.xml` directly as a SUMO additional file.

## 10. Parameter Bounds

Search bounds live in `sumo_models.py` as `IDM_BOUNDS`, `KRAUSS_BOUNDS`, and `LC2013_BOUNDS`. Tuple order matches `model_parameter_names(model)`:

- IDM: `v0`, `tau`, `minGap`, `accel`, `decel`
- Krauss: `maxSpeed`, `tau`, `minGap`, `accel`, `decel`, `sigma`
- LC2013: `lcStrategic`, `lcCooperative`, `lcSpeedGain`, `lcKeepRight`, `lcAssertive`, `lcImpatience`

The car-following bounds are intentionally wider than SUMO defaults to lower offline RMSE. The fitted parameters should still be sanity-checked in a SUMO closed-loop run for trajectory stability.
