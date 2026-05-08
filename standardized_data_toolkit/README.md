# Standardized Data Toolkit

This directory contains processing tools that are designed to work on a unified, standardized trajectory data format.

## Data Format First

All downstream tools in this folder assume the same standard data schema.  
This design keeps the analysis pipeline consistent, reusable, and easier to extend.

## How to Get the Standardized Data

The standardized format can be generated from existing datasets using:

- `dataloader/`: dataset-to-standard-format converter
- Detailed guide: [`dataloader/README.md`](./dataloader/README.md)

Use `dataloader` to transform heterogeneous source datasets into the project’s standard trajectory CSV format before running downstream analysis tools.

## Dataset Metadata and References

To understand dataset-specific field mappings, coordinate systems, and normalization notes, use:

- `dataset_metadata.md`: consolidated metadata documentation
- Detailed reference: [`dataset_metadata.md`](./dataset_metadata.md)

Reference figures for coordinate systems and related illustrations are stored in:

- `images/`: supporting diagrams used by metadata and conversion docs

## Current Processing Tools

At this stage, downstream processing currently includes:

- `ssm_calculator/`: safety surrogate metric computation (SSM)
- Detailed guide: [`ssm_calculator/README.md`](./ssm_calculator/README.md)

This includes the main SSM calculation flow and related analysis utilities built on top of standardized data.

## Shared Utilities

Reusable helper scripts are grouped in:

- `utils/`: shared utility code for loading data, SSM-related helpers, JSON output, and visualization support

Current utility scripts include:

- `box_distance.py`
- `data_loader.py`
- `main.py`
- `output_json.py`
- `ssm.py`
- `visualizer.py`

These utilities support quick experiments and common reusable logic across the toolkit.

