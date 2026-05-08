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

## Current Processing Tools

At this stage, downstream processing currently includes:

- `ssm_calculator/`: safety surrogate metric computation (SSM)
- Detailed guide: [`ssm_calculator/README.md`](./ssm_calculator/README.md)

This includes the main SSM calculation flow and related analysis utilities built on top of standardized data.


