#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Parquet to CSV Converter for LeRobot VLA Dataset

Converts LeRobot parquet files to CSV format for easier analysis and visualization.

Usage:
    python3 parquet_to_csv.py <parquet_file> [output_csv]
    python3 parquet_to_csv.py ~/lerobot_datasets/openarm_bimanual/data/train-00000.parquet

If output_csv is not specified, it will be saved as <parquet_file>.csv
"""
import argparse
import os
import sys
from pathlib import Path

import pandas as pd
import pyarrow.parquet as pq


# Joint names for column headers
JOINT_NAMES = [
    'left_rev1', 'left_rev2', 'left_rev3', 'left_rev4',
    'left_rev5', 'left_rev6', 'left_rev7', 'left_rev8',
    'right_rev1', 'right_rev2', 'right_rev3', 'right_rev4',
    'right_rev5', 'right_rev6', 'right_rev7', 'right_rev8',
]


def parquet_to_csv(parquet_path: str, csv_path: str = None, expand_arrays: bool = True):
    """
    Convert parquet file to CSV.
    
    Args:
        parquet_path: Path to input parquet file
        csv_path: Path to output CSV file (optional)
        expand_arrays: If True, expand observation.state and action arrays into separate columns
    
    Returns:
        DataFrame of converted data
    """
    parquet_path = Path(parquet_path).expanduser()
    
    if not parquet_path.exists():
        raise FileNotFoundError(f"Parquet file not found: {parquet_path}")
    
    # Default output path
    if csv_path is None:
        csv_path = parquet_path.with_suffix('.csv')
    else:
        csv_path = Path(csv_path).expanduser()
    
    print(f"📂 Reading: {parquet_path}")
    
    # Read parquet
    table = pq.read_table(parquet_path)
    df = table.to_pandas()
    
    print(f"   Rows: {len(df)}")
    print(f"   Columns: {list(df.columns)}")
    
    if expand_arrays:
        # Expand observation.state array into individual columns
        if 'observation.state' in df.columns:
            obs_data = df['observation.state'].tolist()
            for i, joint_name in enumerate(JOINT_NAMES):
                df[f'state.{joint_name}'] = [row[i] if i < len(row) else 0.0 for row in obs_data]
            df = df.drop(columns=['observation.state'])
        
        # Expand action array into individual columns
        if 'action' in df.columns:
            action_data = df['action'].tolist()
            for i, joint_name in enumerate(JOINT_NAMES):
                df[f'action.{joint_name}'] = [row[i] if i < len(row) else 0.0 for row in action_data]
            df = df.drop(columns=['action'])
    
    # Reorder columns for better readability
    base_cols = ['timestamp', 'frame_index', 'episode_index']
    state_cols = [f'state.{j}' for j in JOINT_NAMES if f'state.{j}' in df.columns]
    action_cols = [f'action.{j}' for j in JOINT_NAMES if f'action.{j}' in df.columns]
    other_cols = [c for c in df.columns if c not in base_cols + state_cols + action_cols]
    
    ordered_cols = []
    for c in base_cols:
        if c in df.columns:
            ordered_cols.append(c)
    ordered_cols.extend(state_cols)
    ordered_cols.extend(action_cols)
    ordered_cols.extend(other_cols)
    
    df = df[ordered_cols]
    
    # Save to CSV
    df.to_csv(csv_path, index=False)
    
    print(f"✅ Saved: {csv_path}")
    print(f"   Columns: {len(df.columns)}")
    
    return df


def convert_all_in_directory(data_dir: str, output_dir: str = None):
    """
    Convert all parquet files in a directory to CSV.
    
    Args:
        data_dir: Directory containing parquet files
        output_dir: Output directory for CSV files (default: same as data_dir)
    """
    data_dir = Path(data_dir).expanduser()
    output_dir = Path(output_dir).expanduser() if output_dir else data_dir
    
    parquet_files = list(data_dir.glob('*.parquet'))
    
    if not parquet_files:
        print(f"❌ No parquet files found in: {data_dir}")
        return
    
    print(f"📁 Found {len(parquet_files)} parquet file(s)")
    
    for pq_file in parquet_files:
        csv_file = output_dir / pq_file.with_suffix('.csv').name
        parquet_to_csv(pq_file, csv_file)
        print()


def main():
    parser = argparse.ArgumentParser(
        description='Convert LeRobot parquet files to CSV format',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s ~/lerobot_datasets/openarm_bimanual/data/train-00000.parquet
  %(prog)s ~/lerobot_datasets/openarm_bimanual/data/train-00000.parquet output.csv
  %(prog)s --dir ~/lerobot_datasets/openarm_bimanual/data/
  %(prog)s --no-expand ~/lerobot_datasets/openarm_bimanual/data/train-00000.parquet
        """
    )
    
    parser.add_argument('parquet_file', nargs='?', 
                        help='Path to parquet file to convert')
    parser.add_argument('output_csv', nargs='?', default=None,
                        help='Output CSV path (optional)')
    parser.add_argument('--dir', '-d', type=str, default=None,
                        help='Convert all parquet files in directory')
    parser.add_argument('--no-expand', action='store_true',
                        help='Do not expand arrays into separate columns')
    
    args = parser.parse_args()
    
    if args.dir:
        convert_all_in_directory(args.dir)
    elif args.parquet_file:
        parquet_to_csv(args.parquet_file, args.output_csv, expand_arrays=not args.no_expand)
    else:
        parser.print_help()
        print("\n❌ Error: Please provide a parquet file or use --dir option")
        sys.exit(1)


if __name__ == '__main__':
    main()
