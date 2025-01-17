#!/usr/bin/env python3

import scripts.python.dynamic_degradation_viz_module as ddvm
import pandas as pd
import os
import argparse
import sys
import warnings
from joblib import Parallel, delayed

"""Saving numpy arrays in HDF files will give a performance warning, so ignoring it"""
warnings.simplefilter(action='ignore', category=pd.errors.PerformanceWarning)

def find_h5_files(args):
    """Find all the HDF files from a given folder.
    
    Args:
        args: Argument parsed from command line that should contain the member variable FOLDER.
    """

    inf_est_df_filepaths = []
    sensor_acc_df_filepaths = []

    # Parallel implementation of generating DynamicDegradationJsonData objects
    for root, _, files in os.walk(args.FOLDER):

        if not files: continue # no files detected
        else:
            inf_est_dfs = [
                os.path.join(root, filename) for filename in files if filename.endswith(".h5") and filename.startswith(ddvm.INF_EST_DF_PREFIX)
            ]
            sensor_acc_dfs = [
                os.path.join(root, filename) for filename in files if filename.endswith(".h5") and filename.startswith(ddvm.SENSOR_ACC_DF_PREFIX)
            ]

            # Collect the file paths
            if any(inf_est_dfs):
                if args.verbose:
                    print("Found {0} \"{1}*\" files in folder={2}...".format(len(inf_est_dfs), ddvm.INF_EST_DF_PREFIX, root))
                    sys.stdout.flush()

                inf_est_df_filepaths.extend(inf_est_dfs)
            
            if any(sensor_acc_dfs):
                if args.verbose:
                    print("Found {0} \"{1}*\" files in folder={2}...".format(len(sensor_acc_dfs), ddvm.SENSOR_ACC_DF_PREFIX, root))
                    sys.stdout.flush()
                    
                sensor_acc_df_filepaths.extend(sensor_acc_dfs)

    return inf_est_df_filepaths, sensor_acc_df_filepaths

def process_and_compute_rmsd(
    inf_est_df_filepaths: list,
    sensor_acc_df_filepaths: list,
    args
):
    """Load the HDF files to compute the RMSD.
    """
    if (len(inf_est_df_filepaths) != len(sensor_acc_df_filepaths)):
        raise RuntimeError("The number of \"{0}*\" files and \"{1}*\" are not equal.".format(ddvm.INF_EST_DF_PREFIX, ddvm.SENSOR_ACC_DF_PREFIX))

    def parallel_load_compute_rmsd(inf_est_df_path, sensor_acc_df_path):

        # Load HDF files
        inf_est_df = pd.read_hdf(inf_est_df_path)
        sensor_acc_df = pd.read_hdf(sensor_acc_df_path)

        # Compute RMSD
        return ddvm.compute_raw_rmsd(inf_est_df, sensor_acc_df, args.inf_est_type)

    dfs = Parallel(n_jobs=-1, verbose=0)(
        delayed(parallel_load_compute_rmsd)(inf_est_df, sensor_acc_df) for inf_est_df, sensor_acc_df in zip(inf_est_df_filepaths, sensor_acc_df_filepaths)
    )

    df = pd.concat(dfs, axis=0, ignore_index=True)
    df.to_hdf(args.output, key="df" if not args.key else args.key)

    print("Stored RMSD data as {0} with key \"{1}\"".format(os.path.join(os.getcwd(), args.output), "df" if not args.key else args.key))

def main(args):

    inf_est_paths, sensor_acc_paths = find_h5_files(args)
    if inf_est_paths and sensor_acc_paths: process_and_compute_rmsd(inf_est_paths, sensor_acc_paths, args)
    else: print("No dynamic degradation data HDF files found.")

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Compute the RMSD values from raw dynamic degradation HDF files and concatenate into a single HDF file.")
    parser.add_argument("FOLDER", type=str, help="path to the folder containing all the JSON data files; the files can be several levels deep")
    parser.add_argument("--output", default="rmsd_data.h5", type=str, help="output filename (default: rmsd_data.h5)")
    parser.add_argument("--key", default="df", help="dictionary key when storing the Pandas DataFrame (default: \"df\")")
    parser.add_argument("--verbose", action="store_true", help="flag to run with verbose output")
    parser.add_argument("--inf_est_type", type=str, default="weighted", help="choose between \"weighted\" or \"regular\" informed estimates (default: \"weighted\")")
    
    args = parser.parse_args()

    main(args)