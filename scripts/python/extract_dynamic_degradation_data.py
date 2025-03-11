#!/usr/bin/env python3

import scripts.python.dynamic_degradation_viz_module as ddvm
import pandas as pd
import os
import argparse
import timeit
import sys
import warnings
from joblib import Parallel, delayed

"""Saving numpy arrays in HDF files will give a performance warning, so ignoring it"""
warnings.simplefilter(action='ignore', category=pd.errors.PerformanceWarning)

def load_data(args):
    """Load the JSON data from a given folder
    
    Args:
        args: Argument parsed from command line that should contain the member variable FOLDER.
    """
    json_data_lst = []

    # Parallel implementation of generating DynamicDegradationJsonData objects
    def parallel_generate_json_data_lst(root, files):

        if not files: return None # no files detected
        else:
            json_files = [filename for filename in files if filename.endswith(".json")]
            if any(json_files):
                if args.verbose:
                    print("Creating a DynamicDegradationJsonData object for {0} JSON files in folder={1}...".format(len(json_files), root))
                    sys.stdout.flush()

                return ddvm.DynamicDegradationJsonData(root)

    # Extract JSON data objects in parallel
    json_data_lst = Parallel(n_jobs=-1, verbose=0)(
        delayed(parallel_generate_json_data_lst)(root, files) for root, _, files in os.walk(args.FOLDER)
    )

    # Clean up the `None`s in the list
    return [obj for obj in json_data_lst if obj is not None]

def save_to_h5(json_data_lst, args):
    inf_est_df = pd.DataFrame()
    sensor_acc_df = pd.DataFrame()

    start = None

    for i, d in enumerate(json_data_lst):
        if args.verbose:
            start = timeit.default_timer()
            print("Extracting data {0}/{1}...".format(i+1, len(json_data_lst)), end=" ")
            sys.stdout.flush()

        inf_est_df, sensor_acc_df = ddvm.extract_dynamic_degradation_data_to_dfs(d, inf_est_df, sensor_acc_df)

        if args.verbose:
            print("Done!")
            end = timeit.default_timer()
            print("\t-- Elapsed time:", end-start, "s --\n")


    # Split into TFR, CORB, and FLWB partitions
    unique_tfrs = inf_est_df["tfr"].unique()
    unique_flwbs = inf_est_df["flawed_sensor_acc_b"].unique()
    unique_corbs = inf_est_df["correct_sensor_acc_b"].unique()

    # Verify that the sensor_acc_df is the same
    if ((sensor_acc_df["tfr"].unique() != unique_tfrs).all() or
        (sensor_acc_df["flawed_sensor_acc_b"].unique() != unique_flwbs).all() or
        (sensor_acc_df["correct_sensor_acc_b"].unique() != unique_corbs).all()):
        raise ValueError("The unique values of either \"tfr\", \"flawed_sensor_acc_b\" or \"correct_sensor_acc_b\" are not the same between the `inf_est` and `sensor_acc` DataFrames.")

    for t in unique_tfrs:
        for f in unique_flwbs:
            for c in unique_corbs:

                suffix = "_tfr{0}_flwb{1}_corb{2}".format(t, f, c)

                inf_est_df[
                    (inf_est_df["tfr"] == t) &
                    (inf_est_df["flawed_sensor_acc_b"] == f) &
                    (inf_est_df["correct_sensor_acc_b"] == c)
                ].to_hdf(ddvm.INF_EST_DF_PREFIX + os.path.splitext(args.output)[0] + suffix + ".h5", key="df" if not args.key else args.key)

                sensor_acc_df[
                    (sensor_acc_df["tfr"] == t) &
                    (sensor_acc_df["flawed_sensor_acc_b"] == f) &
                    (sensor_acc_df["correct_sensor_acc_b"] == c)
                ].to_hdf(ddvm.SENSOR_ACC_DF_PREFIX + os.path.splitext(args.output)[0] + suffix + ".h5", key="df" if not args.key else args.key)

    print("Stored data as {0} with key \"{1}\"".format(os.path.join(os.getcwd(), "<PREFIX>" + args.output), "df" if not args.key else args.key))

def main(args):

    json_data_lst = load_data(args)
    if json_data_lst: save_to_h5(json_data_lst, args)
    else: print("No JSON files found.")

if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description="Extract time series data from simulated DYNAMIC DEGRADATION experiments to save into HDF files.\n"
        "The data is stored into a pair of Pandas DataFrame for each unique combination of \"tfr\", \"flawed_sensor_acc_b\", and \"correct_sensor_acc_b\".\n"
        "WARNING: Trials of the same experiment must be stored in the same directory; otherwise, the extracted data will contain duplicates.",
        formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("FOLDER", type=str, help="path to the folder containing all the JSON data files; the files can be several levels deep")
    parser.add_argument("--output", default="dynamic_degradation_data.h5", type=str, help="output filename suffix (default: dynamic_degradation_data.h5)")
    parser.add_argument("--key", default="df", help="dictionary key when storing the Pandas DataFrame (default: \"df\")")
    parser.add_argument("--verbose", action="store_true", help="flag to run with verbose output")
    
    args = parser.parse_args()

    main(args)