#!/usr/bin/env python3

import scripts.python.dynamic_degradation_viz_module as ddvm
import pandas as pd
import os
import argparse
import sys
import itertools as iter
from joblib import Parallel, delayed

def find_rmsd_hdf_files(args):
    
    rmsd_df_filepaths = []

    # Find the RMSD HDF files
    for root, _, files in os.walk(args.FOLDER):

        if not files: continue # no files detected
        else:
            rmsd_dfs = [
                os.path.join(root, filename) for filename in files if filename.endswith(".h5") and (ddvm.RMSD_DF_PREFIX in filename)
            ]

            # Collect the file paths
            if any(rmsd_dfs): rmsd_df_filepaths.extend(rmsd_dfs)

    print("Found RMSD HDF files at:\n", rmsd_df_filepaths)

    return rmsd_df_filepaths

def export_isolated_parameter_hdf_files(args, df_filepath):

    # Load the DataFrame
    df = pd.read_hdf(df_filepath, key=args.key)

    # Check if need to create a column for filter_specific_params
    if type(df["filter_specific_params"].iloc[0]) == dict:
        for title in args.column_titles:
            if title.startswith("fsp_"):
                df[title] = df["filter_specific_params"].apply(lambda x: float(x[title[4:]]))

    # Apply modifications to the LDAL column
    df["lowest_degraded_acc_lvl"] = df["lowest_degraded_acc_lvl"].round(3)

    unique_col_vals = [[] for _ in range(len(args.column_titles))]

    for ind, col_str in enumerate(args.column_titles):
        unique_col_vals[ind] = df[col_str].unique()

    combinations = list(
        iter.product(
            *unique_col_vals
        )
    )

    # Go through each combination to export HDF file
    for comb in combinations:

        filename = "rmsd_"
        boolean_conditions = True

        for c, t in zip(comb, args.column_titles):
        # for ind, col_title in enumerate(args.column_titles):
            filename += "{0}{1}_".format(t, c)
            boolean_conditions = boolean_conditions & (df[t] == c)

        output_name = os.path.join(
            os.path.dirname(df_filepath),
            filename[:-1] + ".h5"
        )

        df[boolean_conditions].to_hdf(output_name, key="df")

        print("Done with {0} = {1}".format(args.column_titles, comb))

def main(args):

    # Find the filepaths
    df_filepaths = find_rmsd_hdf_files(args)

    # Export the HDF files
    Parallel(n_jobs=-1)(
        delayed(export_isolated_parameter_hdf_files)(args, df_filepath) for df_filepath in df_filepaths
    )

if __name__ == "__main__":

    # Find the unique values for the columns of interest
    default_column_title_lst = [
        "true_drift_coeff",
        "lowest_degraded_acc_lvl",
        "fsp_pred_deg_model_B",
        "correct_sensor_acc_b",
        "flawed_sensor_acc_b",
        "tfr"
    ]

    parser = argparse.ArgumentParser(description="(HPC USE ONLY) Extract isolated RMSD data from multiple RMSD HDF files. This outputs HDF files for each unique combination of each parameter.")
    parser.add_argument("FOLDER", type=str, help="path to the folder containing all the RMSD HDF data files; the files can be several levels deep")
    parser.add_argument("--key", default="df", help="dictionary key when storing the Pandas DataFrame (default: \"df\")")
    parser.add_argument("--column_titles", nargs="+", default=default_column_title_lst, help="list of column titles to create combinations of HDF files from (default: [\"true_drift_coeff\", \"lowest_degraded_acc_lvl\", \"fsp_pred_deg_model_B\", \"correct_sensor_acc_b\", \"flawed_sensor_acc_b\", \"tfr\"])")

    args = parser.parse_args()

    main(args)