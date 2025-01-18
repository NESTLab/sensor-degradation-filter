#!/bin/bash

# This scripts extracts the convergence and accuracy data from the JSON files into a single Pandas DataFrame (per job).
# The DataFrame contains the convergence and accuracy data from experiments of a particular filter period and
# density. This means that for each flawed_acc-correct_acc-TFR-flawed_robot(-type_2_err_prob) combination, all trials from
# that combination is stored as a single row in the DataFrame.

JOB_TYPE=conv_acc_data_bravo_corfilt1

DENSITY=(1 10)
FILTER_PERIOD=(1000 2000 4000)

module load slurm

EXECUTABLE_SCRIPT=extract_convergence_accuracy_data.py

# Activate virtual environment
PYTHON_VENV_ACT_BIN=/home/kchin/sensor-degradation-filter/.venv/bin/activate
source ${PYTHON_VENV_ACT_BIN}

for (( i = 0; i < ${#DENSITY[@]}; i++ ))
do
    for (( j = 0; j < ${#FILTER_PERIOD[@]}; j++ ))
    do
        NEW_WORKING_DIR=filtp${FILTER_PERIOD[j]}/den${DENSITY[i]}/
        pushd ${NEW_WORKING_DIR}

        JOB_NAME=${JOB_TYPE}_filtp${FILTER_PERIOD[j]}_den${DENSITY[i]}

        # Run the job
        # (The memory usage is actually as high as 90Gb, but the --mem flag asks for minimum amount, and there's no upper limit)
	    sbatch -N 1 -n 32 --mem=64G -p short -o "log_%x_%j.out" -e "log_%x_%j.err" -J ${JOB_NAME} -t 04:00:00 --mail-user=kchin@wpi.edu --mail-type=fail,end --wrap "${EXECUTABLE_SCRIPT} . 0.01 --output ${JOB_NAME}.h5 --verbose"
        popd
    done
done