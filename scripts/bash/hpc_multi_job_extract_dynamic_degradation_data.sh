#!/usr/bin/env bash

JOB_TYPE=extract_dynamic_degradation_data

TRUE_DEG_DRIFT=(0 -10e-6)
DENSITY=(1)
LDAL=(500 700)
PRED_DEG_MODEL_B_RANGE=(-5e-6 -10e-6 -15e-6)

module load slurm

EXECUTABLE_SCRIPT=extract_dynamic_degradation_data.py

# Activate virtual environment
PYTHON_VENV_ACT_BIN=/home/kchin/sensor-degradation-filter/.venv/bin/activate
source ${PYTHON_VENV_ACT_BIN}

for ((i = 0; i < ${#TRUE_DEG_DRIFT[@]}; i++)); do
    for ((j = 0; j < ${#DENSITY[@]}; j++)); do
        for ((k = 0; k < ${#LDAL[@]}; k++)); do
            for ((l = 0; l < ${#PRED_DEG_MODEL_B_RANGE[@]}; l++)); do
                NEW_WORKING_DIR=drift${TRUE_DEG_DRIFT[i]}/den${DENSITY[j]}/ldal${LDAL[k]}/modelb${PRED_DEG_MODEL_B_RANGE[l]}/
                pushd ${NEW_WORKING_DIR}

                JOB_NAME=${JOB_TYPE}_drift${TRUE_DEG_DRIFT[i]}_den${DENSITY[j]}_ldal${LDAL[k]}_modelb${PRED_DEG_MODEL_B_RANGE[l]}

                # Run the job
                # (The memory usage can go higher, but the --mem flag asks for minimum amount, and there's no upper limit)
                sbatch -N 1 -n 32 --mem=40G -p short -o "log_%x_%j.out" -e "log_%x_%j.err" -J ${JOB_NAME} -t 01:00:00 --mail-user=kchin@wpi.edu --mail-type=fail,end --wrap "${EXECUTABLE_SCRIPT} . --output ${JOB_NAME}.h5 --verbose"
                popd
            done
        done
    done
done
