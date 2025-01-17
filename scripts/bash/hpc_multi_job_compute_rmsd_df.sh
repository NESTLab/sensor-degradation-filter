#!/usr/bin/env bash

JOB_TYPE=rmsd

TRUE_DEG_DRIFT=(0 -25e-6)
DENSITY=(1)
LDAL=(500 700)
PRED_DEG_MODEL_B=(-10e-6 -25e-6 -40e-6)

module load slurm

EXECUTATBLE_SCRIPT=compute_rmsd_df.py

# Activate virtual environment
PYTHON_VENV_ACT_BIN=/home/kchin/sensor-degradation-filter/.venv/bin/activate
source ${PYTHON_VENV_ACT_BIN}

for ((i = 0; i < ${#TRUE_DEG_DRIFT[@]}; i++)); do
    for ((j = 0; j < ${#DENSITY[@]}; j++)); do
        for ((k = 0; k < ${#LDAL[@]}; k++)); do
            for ((l = 0; l < ${#PRED_DEG_MODEL_B[@]}; l++)); do
                NEW_WORKING_DIR=drift${TRUE_DEG_DRIFT[i]}/den${DENSITY[j]}/ldal${LDAL[k]}/modelb${PRED_DEG_MODEL_B[l]}
                pushd ${NEW_WORKING_DIR}

                JOB_NAME=${JOB_TYPE}_drift${TRUE_DEG_DRIFT[i]}_den${DENSITY[j]}_ldal${LDAL[k]}_modelb${PRED_DEG_MODEL_B[l]}

                # Run the job
                # (The memory usage can go higher, but the --mem flag asks for minimum amount, and there's no upper limit)
                sbatch -N 1 -n 16 --mem=64G -p short -o "log_%x_%j.out" -e "log_%x_%j.err" -J ${JOB_NAME} -t 04:00:00 --mail-user=kchin@wpi.edu --mail-type=fail,end --wrap "${EXECUTATBLE_SCRIPT} . --output ${JOB_NAME}.h5 --verbose"
                popd
            done
        done
    done
done
