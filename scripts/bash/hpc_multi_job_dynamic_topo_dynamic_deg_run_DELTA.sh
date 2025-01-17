#!/usr/bin/env bash

# Trap errors and log the line number
trap 'echo "${LINENO}: ${BASH_COMMAND}"; exit 1' ERR

CURR_WORKING_DIR=$(pwd)
SBATCH_SCRIPT_TEMPLATE=sbatch_dynamic_topo_dynamic_deg_run_DELTA.sh
ARGOSFILE=param_multi_robot_sim_1d_dynamic_degradation_DELTA.argos

# Parameters to test
TRUE_DEG_DRIFT=(-25e-6)
DENSITY=(1)
WALL_POSITION=(2.452639)
PRED_DEG_MODEL_B_RANGE=(-10e-6 -25e-6 -40e-6)
LDAL_RANGE=(500 700)
LDAL_DEC_RANGE=(0.5 0.7)

# Fixed parameters (shouldn't be changed typically)
TRUE_DEG_DIFFUSION=1e-4
OBS_QUEUE_SIZE=1000
DYNAMIC_QUEUE=true
WEIGHTED_AVG_INFORMED_EST=true
PRED_DEG_VAR_R=1e-8
INIT_VAR=1e-4
LOWEST_ASSUMED_ACC_LVL=500
LAAL_DEC=0.5
CORRECT_FILTER=false
NUM_ROBOTS=15
NUM_FLAWED_ROBOTS=${NUM_ROBOTS} # set all robots to be flawed so that the filter will be run for all of them
NUM_TRIALS=1
NUM_TICKS=30000
FILTER_PERIOD=5
METHOD=DELTA
VARIANTS=("bin" "lap")

module load slurm

# Modified copied param file
sed -i -E "s/(TRUE_DEG_DIFFUSION=).*/\1${TRUE_DEG_DIFFUSION}/" ${SBATCH_SCRIPT_TEMPLATE}
sed -i -E "s/(OBS_QUEUE_SIZE=).*/\1${OBS_QUEUE_SIZE}/" ${SBATCH_SCRIPT_TEMPLATE}
sed -i -E "s/(DYNAMIC_QUEUE=).*/\1${DYNAMIC_QUEUE}/" ${SBATCH_SCRIPT_TEMPLATE}
sed -i -E "s/(WEIGHTED_AVG_INFORMED_EST=).*/\1${WEIGHTED_AVG_INFORMED_EST}/" ${SBATCH_SCRIPT_TEMPLATE}
sed -i -E "s/(PRED_DEG_VAR_R=).*/\1${PRED_DEG_VAR_R}/" ${SBATCH_SCRIPT_TEMPLATE}
sed -i -E "s/(INIT_VAR=).*/\1${INIT_VAR}/" ${SBATCH_SCRIPT_TEMPLATE}
sed -i -E "s/(LOWEST_ASSUMED_ACC_LVL=).*/\1${LOWEST_ASSUMED_ACC_LVL}/" ${SBATCH_SCRIPT_TEMPLATE}
sed -i -E "s/(LAAL_DEC=).*/\1${LAAL_DEC}/" ${SBATCH_SCRIPT_TEMPLATE}
sed -i -E "s/(CORRECT_FILTER=).*/\1${CORRECT_FILTER}/" ${SBATCH_SCRIPT_TEMPLATE}
sed -i -E "s/(NUM_ROBOTS=).*/\1${NUM_ROBOTS}/" ${SBATCH_SCRIPT_TEMPLATE}
sed -i -E "s/(NUM_FLAWED_ROBOTS=).*/\1${NUM_FLAWED_ROBOTS}/" ${SBATCH_SCRIPT_TEMPLATE}
sed -i -E "s/(NUM_TRIALS=).*/\1${NUM_TRIALS}/" ${SBATCH_SCRIPT_TEMPLATE}
sed -i -E "s/(NUM_TICKS=).*/\1${NUM_TICKS}/" ${SBATCH_SCRIPT_TEMPLATE}
sed -i -E "s/(FILTER_PERIOD=).*/\1${FILTER_PERIOD}/" ${SBATCH_SCRIPT_TEMPLATE}
sed -i -E "s/(METHOD=).*/\1${METHOD}/" ${SBATCH_SCRIPT_TEMPLATE}
sed -i -E "s/(VARIANTS=).*/\1\(${VARIANTS[*]}\)/" ${SBATCH_SCRIPT_TEMPLATE}

for ((i = 0; i < ${#TRUE_DEG_DRIFT[@]}; i++)); do
    sed -i -E "s/(TRUE_DEG_DRIFT=).*/\1${TRUE_DEG_DRIFT[i]}/" ${SBATCH_SCRIPT_TEMPLATE}

    for ((j = 0; j < ${#DENSITY[@]}; j++)); do
        sed -i -E "s/(DENSITY=).*/\1${DENSITY[j]}/" ${SBATCH_SCRIPT_TEMPLATE}
        sed -i -E "s/(WALL_POSITION=).*/\1${WALL_POSITION}/" ${SBATCH_SCRIPT_TEMPLATE}

        for ((k = 0; k < ${#LDAL_RANGE[@]}; k++)); do
            sed -i -E "s/(LOWEST_DEGRADED_ACC_LVL=).*/\1${LDAL_RANGE[k]}/" ${SBATCH_SCRIPT_TEMPLATE}
            sed -i -E "s/(LDAL_DEC=).*/\1${LDAL_DEC_RANGE[k]}/" ${SBATCH_SCRIPT_TEMPLATE}

            for ((l = 0; l < ${#PRED_DEG_MODEL_B_RANGE[@]}; l++)); do
                sed -i -E "s/(PRED_DEG_MODEL_B=).*/\1${PRED_DEG_MODEL_B_RANGE[l]}/" ${SBATCH_SCRIPT_TEMPLATE}

                # Copy the param file
                TARGET_DIR=${CURR_WORKING_DIR}/drift${TRUE_DEG_DRIFT[i]}/den${DENSITY[j]}/ldal${LDAL_RANGE[k]}/modelb${PRED_DEG_MODEL_B_RANGE[l]}
                mkdir -p ${TARGET_DIR}
                cp ${SBATCH_SCRIPT_TEMPLATE} ${TARGET_DIR}
                cp ${ARGOSFILE} ${TARGET_DIR}
                pushd ${TARGET_DIR}
                mkdir -p data

                JOB_NAME=${METHOD}_drift${TRUE_DEG_DRIFT[i]}_den${DENSITY[j]}_ldal${LDAL_RANGE[k]}_modelb${PRED_DEG_MODEL_B_RANGE[l]}

                # Run the job
                # With each core running 1 trial, we should only need about 500M each per core, leaving about 1G left for the waiting core
                sbatch -N 1 -n 32 --mem=16G -p short -o "log_%x_%j.out" -e "log_%x_%j.err" -J ${JOB_NAME} -t 01:00:00 --mail-user=kchin@wpi.edu --mail-type=fail,end ${SBATCH_SCRIPT_TEMPLATE}
            done
        done
    done
done
