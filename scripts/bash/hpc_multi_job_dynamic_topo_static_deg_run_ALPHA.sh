#!/bin/bash

# This script requires the low-level `sbatch_dynamic_topo_static_deg_run.sh` job script to successfully execute the 
# dynamic-topology-dynamic-degradation experiments. It iterates over at least 2 parameters: density and filter period.
# This script is only suitable to run experiments using the ALPHA filter. Additionally, this script whether
# also modifies the low-level script on whether to activate the filter for all robots -- that stays constant throughout
# all iterations. After this script is done with its modifications, the `sbatch_dynamic_topo_static_deg_run.sh` script
# iterates over the different flawed_acc-correct_acc-TFR-flawed_robot combinations within a single job.

# The script works by first modifying the variables in `sbatch_dynamic_topo_static_deg_run.sh` then copying it to a target
# directory ${TARGET_DIR} before executing the `sbatch_dynamic_topo_static_deg_run.sh` script in said directory.

CURR_WORKING_DIR=$(pwd)
SBATCH_SCRIPT_TEMPLATE=sbatch_dynamic_topo_static_deg_run.sh
ARGOSFILE=param_multi_robot_sim_1d_static_degradation.argos

# Parameters to test
CORRECT_FILTER=true
DENSITY=(1 10)
WALL_POSITION=(2.82433 0.92732)

# Fixed parameters (shouldn't be changed typically)
FLAWED_ROBOT_RANGE=(2 2 10)
NUM_ROBOTS=20
NUM_TRIALS=30
NUM_STEPS=4.0e+4
FILTER_PERIOD=(1000 2000 4000)
METHOD=ALPHA

module load slurm

# Modify copied param file
sed -i "s/CORRECT_FILTER=.*/CORRECT_FILTER=${CORRECT_FILTER}/" ${SBATCH_SCRIPT_TEMPLATE}
sed -i "s/FLAWED_ROBOT_RANGE=.*/FLAWED_ROBOT_RANGE=(${FLAWED_ROBOT_RANGE[0]} ${FLAWED_ROBOT_RANGE[1]} ${FLAWED_ROBOT_RANGE[2]})/" ${SBATCH_SCRIPT_TEMPLATE}
sed -i "s/NUM_ROBOTS=.*/NUM_ROBOTS=${NUM_ROBOTS}/" ${SBATCH_SCRIPT_TEMPLATE}
sed -i "s/WALL_POSITION=.*/WALL_POSITION=${WALL_POSITION}/" ${SBATCH_SCRIPT_TEMPLATE}
sed -i "s/METHOD=.*/METHOD=${METHOD}/" ${SBATCH_SCRIPT_TEMPLATE}

for (( i = 0; i < ${#DENSITY[@]}; i++ ))
do
    for (( j = 0; j < ${#FILTER_PERIOD[@]}; j++ ))
    do
        # Modify the parameters
        sed -i "s/WALL_POSITION=.*/WALL_POSITION=${WALL_POSITION[i]}/" ${SBATCH_SCRIPT_TEMPLATE}
        sed -i "s/DENSITY=.*/DENSITY=${DENSITY[i]}/" ${SBATCH_SCRIPT_TEMPLATE}
        sed -i "s/FILTER_PERIOD=.*/FILTER_PERIOD=${FILTER_PERIOD[j]}/" ${SBATCH_SCRIPT_TEMPLATE}

        # Copy param file
        TARGET_DIR=${CURR_WORKING_DIR}/filtp${FILTER_PERIOD[j]}/den${DENSITY[i]}/alpha
        mkdir -p ${TARGET_DIR}
        cp ${SBATCH_SCRIPT_TEMPLATE} ${TARGET_DIR}
        cp ${ARGOSFILE} ${TARGET_DIR}
        pushd ${TARGET_DIR}
        mkdir -p data

        JOB_NAME=${METHOD}_corfilt${CORRECT_FILTER}_filtp${FILTER_PERIOD[j]}_den${DENSITY[i]}

        # Run the job
        sbatch -N 1 -n 2 --mem=8G -p short -o "log_%x_%j.out" -e "log_%x_%j.err" -J ${JOB_NAME} -t 16:00:00 --mail-user=kchin@wpi.edu --mail-type=fail,end ${SBATCH_SCRIPT_TEMPLATE}
        popd
    done
done