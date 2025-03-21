#!/usr/bin/env bash

# This script is meant to be executed by a high-level script as an `sbatch` command, and should be copied by said script
# to the intended working directory before execution. The high-level script will modify the variables for density, whether
# to activate the filter for all robots, and the filter period in this script, thus keeping them constant within a single
# job. Meanwhile, this script iterates different flawed_acc-correct_acc-TFR-flawed_robot combinations within a single job
# run.

# The script works by first modifying a template parameter file ${ARGOSFILE}, then making a copy of it with a specific
# name ${PARAM_FILE} in the new working directory WORKDIR=${LOCALDIR}/${MYUSER}/${THISJOB}/${SLURM_JOB_ID}. The actual
# working directory starts in the folder the high-level script starts this job in, then it moves to ${WORKDIR} to run the
# experiments. Once complete, all the created files in ${WORKDIR} will be copied over back to the original directory
# where this job script was started in.

# Stop execution after any error
set -e

# Cleanup function to be executed upon exit, for any reason
function cleanup() {
    rm -rf $WORKDIR
}



########################################
#
# Useful variables
#
########################################

# Your user name
# (Don't change this)
MYUSER=$(whoami)

# Path of the local storage on a node
# Use this to avoid sending data streams over the network
# (Don't change this)
LOCALDIR=/local

# ARGoS environment variables
# (Don't change this)
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/sensor-degradation-filter/build_argos/src
export ARGOS_PLUGIN_PATH=$ARGOS_PLUGIN_PATH:$HOME/sensor-degradation-filter/build_argos/src
export PATH=$PATH:$HOME/sensor-degradation-filter/build_argos/src

# Folder where you want your data to be stored
# (Adapt this to your needs)
DATADIR=$(pwd)

# Path to the LOCAL param_multi_robot_sim_1d_static_degradation.argos (copied by the high-level script)
# (Adapt this to your needs)
ARGOSFILE=${DATADIR}/param_multi_robot_sim_1d_static_degradation.argos



########################################
#
# Job-related variables
#
########################################

# Parameters related to this job
# (To be changed by the high level script)
CORRECT_FILTER=true
FLAWED_ROBOT_RANGE=(2 2 10) # array of 3 elements: (min increment max)
NUM_ROBOTS=20
DENSITY=10
WALL_POSITION=2.824329 # related to density
FILTER_PERIOD=1000
METHOD=ALPHA
TYPE_2_ERR_PROB=0.05 # only applicable for BRAVO filter

# Parameters related to this job
# (Fixed)
FLAWED=(550 750 950)
CORRECT=(550 750 950)
TFR=(550 750 950)
FLAWED_DEC=(0.55 0.75 0.95)
CORRECT_DEC=(0.55 0.75 0.95)
TFR_DEC=(0.55 0.75 0.95)
NUM_TICKS=40000
SPEED=14.14
NUM_TRIALS=30
MEAS_PERIOD=1
COMMS_PERIOD=1
VERBOSITY=none
TICKS_PER_SECOND=10
NUM_TILES=1000
WALL_THICKNESS=0.1
ARENA_LEN=10

# Job id
# (Change this to reflect the above parameters)
if [[ "${CORRECT_FILTER}" == "true" ]]; then CORFILT=1; else CORFILT=0; fi
FLW_RANGE=${FLAWED_ROBOT_RANGE[0]}-${FLAWED_ROBOT_RANGE[1]}-${FLAWED_ROBOT_RANGE[2]}
THISJOB=dynamic_topo_static_deg

# Job working directory
# (Don't change this)
WORKDIR=${LOCALDIR}/${MYUSER}/${THISJOB}/${SLURM_JOB_ID}



########################################
#
# Job directory
#
########################################

# Create work dir from scratch, enter it
# (Don't change this)
rm -rf $WORKDIR && mkdir -p $WORKDIR && cd $WORKDIR

# Make sure you cleanup upon exit
# (Don't change this)
trap cleanup EXIT SIGINT SIGTERM



########################################
#
# Actual job logic
#
########################################

# Configure experiment length
sed -i "s/<experiment.*/<experiment length=\"$(( ${NUM_TICKS} / ${TICKS_PER_SECOND} ))\" ticks_per_second=\"${TICKS_PER_SECOND}\" random_seed=\"0\" \/>/" ${ARGOSFILE}

# Configure communication period
sed -i "s/<comms period_ticks=.*/<comms period_ticks=\"${COMMS_PERIOD}\" \/>/" ${ARGOSFILE}

# Configure speed
sed -i "s/max_speed=.*/max_speed=\"${SPEED}\" \/>/" ${ARGOSFILE}

# Configure filter
sed -i "s/<sensor_degradation_filter.*/<sensor_degradation_filter method=\"${METHOD}\" period_ticks=\"${FILTER_PERIOD}\" observation_queue_size=\"0\" use_weighted_avg_informed_est=\"false\" dynamic_observation_queue=\"false\">/" ${ARGOSFILE}
if [[ ${METHOD} == "BRAVO" ]]
then
    sed -i "s/<params type_2_err_prob=.*/<params type_2_err_prob=\"${TYPE_2_ERR_PROB}\" \/>/" ${ARGOSFILE}
fi

# Configure number of trials
sed -i "s/<num_trials.*/<num_trials value=\"${NUM_TRIALS}\" \/>/" ${ARGOSFILE}

# Configure number of arena tiles
sed -i "s/<arena_tiles.*/<arena_tiles tile_count_x=\"${NUM_TILES}\" tile_count_y=\"${NUM_TILES}\" \/>/" ${ARGOSFILE}

# Configure verbosity level
sed -i "s/<verbosity.*/<verbosity level=\"${VERBOSITY}\" \/>/" ${ARGOSFILE}

# Configure density (wall positions and robot starting locations)
sed -i "s/<arena size.*/<arena size=\"${ARENA_LEN}, ${ARENA_LEN}, 1\" center=\"0,0,0.5\">/" ${ARGOSFILE} # arena size
sed -i "/<box id=\"wall_north\".*/{n;d}" ${ARGOSFILE} # remove the line after "wall_north"
sed -i "/<box id=\"wall_south\".*/{n;d}" ${ARGOSFILE} # remove the line after "wall_south"
sed -i "/<box id=\"wall_east\".*/{n;d}" ${ARGOSFILE} # remove the line after "wall_east"
sed -i "/<box id=\"wall_west\".*/{n;d}" ${ARGOSFILE} # remove the line after "wall_west"

sed -i "s/<box id=\"wall_north\".*/<box id=\"wall_north\" size=\"10,${WALL_THICKNESS},0.5\" movable=\"false\">/" ${ARGOSFILE} # north wall size
sed -i "s/<box id=\"wall_south\".*/<box id=\"wall_south\" size=\"10,${WALL_THICKNESS},0.5\" movable=\"false\">/" ${ARGOSFILE} # south wall size
sed -i "s/<box id=\"wall_east\".*/<box id=\"wall_east\" size=\"${WALL_THICKNESS},10,0.5\" movable=\"false\">/" ${ARGOSFILE} # east wall size
sed -i "s/<box id=\"wall_west\".*/<box id=\"wall_west\" size=\"${WALL_THICKNESS},10,0.5\" movable=\"false\">/" ${ARGOSFILE} # west wall size

sed -i "s/<box id=\"wall_north\".*/<box id=\"wall_north\" size=\"10,${WALL_THICKNESS},0.5\" movable=\"false\">\n            <body position=\"0,${WALL_POSITION},0\" orientation=\"0,0,0\" \/>/" ${ARGOSFILE}
sed -i "s/<box id=\"wall_south\".*/<box id=\"wall_south\" size=\"10,${WALL_THICKNESS},0.5\" movable=\"false\">\n            <body position=\"0,-${WALL_POSITION},0\" orientation=\"0,0,0\" \/>/" ${ARGOSFILE}
sed -i "s/<box id=\"wall_east\".*/<box id=\"wall_east\" size=\"${WALL_THICKNESS},10,0.5\" movable=\"false\">\n            <body position=\"${WALL_POSITION},0,0\" orientation=\"0,0,0\" \/>/" ${ARGOSFILE}
sed -i "s/<box id=\"wall_west\".*/<box id=\"wall_west\" size=\"${WALL_THICKNESS},10,0.5\" movable=\"false\">\n            <body position=\"-${WALL_POSITION},0,0\" orientation=\"0,0,0\" \/>/" ${ARGOSFILE}

# Configure area of robot starting positions
sed -i "s/<position method=\"uniform\".*/<position method=\"uniform\" min=\"-${WALL_POSITION},-${WALL_POSITION},0\" max=\"${WALL_POSITION},${WALL_POSITION},0\" \/>/" ${ARGOSFILE}

# Configure number of robots
sed -i "s/<entity.*/<entity quantity=\"${NUM_ROBOTS}\" max_trials=\"100\" base_num=\"0\">/" ${ARGOSFILE}
{
    START_TIME=$(date +%m/%d/%Y-%H:%M:%S)
    echo -e "\n################################### EXECUTION BEGIN ###################################"
    echo -e "################################# ${START_TIME} #################################\n"

    # Iterate over flawed sensor accuracies
    for (( i = 0; i < ${#FLAWED[@]}; i++ ))
    do
        # Iterate over correct sensor accuracies
        for (( j = 0; j < ${#CORRECT[@]}; j++ ))
        do
            # Iterate over target fill ratios
            for (( k = 0; k < ${#TFR[@]}; k++ ))
            do
                # Configure target fill ratio
                sed -i "s/<target_fill_ratio.*/<target_fill_ratio value=\"${TFR_DEC[k]}\" \/>/" ${ARGOSFILE}

                if (( ${FLAWED[i]} != ${CORRECT[j]} ))
                then
                    # Current date and time
                    CURR_DATETIME=$(date "+%m%d%y_%H%M%S")

                    # Define JSON folder name for current iteration
                    FLWB=${FLAWED[i]}
                    FLWW=${FLAWED[i]}
                    CORB=${CORRECT[j]}
                    CORW=${CORRECT[j]}

                    JSON_FOLDER=${CURR_DATETIME}_t${NUM_TRIALS}_s${NUM_TICKS}_tfr${TFR[k]}_flw${FLW_RANGE}_flwb${FLWB}_flwc${FLWW}_corb${CORB}_corw${CORW}_commsp${COMMSP}_filtp${FILTER_PERIOD}_corfilt${CORFILT}
                    mkdir -p data/${JSON_FOLDER}

                    # Configure actual sensor accuracy
                    sed -i "s/<ground_sensor period_ticks=.*/<ground_sensor period_ticks=\"${MEAS_PERIOD}\" sensor_acc_b=\"${CORRECT_DEC[j]}\" sensor_acc_w=\"${CORRECT_DEC[j]}\" sim=\"true\" dynamic=\"false\" true_deg_drift_coeff=\"0.0\" true_deg_diffusion_coeff=\"-1.0\" lowest_degraded_acc_lvl=\"0.0\" \/>/" ${ARGOSFILE}

                    # Iterate over number of flawed robots
                    for l in $(seq ${FLAWED_ROBOT_RANGE[0]} ${FLAWED_ROBOT_RANGE[1]} ${FLAWED_ROBOT_RANGE[2]})
                    do
                        # Configure flawed sensor accuracy
                        sed -i "s/<flawed_robots.*/<flawed_robots num=\"${l}\" acc_b=\"${FLAWED_DEC[i]}\" acc_w=\"${FLAWED_DEC[i]}\" activate_filter_for_all=\"${CORRECT_FILTER}\" \/>/" ${ARGOSFILE}

                        # Configure output path
                        sed -i "s/<path folder=.*/<path folder=\"data\/${JSON_FOLDER}\" \/>/" ${ARGOSFILE}

                        # Define configuration file name
                        PARAM_FILE=tfr${TFR[k]}_flw${l}_flwb${FLWB}_flwc${FLWW}_corb${CORB}_corw${CORW}.argos

                        # Create the modifed configuration file in the current folder
                        cp ${ARGOSFILE} ${PARAM_FILE}

                        # Run the experiment in parallel (each core runs a single experiment for a specific number of flawed robots)
                        # With 40,000 steps, maxRSS is lower than 4G, but we use 4.2G just to be safe
                        srun --exclusive --ntasks=1 --mem-per-cpu=4200M run_dynamic_topo_simulations -l /dev/null -c ${PARAM_FILE} &
                    done

                    # Let all the background processes complete
                    wait

                    # Transfer data into target data directory (including parameter files)
                    # (this is to prevent overflowing the /local disk space)
                    mv data/${JSON_FOLDER} *.argos ${DATADIR}/data
                fi
            done
        done
    done

    END_TIME=$(date +%m/%d/%Y-%H:%M:%S)
    echo -e "\n################################### EXECUTION END ###################################"
    echo -e "################################# ${END_TIME} #################################\n"
}