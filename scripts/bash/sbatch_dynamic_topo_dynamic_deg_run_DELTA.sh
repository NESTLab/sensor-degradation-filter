#!/usr/bin/env bash

# This script is modeled after the `sbatch_dynamic_topo_static_deg_run.sh` script and is meant to be executed by a high-level
# script as an `sbatch` command. It should be copied by said script to the intended working directory before execution. The
# high-level script will modify the variables for
# (not on an HPC). Here we run the two DELTA variants using the same random seed; each trial is run sequentially. The experiments
# are only run across one set of
#       `ground_sensor` parameters except true accuracy (`period_ticks`, `sensor_acc_*`, `true_deg_*_coeff`)
#       `sensor_degradation_filter` parameters (except `variant`, where both "bin" and "lap" are run)
#       `loop_functions` parameters (`num_trials`, `arena_tiles`, `target_fill_ratio`, `flawed_robots`, etc.)
#       swarm density (`arena` parameters)

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

# Trap errors and log the line number
trap 'echo "${LINENO}: ${BASH_COMMAND}"; exit 1' ERR

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

# Path to the LOCAL param_multi_robot_sim_1d_dynamic_degradation.argos (copied by the high-level script)
# (Adapt this to your needs)
TEMPLATE_ARGOSFILE=${DATADIR}/param_multi_robot_sim_1d_dynamic_degradation_DELTA.argos

########################################
#
# Job-related variables
#
########################################

# Parameters related to this job
# (To be changed by the high level script)
DENSITY=1                      # desired swarm density
WALL_POSITION=2.452639         # computed from desired density
TRUE_DEG_DRIFT=-25e-6          # true sensor degradation drift coefficient (Wiener model)
TRUE_DEG_DIFFUSION=1e-4        # true sensor degradation drift coefficient (Wiener model)
LOWEST_DEGRADED_ACC_LVL=500    # lowest true accuracy value that the sensor will degrade to, in 1000s format
LDAL_DEC=0.5                   # lowest true accuracy value in decimal format (bash doesn't support decimal math)
OBS_QUEUE_SIZE=1000            # max observation queue size
DYNAMIC_QUEUE=true             # whether to use dynamic-sized observation queues
WEIGHTED_AVG_INFORMED_EST=true # whether to use the weighted informed estimates in the filter
PRED_DEG_MODEL_B=-5e-5         # state prediction model B
PRED_DEG_VAR_R=1e-8            # state prediction variance R
INIT_VAR=1e-4                  # initial variance for the filter (the initial mean will be the initial assumed sensor accuracy)
LOWEST_ASSUMED_ACC_LVL=500     # lowest assumed accuracy value that the filter will estimate, in 1000s format
LAAL_DEC=0.5                   # lowest assumed accuracy value in decimal format (bash doesn't support decimal math)

# Parameters related to this job
# (Fixed)
NUM_ROBOTS=15
NUM_FLAWED_ROBOTS=${NUM_ROBOTS} # set all robots to be flawed so that the filter will be run for all of them
CORRECT_FILTER=false            # whether non-flawed robots should run the filter (this only applies for *non-flawed* robots)
if [[ "${CORRECT_FILTER}" == "true" ]]; then CORFILT=1; else CORFILT=0; fi
TRUE_ACC=(999 800)
TRUE_ACC_DEC=(0.999 0.8)
ASSUMED_ACC=(999 800) # when flawed robots num is set to 0 this isn't applied
ASSUMED_ACC_DEC=(0.999 0.8)
TFR=(550 750 950)
TFR_DEC=(0.55 0.75 0.95)
SEEDS=(
    290436
    181119
    38912
    154705
    155515
    608132
    132498
    866484
    266445
    189590
    770323
    395632
    353363
    276233
    416059
    568733
    409278
    671822
    625856
    469425
    576648
    91012
    326029
    810690
    553891
    45200
    905509
    540361
    345030
    452015
)            # the randomly pre-generated list of seeds (so that all the experiments are the same across the two variants)
METHOD=DELTA # filter type
NUM_TICKS=30000
SPEED=14.14
COMMS_PERIOD=5
VERBOSITY=none
TICKS_PER_SECOND=10
NUM_TILES=100
WALL_THICKNESS=0.1
ARENA_LEN=5
DYNAMIC_DEGRADATION=true # dynamic sensor degradation
MEAS_PERIOD=5
FILTER_PERIOD=5
VARIANTS=("bin" "lap")
NUM_TRIALS=1                                            # must be 1 because the we want each trial to use a particular seed
UNPROCESSED_DATA_FILE="flw${NUM_FLAWED_ROBOTS}_t0.json" # used to search the output data file

########################################
#
# Job directory
#
########################################

THISJOB=dynamic_topo_dynamic_deg_DELTA

# Job working directory
# (Don't change this)
WORKDIR=${LOCALDIR}/${MYUSER}/${THISJOB}/${SLURM_JOB_ID}

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

ACTUAL_ARGOSFILE=temp.argos
cp ${TEMPLATE_ARGOSFILE} ${ACTUAL_ARGOSFILE}

# Configure ground sensor
sed -i -E "/<ground_sensor/,/\/>/ s/([[:space:]]*period_ticks=\")[^\"]*/\1${MEAS_PERIOD}/" ${ACTUAL_ARGOSFILE}
sed -i -E "/<ground_sensor/,/\/>/ s/([[:space:]]*dynamic=\")[^\"]*/\1${DYNAMIC_DEGRADATION}/" ${ACTUAL_ARGOSFILE}
sed -i -E "/<ground_sensor/,/\/>/ s/([[:space:]]*true_deg_drift_coeff=\")[^\"]*/\1${TRUE_DEG_DRIFT}/" ${ACTUAL_ARGOSFILE}
sed -i -E "/<ground_sensor/,/\/>/ s/([[:space:]]*true_deg_diffusion_coeff=\")[^\"]*/\1${TRUE_DEG_DIFFUSION}/" ${ACTUAL_ARGOSFILE}
sed -i -E "/<ground_sensor/,/\/>/ s/([[:space:]]*lowest_degraded_acc_lvl=\")[^\"]*/\1${LDAL_DEC}/" ${ACTUAL_ARGOSFILE}

# Configure communication period
sed -i "s/<comms period_ticks=.*/<comms period_ticks=\"${COMMS_PERIOD}\" \/>/" ${ACTUAL_ARGOSFILE}

# Configure speed
sed -i -E "/<wheel_turning/,/\/>/ s/(max_speed=\")[^\"]*/\1${SPEED}/" ${ACTUAL_ARGOSFILE}

# Configure filter
sed -i -E "/<sensor_degradation_filter/,/<\/sensor_degradation_filter>/ s/(method=\")[^\"]*/\1${METHOD}/" ${ACTUAL_ARGOSFILE}
sed -i -E "/<sensor_degradation_filter/,/<\/sensor_degradation_filter>/ s/(period_ticks=\")[^\"]*/\1${FILTER_PERIOD}/" ${ACTUAL_ARGOSFILE}
sed -i -E "/<sensor_degradation_filter/,/<\/sensor_degradation_filter>/ s/(observation_queue_size=\")[^\"]*/\1${OBS_QUEUE_SIZE}/" ${ACTUAL_ARGOSFILE}
sed -i -E "/<sensor_degradation_filter/,/<\/sensor_degradation_filter>/ s/(dynamic_observation_queue=\")[^\"]*/\1${DYNAMIC_QUEUE}/" ${ACTUAL_ARGOSFILE}
sed -i -E "/<sensor_degradation_filter/,/<\/sensor_degradation_filter>/ s/(use_weighted_avg_informed_est=\")[^\"]*/\1${WEIGHTED_AVG_INFORMED_EST}/" ${ACTUAL_ARGOSFILE}
sed -i -E "/<sensor_degradation_filter/,/<\/sensor_degradation_filter>/ s/(pred_deg_model_B=\")[^\"]*/\1${PRED_DEG_MODEL_B}/" ${ACTUAL_ARGOSFILE}
sed -i -E "/<sensor_degradation_filter/,/<\/sensor_degradation_filter>/ s/(pred_deg_var_R=\")[^\"]*/\1${PRED_DEG_VAR_R}/" ${ACTUAL_ARGOSFILE}
sed -i -E "/<sensor_degradation_filter/,/<\/sensor_degradation_filter>/ s/(init_var=\")[^\"]*/\1${INIT_VAR}/" ${ACTUAL_ARGOSFILE}
sed -i -E "/<sensor_degradation_filter/,/<\/sensor_degradation_filter>/ s/(lowest_assumed_acc_lvl=\")[^\"]*/\1${LAAL_DEC}/" ${ACTUAL_ARGOSFILE}

# Configure loop functions
sed -i -E "/<loop_functions/,/<\/loop_functions>/ s/(num_trials value=\")[^\"]*/\1${NUM_TRIALS}/" ${ACTUAL_ARGOSFILE}
sed -i -E "/<loop_functions/,/<\/loop_functions>/ s/(arena_tiles tile_count_x=\")[^\"]*(\"[[:space:]]*tile_count_y=\")[^\"]*/\1${NUM_TILES}\2${NUM_TILES}/" ${ACTUAL_ARGOSFILE}
sed -i -E "/<loop_functions/,/<\/loop_functions>/ s/(verbosity level=\")[^\"]*/\1${VERBOSITY}/" ${ACTUAL_ARGOSFILE}

# Configure density (wall positions and robot starting locations)
sed -i "s/<arena size.*/<arena size=\"${ARENA_LEN}, ${ARENA_LEN}, 1\" center=\"0, 0, 0.5\">/" ${ACTUAL_ARGOSFILE} # arena size
sed -i "/<box id=\"wall_north\".*/{n;d}" ${ACTUAL_ARGOSFILE}                                                      # remove the line after "wall_north"
sed -i "/<box id=\"wall_south\".*/{n;d}" ${ACTUAL_ARGOSFILE}                                                      # remove the line after "wall_south"
sed -i "/<box id=\"wall_east\".*/{n;d}" ${ACTUAL_ARGOSFILE}                                                       # remove the line after "wall_east"
sed -i "/<box id=\"wall_west\".*/{n;d}" ${ACTUAL_ARGOSFILE}                                                       # remove the line after "wall_west"

sed -i "s/<box id=\"wall_north\".*/<box id=\"wall_north\" size=\"${ARENA_LEN},${WALL_THICKNESS},0.5\" movable=\"false\">/" ${ACTUAL_ARGOSFILE} # north wall size
sed -i "s/<box id=\"wall_south\".*/<box id=\"wall_south\" size=\"${ARENA_LEN},${WALL_THICKNESS},0.5\" movable=\"false\">/" ${ACTUAL_ARGOSFILE} # south wall size
sed -i "s/<box id=\"wall_east\".*/<box id=\"wall_east\" size=\"${WALL_THICKNESS},${ARENA_LEN},0.5\" movable=\"false\">/" ${ACTUAL_ARGOSFILE}   # east wall size
sed -i "s/<box id=\"wall_west\".*/<box id=\"wall_west\" size=\"${WALL_THICKNESS},${ARENA_LEN},0.5\" movable=\"false\">/" ${ACTUAL_ARGOSFILE}   # west wall size

sed -i "s/<box id=\"wall_north\".*/<box id=\"wall_north\" size=\"${ARENA_LEN},${WALL_THICKNESS},0.5\" movable=\"false\">\n            <body position=\"0,${WALL_POSITION},0\" orientation=\"0,0,0\" \/>/" ${ACTUAL_ARGOSFILE}
sed -i "s/<box id=\"wall_south\".*/<box id=\"wall_south\" size=\"${ARENA_LEN},${WALL_THICKNESS},0.5\" movable=\"false\">\n            <body position=\"0,-${WALL_POSITION},0\" orientation=\"0,0,0\" \/>/" ${ACTUAL_ARGOSFILE}
sed -i "s/<box id=\"wall_east\".*/<box id=\"wall_east\" size=\"${WALL_THICKNESS},${ARENA_LEN},0.5\" movable=\"false\">\n            <body position=\"${WALL_POSITION},0,0\" orientation=\"0,0,0\" \/>/" ${ACTUAL_ARGOSFILE}
sed -i "s/<box id=\"wall_west\".*/<box id=\"wall_west\" size=\"${WALL_THICKNESS},${ARENA_LEN},0.5\" movable=\"false\">\n            <body position=\"-${WALL_POSITION},0,0\" orientation=\"0,0,0\" \/>/" ${ACTUAL_ARGOSFILE}

# Configure area of robot starting positions
sed -i "s/<position method=\"uniform\".*/<position method=\"uniform\" min=\"-${WALL_POSITION},-${WALL_POSITION},0\" max=\"${WALL_POSITION},${WALL_POSITION},0\" \/>/" ${ACTUAL_ARGOSFILE}

# Configure number of robots
sed -i "s/<entity.*/<entity quantity=\"${NUM_ROBOTS}\" max_trials=\"100\" base_num=\"0\">/" ${ACTUAL_ARGOSFILE}

{
    START_TIME=$(date +%m/%d/%Y-%H:%M:%S)
    echo -e "\n################################### EXECUTION BEGIN ###################################"
    echo -e "################################# ${START_TIME} #################################\n"

    # Iterate over assumed sensor accuracies
    for ((i = 0; i < ${#ASSUMED_ACC[@]}; i++)); do
        sed -i -E "/<loop_functions/,/<\/loop_functions>/ s/(flawed_robots[[:space:]]*num=\")[^\"]*(\"[[:space:]]*acc_b=\")[^\"]*(\"[[:space:]]*acc_w=\")[^\"]*(\"[[:space:]]*activate_filter_for_all=\")[^\"]*/\1${NUM_FLAWED_ROBOTS}\2${ASSUMED_ACC_DEC[i]}\3${ASSUMED_ACC_DEC[i]}\4${CORRECT_FILTER}/" ${ACTUAL_ARGOSFILE}
        sed -i -E "/<sensor_degradation_filter/,/<\/sensor_degradation_filter>/ s/(init_mean=\")[^\"]*/\1${ASSUMED_ACC_DEC[i]}/" ${ACTUAL_ARGOSFILE}

        # Iterate over true sensor accuracies
        for ((j = 0; j < ${#TRUE_ACC[@]}; j++)); do
            sed -i -E "/<ground_sensor/,/\/>/ s/([[:space:]]*sensor_acc_b=\")[^\"]*/\1${TRUE_ACC_DEC[j]}/" ${ACTUAL_ARGOSFILE}
            sed -i -E "/<ground_sensor/,/\/>/ s/([[:space:]]*sensor_acc_w=\")[^\"]*/\1${TRUE_ACC_DEC[j]}/" ${ACTUAL_ARGOSFILE}

            # Iterate over target fill ratios
            for ((k = 0; k < ${#TFR[@]}; k++)); do
                sed -i -E "/<loop_functions/,/<\/loop_functions>/ s/(target_fill_ratio value=\")[^\"]*/\1${TFR_DEC[k]}/" ${ACTUAL_ARGOSFILE}

                # Current date and time
                CURR_DATETIME=$(date "+%m%d%y_%H%M%S")

                JSON_FOLDER="${CURR_DATETIME}_t${#SEEDS[@]}_s${NUM_TICKS}_tfr${TFR[k]}_flw${NUM_FLAWED_ROBOTS}_flwb${ASSUMED_ACC[i]}_corb${TRUE_ACC[j]}_drift${TRUE_DEG_DRIFT}_diff${TRUE_DEG_DIFFUSION}_ldal${LOWEST_DEGRADED_ACC_LVL}_modelb${PRED_DEG_MODEL_B}_modelr${PRED_DEG_VAR_R}_laal${LOWEST_ASSUMED_ACC_LVL}_commsp${COMMS_PERIOD}_filtp${FILTER_PERIOD}_corfilt${CORFILT}"
                mkdir -p data/${JSON_FOLDER}

                # Define configuration file name
                PARAM_FILE=tfr${TFR[k]}_flw${NUM_FLAWED_ROBOTS}_flwb${ASSUMED_ACC[i]}_corb${TRUE_ACC[j]}.argos

                # Iterate over variants
                for ((l = 0; l < ${#VARIANTS[@]}; l++)); do
                    # Create directories for different variants
                    mkdir -p data/${JSON_FOLDER}/${VARIANTS[l]}_variant

                    # Modify variant type
                    sed -i -E "/<sensor_degradation_filter/,/<\/sensor_degradation_filter>/ s/(variant=\")[^\"]*/\1${VARIANTS[l]}/" ${ACTUAL_ARGOSFILE}

                    # Iterate over different seeds (each seed is for one trial)
                    for ((m = ${#SEEDS[@]} - 1; m >= 0; m--)); do
                        # Create folder for each seed (even though only one data file generated, as multi-threaded run would overwrite the data before we can change its filename)
                        mkdir -p data/${JSON_FOLDER}/${VARIANTS[l]}_variant/${m}

                        # Configure experiment length and seed
                        sed -i "s/<experiment.*/<experiment length=\"$((${NUM_TICKS} / ${TICKS_PER_SECOND}))\" ticks_per_second=\"${TICKS_PER_SECOND}\" random_seed=\"${SEEDS[m]}\" \/>/" ${ACTUAL_ARGOSFILE}

                        # Configure output path
                        sed -i -E "/<loop_functions/,/<\/loop_functions>/ s/(path folder=\")[^\"]*/\1data\/${JSON_FOLDER}\/${VARIANTS[l]}_variant\/${m}/" ${ACTUAL_ARGOSFILE}

                        sleep 1.0

                        # Run the following in a subshell in multi-threaded mode
                        (
                            cp ${ACTUAL_ARGOSFILE} data/${JSON_FOLDER}/${VARIANTS[l]}_variant/${m}/exp.argos

                            # Run the experiment
                            srun --exclusive --ntasks=1 --mem-per-cpu=500M run_dynamic_topo_simulations -l /dev/null -c data/${JSON_FOLDER}/${VARIANTS[l]}_variant/${m}/exp.argos

                            # Modify the JSON data file so that `trial_ind` value reflects the correct trial index
                            sed -i -E "s/([[:space:]]*\"num_trials\"[[:space:]]*:[[:space:]]*)[0-9]+/\1${#SEEDS[@]}/" data/${JSON_FOLDER}/${VARIANTS[l]}_variant/${m}/${UNPROCESSED_DATA_FILE}
                            sed -i -E "s/([[:space:]]*\"trial_ind\"[[:space:]]*:[[:space:]]*)0/\1${m}/" data/${JSON_FOLDER}/${VARIANTS[l]}_variant/${m}/${UNPROCESSED_DATA_FILE}

                            # Rename and move the file
                            mv data/${JSON_FOLDER}/${VARIANTS[l]}_variant/${m}/${UNPROCESSED_DATA_FILE} data/${JSON_FOLDER}/${VARIANTS[l]}_variant/flw${NUM_FLAWED_ROBOTS}_t${m}.json
                        ) &
                    done

                    # Let all the background processes complete
                    wait
                done

                # Transfer data into target data directory (including parameter files)
                # (this is to prevent overflowing the /local disk space)
                mv data/${JSON_FOLDER} ${DATADIR}/data
            done
        done
    done

    END_TIME=$(date +%m/%d/%Y-%H:%M:%S)
    echo -e "\n################################### EXECUTION END ###################################"
    echo -e "################################# ${END_TIME} #################################\n"
}
