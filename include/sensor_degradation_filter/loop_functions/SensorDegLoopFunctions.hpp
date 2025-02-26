#ifndef SENSOR_DEG_LOOP_FUNCTIONS_HPP
#define SENSOR_DEG_LOOP_FUNCTIONS_HPP

#include <filesystem>
#include <algorithm>

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_entity.h>
#include <argos3/plugins/simulator/entities/rab_equipped_entity.h>

#include "Util.hpp"
#include "json.hpp"
#include "Arena.hpp"
#include "controllers/KheperaIVDiffusionMotion.hpp"

using namespace argos;
using namespace nlohmann;

using RobotIdDataStrMap = std::unordered_map<std::string, std::vector<std::string>>;

struct ExperimentParams
{
    UInt32 NumRobots = 0;

    UInt32 NumFlawedRobots = 0;

    UInt32 NumTrials = 0;

    UInt32 CommsPeriod = 0;

    UInt32 MeasurementPeriod = 0;

    UInt32 FilterPeriod = 0;

    UInt32 ObservationQueueSize = 0;

    UInt64 NumSteps = 0;

    UInt32 MaxPlacementTrials = 0;

    UInt32 DynamicObservationQueueWindowSize = false;

    std::unordered_map<std::string, std::string> FilterSpecificParams;

    std::unordered_map<std::string, Real> ActualSensorAcc = {{"b", -1.0}, {"w", -1.0}};

    std::unordered_map<std::string, Real> AssumedSensorAcc = {{"b", -1.0}, {"w", -1.0}};

    Real CommsRange = -1.0;

    Real RobotSpeed = -1.0;

    Real Density = -1.0;

    Real TargetFillRatio = -1.0;

    Real GroundSensorDriftCoeff = -1.0;

    Real GroundSensorDiffusionCoeff = -1.0;

    Real LowestDegradedAccuracyLevel = -1.0;

    bool DistributeRobotPlacement = false;

    bool FilterActiveForAll = false;

    bool DynamicDegradation = false;

    bool DynamicObservationQueue = false;

    std::string FilterMethod = "Unset";

    std::string SaveFolder = "Unset";

    std::shared_ptr<RealNumberGenerator> PositionPlacementGeneratorPtr;

    std::shared_ptr<RealNumberGenerator> OrientationPlacementGeneratorPtr;
};

class SensorDegLoopFunctions : public CLoopFunctions
{
public:
    /**
     * @brief Default destructor
     *
     */
    virtual ~SensorDegLoopFunctions()
    {
    }

    /**
     * @brief Initialize loop functions
     *
     * @param t_tree Pointer to the XML config node
     */
    virtual void Init(TConfigurationNode &t_tree);

    /**
     * @brief Reset loop functions (triggered by simulation reset)
     *
     */
    inline void Reset();

    /**
     * @brief Setup experiment
     *
     */
    virtual void SetupExperiment();

    /**
     * @brief Execute post step activities
     *
     */
    virtual void PostStep();

    /**
     * @brief Execute post experiment activities
     *
     */
    virtual void PostExperiment();

    /**
     * @brief Write data to disk
     *
     */
    virtual void SaveData();

    /**
     * @brief Check if experiment is over
     *
     * @return true
     * @return false
     */
    inline bool IsExperimentFinished() { return finished_; }

    /**
     * @brief Get the floor color
     *
     * @param c_position_on_plane Coordinates of the floor
     * @return CColor Color at the specified coordinates
     */
    virtual CColor GetFloorColor(const CVector2 &c_position_on_plane);

    /**
     * @brief Convert the data into a string so that it can be stored into JSON later
     *
     * @param data Vector of Real values
     * @return Data string
     */
    std::string ConvertDataToString(const std::vector<Real> &data);

protected:
    /**
     * @brief Initialize a new JSON object
     *
     */
    void InitializeJSON();

    /**
     * @brief Reset the robot positions manually because ARGoS doesn't randomize the positions after reset
     *
     */
    void ResetRobotPositions();

    UInt32 curr_trial_ind_ = 0; ///< Counter to keep track of trials

    ExperimentParams exp_params_; ///< Struct of experiment parameters

    Real arena_tile_size_ = -1.0; ///< Size of arena (square) tiles

    Real ticks_per_sec_ = -1.0; ///< Number of ticks in one second

    std::pair<UInt64, UInt64> arena_tile_count_ = {0, 0}; ///< Number of arena tiles

    std::pair<Real, Real> arena_lower_lim_ = {-1.0, -1.0}; ///< Bottom left corner coordinate of the arena

    Arena arena_; ///< Arena object

    CFloorEntity *floor_entity_ptr_ = NULL; ///< Pointer to the floor entity class

    CSpace *space_ptr_ = NULL; ///< Pointer to the space class

    std::string verbose_level_ = "Unset"; ///< Output verbosity level

    std::string output_filename_ = "Unset"; ///< Filename of output data

    ordered_json curr_json_; ///< Current JSON data object

    RobotIdDataStrMap id_data_str_map_; ///< Map storing the data for each robot

    std::vector<ordered_json> json_data_vec_; ///< Vector of JSON data objects

    std::vector<std::string> sorted_robot_ids_; ///< Vector of robot IDs, sorted

    std::shared_ptr<CSpace::TMapPerType> kheperaiv_entities_map_ptr_;

    bool finished_ = false; ///< Flag to indicate whether all simulation parameters have been executed
};

#endif