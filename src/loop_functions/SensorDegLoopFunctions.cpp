#include "loop_functions/SensorDegLoopFunctions.hpp"

void SensorDegLoopFunctions::Init(TConfigurationNode &t_tree)
{
    // Extract XML information
    try
    {
        // Call parent's Init
        CLoopFunctions::Init(t_tree);

        // Grab arena information
        TConfigurationNode &arena_tiles_node = GetNode(t_tree, "arena_tiles");

        // Grab verbosity level
        GetNodeAttribute(GetNode(t_tree, "verbosity"), "level", verbose_level_);

        // Get a pointer to the ARGoS floor entity (method provided by superclass)
        space_ptr_ = &GetSpace();
        floor_entity_ptr_ = &space_ptr_->GetFloorEntity();

        // Get the size of the arena (in units of tiles)
        UInt64 arena_x, arena_y;

        GetNodeAttribute(arena_tiles_node, "tile_count_x", arena_x);
        GetNodeAttribute(arena_tiles_node, "tile_count_y", arena_y);

        arena_tile_count_ = std::make_pair(arena_x, arena_y);

        // Get the limits of arena
        CRange<CVector3> lims = space_ptr_->GetArenaLimits();
        arena_lower_lim_ = std::make_pair(static_cast<Real>(lims.GetMin().GetX()),
                                          static_cast<Real>(lims.GetMin().GetY()));

        // Compute tile size
        CVector3 arena_size = space_ptr_->GetArenaSize();

        Real length_x = arena_size.GetX() / arena_x; // tile size in the x-direction
        Real length_y = arena_size.GetY() / arena_y; // tile size in the y-direction

        if (length_x != length_y)
        {
            LOGERR << "[WARNING] Tile sizes aren't square; implicitly making square tiles with an edge length that follows the provided x-axis dimension." << std::endl;
        }
        arena_tile_size_ = length_x;

        // Grab the constrained area to compute the true swarm density
        auto &box_map = space_ptr_->GetEntitiesByType("box");

        // Get constrained x length
        CBoxEntity &wall_west = *any_cast<CBoxEntity *>(box_map["wall_west"]);
        CBoxEntity &wall_east = *any_cast<CBoxEntity *>(box_map["wall_east"]);

        Real wall_west_thickness = wall_west.GetSize().GetX();
        Real wall_east_thickness = wall_east.GetSize().GetX();

        Real wall_west_pos_x = wall_west.GetEmbodiedEntity().GetOriginAnchor().Position.GetX();
        Real wall_east_pos_x = wall_east.GetEmbodiedEntity().GetOriginAnchor().Position.GetX();

        assert(abs(wall_west_pos_x) == abs(wall_east_pos_x));         // ensure that walls are evenly separated
        assert(abs(wall_west_thickness) == abs(wall_east_thickness)); // ensure that walls are equally thick

        // Get constrained y length
        CBoxEntity &wall_north = *any_cast<CBoxEntity *>(box_map["wall_north"]);
        CBoxEntity &wall_south = *any_cast<CBoxEntity *>(box_map["wall_south"]);

        Real wall_north_thickness = wall_north.GetSize().GetY();
        Real wall_south_thickness = wall_south.GetSize().GetY();

        Real wall_north_pos_y = wall_north.GetEmbodiedEntity().GetOriginAnchor().Position.GetY();
        Real wall_south_pos_y = wall_south.GetEmbodiedEntity().GetOriginAnchor().Position.GetY();

        assert(abs(wall_north_pos_y) == abs(wall_south_pos_y));         // ensure that walls are evenly separated
        assert(abs(wall_north_thickness) == abs(wall_south_thickness)); // ensure that walls are equally thick

        // Compute constrained arena area
        Real constrained_x_distance = (wall_east_pos_x - wall_west_pos_x) - wall_west_thickness;
        Real constrained_y_distance = (wall_north_pos_y - wall_south_pos_y) - wall_north_thickness;

        Real constrained_area = constrained_x_distance * constrained_y_distance;

        // Grab number of trials
        GetNodeAttribute(GetNode(t_tree, "num_trials"), "value", exp_params_.NumTrials);

        // Grab target fill ratio
        GetNodeAttribute(GetNode(t_tree, "target_fill_ratio"), "value", exp_params_.TargetFillRatio);

        // Grab random KheperaIV static degradation controller
        auto &kheperaiv_entities_map = space_ptr_->GetEntitiesByType("kheperaiv");
        CKheperaIVEntity &random_kheperaiv_entity = *any_cast<CKheperaIVEntity *>(kheperaiv_entities_map.begin()->second);
        KheperaIVDiffusionMotion &diffusion_controller = dynamic_cast<KheperaIVDiffusionMotion &>(random_kheperaiv_entity.GetControllableEntity().GetController());

        // Grab sensor accuracies
        exp_params_.ActualSensorAcc = diffusion_controller.GetGroundSensorParams().ActualSensorAcc;

        TConfigurationNode &flawed_robots_node = GetNode(t_tree, "flawed_robots");
        GetNodeAttribute(flawed_robots_node, "num", exp_params_.NumFlawedRobots);
        GetNodeAttribute(flawed_robots_node, "acc_b", exp_params_.AssumedSensorAcc.at("b"));
        GetNodeAttribute(flawed_robots_node, "acc_w", exp_params_.AssumedSensorAcc.at("w"));
        GetNodeAttribute(flawed_robots_node, "activate_filter_for_all", exp_params_.FilterActiveForAll);

        // Grab robot speeds
        exp_params_.RobotSpeed = diffusion_controller.GetWheelTurningParams().MaxSpeed;

        // Grab sensing and communications period
        exp_params_.MeasurementPeriod = diffusion_controller.GetGroundSensorParams().GroundMeasurementPeriodTicks;
        exp_params_.CommsPeriod = diffusion_controller.GetCommsParams().CommsPeriodTicks;

        // Grab filter parameters
        exp_params_.FilterMethod = diffusion_controller.GetSensorDegradationFilterParams().Method;
        exp_params_.FilterPeriod = diffusion_controller.GetSensorDegradationFilterParams().FilterActivationPeriodTicks;
        exp_params_.DynamicObservationQueue = diffusion_controller.GetSensorDegradationFilterParams().UseDynamicObservationQueue;
        exp_params_.DynamicObservationQueueWindowSize = diffusion_controller.GetSensorDegradationFilterParams().DynamicObservationQueueWindowSize;
        exp_params_.FilterSpecificParams = diffusion_controller.GetSensorDegradationFilterParams().FilterSpecificParams;

        // Grab ground sensor parameters
        exp_params_.DynamicDegradation = diffusion_controller.GetGroundSensorParams().IsDynamic;
        exp_params_.GroundSensorDriftCoeff = diffusion_controller.GetGroundSensorParams().DegradationCoefficients["drift"];
        exp_params_.GroundSensorDiffusionCoeff = diffusion_controller.GetGroundSensorParams().DegradationCoefficients["diffusion"];
        exp_params_.LowestDegradedAccuracyLevel = diffusion_controller.GetGroundSensorParams().LowestDegradedAccuracyLevel;

        // Grab observation queue size (0 if no queue is used)
        exp_params_.ObservationQueueSize = diffusion_controller.GetCollectivePerceptionParams().MaxObservationQueueSize;

        // Grab number of robots and communications range
        UInt64 size;
        Real range;

        CRABEquippedEntity &random_rab = random_kheperaiv_entity.GetRABEquippedEntity();
        size = kheperaiv_entities_map.size();
        range = random_rab.GetRange();

        exp_params_.NumRobots = size;   // the number of range and bearing sensors is the same as the number of robots
        exp_params_.CommsRange = range; // all the range and bearing sensors have the same range
        exp_params_.Density = static_cast<Real>(exp_params_.NumRobots) * M_PI * std::pow(exp_params_.CommsRange, 2) /
                              constrained_area; // the density is the ratio of swarm communication area to total walkable area

        // Grab number of steps
        exp_params_.NumSteps = GetSimulator().GetMaxSimulationClock();

        // Grab number of ticks in a second
        TConfigurationNode &framework_experiment_node = GetNode(GetNode(GetSimulator().GetConfigurationRoot(), "framework"), "experiment");
        GetNodeAttribute(framework_experiment_node, "ticks_per_second", ticks_per_sec_);

        // Grab position and orientation distribution if applicable
        TConfigurationNode &arena_node = GetNode(GetSimulator().GetConfigurationRoot(), "arena");

        try
        {
            // Check whether there are kheperaiv entities to distribute
            auto &distribute_node = GetNode(arena_node, "distribute");
            auto &entity_node = GetNode(distribute_node, "entity");

            // Set flag to re-distribute robot placement during reset
            exp_params_.DistributeRobotPlacement = true;

            // Get position
            exp_params_.PositionPlacementGeneratorPtr = std::shared_ptr<RealNumberGenerator>(CreateGenerator(GetNode(distribute_node, "position")));

            // Get orientation
            exp_params_.OrientationPlacementGeneratorPtr = std::shared_ptr<RealNumberGenerator>(CreateGenerator(GetNode(distribute_node, "orientation")));

            // Get number of placement trials
            GetNodeAttribute(GetNode(distribute_node, "entity"), "max_trials", exp_params_.MaxPlacementTrials);
        }
        catch (const std::exception &e)
        {
            // Do nothing, there's no robot pose to re-distribute when resetting later
        }

        // Grab save path
        GetNodeAttribute(GetNode(t_tree, "path"), "folder", exp_params_.SaveFolder);

        // Check if folder exists
        if (!std::filesystem::exists(exp_params_.SaveFolder))
        {
            THROW_ARGOSEXCEPTION("Save folder " + exp_params_.SaveFolder + " doesn't exist.");
        }

        if (verbose_level_ == "full")
        {
            LOG << "[INFO] Verbose level = \"" << verbose_level_ << "\"" << std::endl;
            LOG << "[INFO] Specifying number of arena tiles = " << arena_x << "*" << arena_y << std::endl;
            LOG << "[INFO] Specifying number of robots = " << exp_params_.NumRobots << std::endl;
            LOG << "[INFO] Specifying number of flawed robots = " << exp_params_.NumFlawedRobots << std::endl;
            LOG << "[INFO] Specifying number of trials = " << exp_params_.NumTrials << std::endl;
            LOG << "[INFO] Specifying number of simulation steps = " << exp_params_.NumSteps << std::endl;
            LOG << "[INFO] Specifying target fill ratio = " << exp_params_.TargetFillRatio << std::endl;
            LOG << "[INFO] Specifying flawed assumed accuracies (b,w) = " << exp_params_.AssumedSensorAcc["b"] << "," << exp_params_.AssumedSensorAcc["w"] << std::endl;
            LOG << "[INFO] Specifying actual accuracies (b,w) = " << exp_params_.ActualSensorAcc["b"] << "," << exp_params_.ActualSensorAcc["w"] << std::endl;
            LOG << "[INFO] Specifying communication range = " << exp_params_.CommsRange << " m" << std::endl;
            LOG << "[INFO] Specifying communication period = " << exp_params_.CommsPeriod << " ticks" << std::endl;
            LOG << "[INFO] Specifying measurement period = " << exp_params_.MeasurementPeriod << " ticks" << std::endl;
            LOG << "[INFO] Specifying robot speed = " << exp_params_.RobotSpeed << " cm/s" << std::endl;
            LOG << "[INFO] Computed swarm density = " << exp_params_.Density << std::endl;
            LOG << "[INFO] Specifying output folder = \"" << exp_params_.SaveFolder << "\"" << std::endl;
            LOG << "[INFO] Generated tile size = " << arena_tile_size_ << " m" << std::endl;
            LOG << "[INFO] Running trial 1" << std::endl;
        }
    }
    catch (CARGoSException &ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
    }

    // Iterate through all the robots and modify the flawed robots assumed accuracies
    kheperaiv_entities_map_ptr_ = std::make_shared<CSpace::TMapPerType>(space_ptr_->GetEntitiesByType("kheperaiv"));

    for (auto itr = kheperaiv_entities_map_ptr_->begin(); itr != kheperaiv_entities_map_ptr_->end(); ++itr)
    {
        sorted_robot_ids_.push_back(itr->first);
    }

    // Sort the robots based on their IDs so that the last n number of robots are flawed
    std::sort(sorted_robot_ids_.begin(), sorted_robot_ids_.end());

    // Go through the first n number of robots to activate degradation filter, if applicable (i.e., only for initially correct robots)
    UInt32 num_correct_robots = exp_params_.NumRobots - exp_params_.NumFlawedRobots;

    if (exp_params_.FilterActiveForAll)
    {
        for (size_t i = 0; i < num_correct_robots; ++i)
        {
            CKheperaIVEntity &kheperaiv_entity = *any_cast<CKheperaIVEntity *>(kheperaiv_entities_map_ptr_->at(sorted_robot_ids_[i]));
            KheperaIVDiffusionMotion &controller = dynamic_cast<KheperaIVDiffusionMotion &>(kheperaiv_entity.GetControllableEntity().GetController());

            controller.UpdateAssumedSensorAcc(exp_params_.ActualSensorAcc, true);
            controller.ActivateDegradationFilter();
            controller.SetLEDs(CColor::GREEN);
        }
    }

    // Go through the last n number of robots from the back to adjust their assumed sensor accuracies (i.e., only for initially flawed robots)
    for (size_t i = num_correct_robots; i < exp_params_.NumRobots; ++i)
    {
        CKheperaIVEntity &kheperaiv_entity = *any_cast<CKheperaIVEntity *>(kheperaiv_entities_map_ptr_->at(sorted_robot_ids_[i]));
        KheperaIVDiffusionMotion &controller = dynamic_cast<KheperaIVDiffusionMotion &>(kheperaiv_entity.GetControllableEntity().GetController());

        controller.UpdateAssumedSensorAcc(exp_params_.AssumedSensorAcc, true);
        controller.ActivateDegradationFilter();
        controller.SetLEDs(CColor::RED);
    }

    // Setup experiment
    SetupExperiment();
}

void SensorDegLoopFunctions::Reset()
{
    if (exp_params_.DistributeRobotPlacement)
    {
        ResetRobotPositions();
    }

    SetupExperiment();
}

void SensorDegLoopFunctions::SetupExperiment()
{
    // Create new Arena object
    arena_ = Arena(arena_tile_count_, arena_lower_lim_, arena_tile_size_, exp_params_.TargetFillRatio, GetSimulator().GetRandomSeed());

    id_data_str_map_ = RobotIdDataStrMap();

    InitializeJSON();

    // Collect data at the zero-th time step
    for (size_t i = 0; i < exp_params_.NumRobots; ++i)
    {
        // Get reference to the robot's controller
        CKheperaIVEntity &kheperaiv_entity = *any_cast<CKheperaIVEntity *>(kheperaiv_entities_map_ptr_->at(sorted_robot_ids_[i]));
        KheperaIVDiffusionMotion &controller = dynamic_cast<KheperaIVDiffusionMotion &>(kheperaiv_entity.GetControllableEntity().GetController());

        // Set the robots' RNG seed (based on top-level seed + ID)
        controller.SetRNGSeed(GetSimulator().GetRandomSeed() + i);

        // Convert and store data string
        id_data_str_map_[kheperaiv_entity.GetId()].push_back(ConvertDataToString(controller.GetData()));
    }
}

void SensorDegLoopFunctions::PostStep()
{
    // Grab data from each robot
    for (size_t i = 0; i < exp_params_.NumRobots; ++i)
    {
        // Get reference to the robot's controller
        CKheperaIVEntity &kheperaiv_entity = *any_cast<CKheperaIVEntity *>(kheperaiv_entities_map_ptr_->at(sorted_robot_ids_[i]));
        KheperaIVDiffusionMotion &controller = dynamic_cast<KheperaIVDiffusionMotion &>(kheperaiv_entity.GetControllableEntity().GetController());

        // Convert and store data string
        id_data_str_map_[kheperaiv_entity.GetId()].push_back(ConvertDataToString(controller.GetData()));
    }
}

void SensorDegLoopFunctions::PostExperiment()
{
    // Store new trial result
    std::vector<std::vector<std::string>> vec;
    vec.reserve(exp_params_.NumRobots);

    for (auto itr = sorted_robot_ids_.begin(); itr != sorted_robot_ids_.end(); ++itr)
    {
        vec.push_back(id_data_str_map_.at(*itr)); // store the values from the map, which are vectors of strings
    }

    curr_json_["data_str"] = vec;

    json_data_vec_.push_back(curr_json_);

    // Determine whether to terminate simulation
    if (++curr_trial_ind_ % exp_params_.NumTrials == 0) // all trials have completed
    {
        if (verbose_level_ == "full")
        {
            LOG << "[INFO] All trials completed." << std::endl;
        }

        SaveData();

        finished_ = true;
    }
    else // more trials required
    {
        // Repeat trial
        if (verbose_level_ == "full")
        {
            LOG << "[INFO] Running trial " << curr_trial_ind_ + 1 << std::endl;
        }
    }
}

void SensorDegLoopFunctions::SaveData()
{
    // Write JSON files into folder
    std::string filepath_prefix = exp_params_.SaveFolder + "/";

    // Create individual JSON files
    for (size_t i = 0; i < json_data_vec_.size(); ++i)
    {
        std::string filename = "flw" + std::to_string(exp_params_.NumFlawedRobots) + "_" +
                               "t" + std::to_string(i);

        filename = filepath_prefix + filename + ".json";

        // Export to single JSON file
        std::ofstream outfile(filename);

        outfile << std::setw(4) << (json_data_vec_[i]) << std::endl; // write pretty JSON

        outfile.close();
    }
}

void SensorDegLoopFunctions::InitializeJSON()
{
    curr_json_ = ordered_json{};
    curr_json_["sim_type"] = exp_params_.DynamicDegradation ? "dynamic_topo_dynamic_deg_1d" : "dynamic_topo_static_deg_1d";
    curr_json_["seed"] = GetSimulator().GetRandomSeed();
    curr_json_["num_robots"] = exp_params_.NumRobots;
    curr_json_["num_flawed_robots"] = exp_params_.NumFlawedRobots;
    curr_json_["num_trials"] = exp_params_.NumTrials;
    curr_json_["num_steps"] = exp_params_.NumSteps;
    curr_json_["method"] = exp_params_.FilterMethod;
    curr_json_["correct_robot_filter"] = exp_params_.FilterActiveForAll;
    curr_json_["sensor_filter_period"] = exp_params_.FilterPeriod;

    if (exp_params_.FilterSpecificParams.find("None") != exp_params_.FilterSpecificParams.end())
    {
        curr_json_["filter_specific_params"] = "None";
    }
    else
    {
        curr_json_["filter_specific_params"] = exp_params_.FilterSpecificParams;
    }

    curr_json_["tfr"] = exp_params_.TargetFillRatio;
    curr_json_["dynamic_degradation"] = exp_params_.DynamicDegradation;
    curr_json_["true_drift_coeff"] = exp_params_.GroundSensorDriftCoeff;
    curr_json_["true_diffusion_coeff"] = exp_params_.GroundSensorDiffusionCoeff;
    curr_json_["lowest_degraded_acc_lvl"] = exp_params_.LowestDegradedAccuracyLevel;
    curr_json_["obs_queue_size"] = exp_params_.ObservationQueueSize;
    curr_json_["dynamic_observation_queue"] = exp_params_.DynamicObservationQueue;
    curr_json_["dynamic_observation_queue_window_size"] = exp_params_.DynamicObservationQueueWindowSize;
    curr_json_["flawed_sensor_acc_b"] = exp_params_.AssumedSensorAcc["b"];
    curr_json_["flawed_sensor_acc_w"] = exp_params_.AssumedSensorAcc["w"];
    curr_json_["correct_sensor_acc_b"] = exp_params_.ActualSensorAcc["b"];
    curr_json_["correct_sensor_acc_w"] = exp_params_.ActualSensorAcc["w"];
    curr_json_["comms_range"] = exp_params_.CommsRange;
    curr_json_["comms_period"] = exp_params_.CommsPeriod;
    curr_json_["meas_period"] = exp_params_.MeasurementPeriod;
    curr_json_["speed"] = exp_params_.RobotSpeed;
    curr_json_["density"] = exp_params_.Density;
    curr_json_["trial_ind"] = curr_trial_ind_;
}

CColor SensorDegLoopFunctions::GetFloorColor(const CVector2 &c_position_on_plane)
{
    UInt32 color_int = arena_.GetColor(c_position_on_plane.GetX(), c_position_on_plane.GetY());

    return color_int == 1 ? CColor::BLACK : CColor::WHITE;
}

std::string SensorDegLoopFunctions::ConvertDataToString(const std::vector<Real> &data)
{
    /*
        The data is in the form:
        data[0] = random_seed
        data[1] = num_black_tiles
        data[2] = num_total_tiles
        data[3] = x_hat
        data[4] = alpha
        data[5] = x_bar
        data[6] = beta
        data[7] = x
        data[8] = assumed_sensor_acc_b
        data[9] = assumed_sensor_acc_w
        data[10] = true_sensor_acc_b
        data[11] = true_sensor_acc_w

        data[3] onwards contain floating point numbers
    */

    std::stringstream ss;
    ss.precision(0);
    ss << std::fixed;

    for (auto itr = data.begin(); itr != data.end(); ++itr)
    {
        // Increase precision for floating point numbers
        if (itr - data.begin() == 3)
        {
            ss.precision(6);
            ss << std::fixed;
        }

        ss << *itr << ",";
    }

    return ss.str().substr(0, ss.str().size() - 1); // return without the last comma
}

void SensorDegLoopFunctions::ResetRobotPositions()
{
    bool done_placing, retry_placing;
    UInt64 placement_trials;
    CVector3 position;
    CQuaternion orientation;
    CVector3 euler_angles;

    // Iterate through each robot
    for (size_t i = 0; i < exp_params_.NumRobots; ++i)
    {
        // Get reference to the robot's entity
        CKheperaIVEntity &kheperaiv_entity = *any_cast<CKheperaIVEntity *>(kheperaiv_entities_map_ptr_->at(sorted_robot_ids_[i]));

        done_placing = false;
        retry_placing = false;
        placement_trials = 0;

        do
        {
            ++placement_trials;

            // Set the positions and orientation
            position = (*exp_params_.PositionPlacementGeneratorPtr)(retry_placing);
            euler_angles = (*exp_params_.OrientationPlacementGeneratorPtr)(retry_placing);
            orientation.FromEulerAngles(
                ToRadians(CDegrees(euler_angles[0])),  // Z
                ToRadians(CDegrees(euler_angles[1])),  // Y
                ToRadians(CDegrees(euler_angles[2]))); // X

            // Move the robot
            done_placing = MoveEntity(kheperaiv_entity.GetEmbodiedEntity(), position, orientation);

            retry_placing = true; // if we get to this point after the first run without done_placing being true, then we're retrying
        } while (!done_placing && placement_trials <= exp_params_.MaxPlacementTrials);

        // Check if placing is done for this particular robot
        if (!done_placing)
        {
            THROW_ARGOSEXCEPTION("Can't place " << kheperaiv_entity.GetId());
        }
    }
}

REGISTER_LOOP_FUNCTIONS(SensorDegLoopFunctions, "sensor_deg_loop_functions")