// POSITIONING MODULE

// compile notes:
// in the cmakelists you can add this (obtained from the Vicon repo) or just require users to also install the vicon sdk libraries themselves (in that case, make this an optional linking thing in my cmakelists)
// include_directories(${CMAKE_SOURCE_DIR}/vicon_sdk/Linux64)
//       link_directories(${CMAKE_SOURCE_DIR}/vicon_sdk/Linux64)
//       set(VICON_PATH "vicon_sdk/Linux64")
//
// what we really need is just extract the .so file and link against libViconDataStreamSDK_CPP.so and #include "Client.h"

// Needs code from https://github.com/NESTLab/Vicon/blob/38ca8d7b52a7a727e8d37c2fb49c1b2058a8ead7/argos3/plugins/simulator/physics_engines/tracking/tracking_updaters/vicon_updater.cpp
// to connect to Vicon Datastream server

// Need the Datastream client header file from the Vicon SDK at https://github.com/NESTLab/Vicon/tree/master/vicon_sdk

// In a gist, we need to connect to the Vicon Datastream server that should be up automatically whenever we start the Vicon tracking program on the windows machine

// at each step,

// DATA LOGGING MODULE

// reuse most of the code from SensorDegLoopFunctions.cpp

// MISC
// start moving signal
#include <thread>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h> // inet_ntoa()

#include "loop_functions/RealKheperaIVExperimentLoopFunctions.hpp"

RealKheperaIVExperimentLoopFunctions::~RealKheperaIVExperimentLoopFunctions()
{
    // Set shutdown flag to stop the listener thread
    shutdown_flag_.store(true, std::memory_order_release);

    // Close sockets
    ::close(server_socket_);

    for (auto itr = robot_ip_socket_map_.begin(); itr != robot_ip_socket_map_.end(); ++itr)
    {
        LOG << "[INFO] Closing connection with robot " << itr->second << " at " << itr->first << std::endl;

        ::close(itr->second);
    }

    // Disconnect the Vicon client
    // Code obtained from
    // https://github.com/NESTLab/Vicon/blob/38ca8d7b52a7a727e8d37c2fb49c1b2058a8ead7/argos3/plugins/simulator/physics_engines/tracking/tracking_updaters/vicon_updater.cpp#L113
    LOG << "[INFO] Disconnecting from Vicon server." << std::endl;
    vicon_client_.Disconnect();

    timespec sleep_ts{};
    sleep_ts.tv_sec = 0;
    sleep_ts.tv_nsec = 100000000;

    while (vicon_client_.IsConnected().Connected)
    {
        LOGERR << "Vicon client is still connected, reattempting to disconnect." << std::endl;
        LOGERR.Flush();
        vicon_client_.Disconnect();
        nanosleep(&sleep_ts, nullptr);
    }
}

void RealKheperaIVExperimentLoopFunctions::Init(TConfigurationNode &t_tree)
{

    // Iterate over child elements with name "kheperaiv" to record all intended robots
    TConfigurationNodeIterator kheperaiv_ids_itr;

    std::string name, addr;
    for (kheperaiv_ids_itr = kheperaiv_ids_itr.begin(&GetNode(t_tree, "kheperaiv_ids"));
         kheperaiv_ids_itr != kheperaiv_ids_itr.end();
         ++kheperaiv_ids_itr)
    {
        GetNodeAttribute(*kheperaiv_ids_itr, "name", name);
        GetNodeAttribute(*kheperaiv_ids_itr, "address", addr);

        // Populate the IP to name map
        robot_ip_name_map_[addr] = name;

        // Initialize the other maps
        robot_name_info_map_[name];
        robot_ip_socket_map_[addr];
        robot_ip_data_map_[addr];

        sorted_robot_ids_.push_back(name);
    }

    expected_num_robot_connections_ = robot_ip_name_map_.size();

    // Sort the robot IDs
    std::sort(sorted_robot_ids_.begin(), sorted_robot_ids_.end());

    // Connect to the Vicon DataStream server
    std::string vicon_addr_str;
    SInt32 vicon_port_number;
    GetNodeAttributeOrDefault(GetNode(t_tree, "vicon_server"), "address", vicon_addr_str, std::string("192.168.1.211"));
    GetNodeAttributeOrDefault(GetNode(t_tree, "vicon_server"), "port", vicon_port_number, 801);

    ConnectToViconServer(vicon_addr_str, vicon_port_number);

    // Create a server for robots to connect
    SInt32 server_port_number;
    GetNodeAttribute(GetNode(t_tree, "argos_server"), "port", server_port_number);
    GetNodeAttribute(GetNode(t_tree, "argos_server"), "robot_to_server_msg_size", robot_to_server_msg_size_);
    GetNodeAttribute(GetNode(t_tree, "argos_server"), "server_to_robot_msg_size", server_to_robot_msg_size_);

    StartARGoSServer(server_port_number); // starts the ARGoS server to accept connections from the robots; will block until all robots are connected

    // Setup experiment parameters for data logging
    InitializeExperimentalParameters(t_tree);
}

void RealKheperaIVExperimentLoopFunctions::ConnectToViconServer(std::string vicon_addr_str, SInt32 port_number)
{
    /*
        The code here is adapted from
        https://github.com/NESTLab/Vicon/blob/38ca8d7b52a7a727e8d37c2fb49c1b2058a8ead7/argos3/plugins/simulator/physics_engines/tracking/tracking_updaters/vicon_updater.cpp
    */

    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = 100000000; // .1 sec sleep

    nanosleep(&ts, nullptr);

    std::string full_hostname = vicon_addr_str + ":" + std::to_string(port_number);

    LOG << "[INFO] Connecting to Vicon DataStream server at " << full_hostname << std::endl;
    LOG.Flush();

    UInt8 retries = 10;
    while (retries > 0)
    {
        vicon_client_.Connect(full_hostname);
        if (vicon_client_.IsConnected().Connected)
        {
            break;
        }
        nanosleep(&ts, nullptr);
        retries--;
        LOG << "[INFO] Failed to connect. " << retries << "retries remaining" << std::endl;
        LOG.Flush();
    }
    if (!vicon_client_.IsConnected().Connected)
        THROW_ARGOSEXCEPTION("Could not reconnect to Vicon Datastream "
                             << "Server at " << vicon_addr_str);

    LOG << "[INFO] Successfully connected." << std::endl;
    LOG.Flush();

    vicon_client_.SetStreamMode(ViconSDK::StreamMode::ClientPull);
    vicon_client_.SetAxisMapping(ViconSDK::Direction::Forward,
                                 ViconSDK::Direction::Left,
                                 ViconSDK::Direction::Up);
    vicon_client_.EnableSegmentData(); // necessary?
    vicon_client_.EnableMarkerData();  // necessary?

    nanosleep(&ts, nullptr); // necessary?

    // Spawn a thread to get poses from the robots
    std::thread vicon_rx_thread(&RealKheperaIVExperimentLoopFunctions::ViconThreadRx, this);
    vicon_rx_thread.detach();
}

void RealKheperaIVExperimentLoopFunctions::ViconThreadRx()
{
    while (!shutdown_flag_.load(std::memory_order_acquire))
    {
        // Get robot poses from the Vicon server
        CollectTrackedRobotPoses();

        // Run thread every 10 ms
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void RealKheperaIVExperimentLoopFunctions::StartARGoSServer(SInt32 port_number)
{
    // Create server socket
    server_socket_ = ::socket(AF_INET, SOCK_STREAM, 0); // automatic protocol is chosen with 0

    if (server_socket_ < 0) // invalid file descriptor
    {
        THROW_ARGOSEXCEPTION("Error creating socket: " << ::strerror(errno));
    }

    // Bind server socket to an IP and port
    ::sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;    // Accept connections on any network interface
    server_addr.sin_port = ::htons(port_number); // Convert port to network byte order (implicitly converting to SInt32)

    if (::bind(server_socket_, (struct ::sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        THROW_ARGOSEXCEPTION("Error binding socket server at port number " << port_number << ": " << ::strerror(errno));
    }

    const SInt32 enable = 1;

    if (setsockopt(server_socket_, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable)) < 0)
    {
        perror("setsockopt(SO_REUSEADDR) failed");
    }

    if (setsockopt(server_socket_, SOL_SOCKET, SO_REUSEPORT, &enable, sizeof(enable)) < 0)
    {
        perror("setsockopt(SO_REUSEPORT) failed");
    }

    // Start listening for connections (max 5 queued connections)
    if (::listen(server_socket_, 5) < 0)
    {
        THROW_ARGOSEXCEPTION("Error marking server socket as passive (to listen to incoming connection requests): " << ::strerror(errno));
    }

    LOG << "[INFO] Server started on port " << port_number << std::endl;
    LOG.Flush();

    // Accept connections from robots
    AcceptConnections();
}

void RealKheperaIVExperimentLoopFunctions::AcceptConnections()
{
    LOG << "[INFO] Waiting for connections..." << std::endl;
    LOG.Flush();

    while (num_connected_robots_ < expected_num_robot_connections_)
    {
        ::sockaddr_in client_addr{};
        ::socklen_t addrlen = sizeof(client_addr);

        // Block on accept, waiting for robots to connect
        SInt32 client_socket = ::accept(server_socket_, (struct ::sockaddr *)&client_addr, &addrlen);

        if (client_socket < 0)
        {
            THROW_ARGOSEXCEPTION("Error accepting connection:" << ::strerror(errno));
        }

        // Convert the IP address to a human-readable string
        char *client_ip = ::inet_ntoa(client_addr.sin_addr);

        // Increment the connected robot counter and store the socket
        ++num_connected_robots_;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            robot_ip_socket_map_[client_ip] = client_socket;
        }

        LOG << "[INFO] Robot connected from IP: " << client_ip << ". Total connected: " << num_connected_robots_ << std::endl;
        LOG.Flush();

        // Spawn a new receiving and sending threads
        std::thread rx_thread(&RealKheperaIVExperimentLoopFunctions::RobotThreadRx, this, client_socket, client_ip);
        std::thread tx_thread(&RealKheperaIVExperimentLoopFunctions::RobotThreadTx, this, client_socket, client_ip);
        rx_thread.detach();
        tx_thread.detach();
    }

    // Once the required number of robots are connected, stop accepting new connections
    LOG << "[INFO] All expected robots connected. Stopping connection acceptance." << std::endl;
}

void RealKheperaIVExperimentLoopFunctions::RobotThreadTx(SInt32 client_socket, const std::string &client_ip)
{
    // Define transmission related variables
    RobotServerMessage data_to_send;
    CByteArray byte_arr;
    UInt8 *send_buffer = new UInt8[server_to_robot_msg_size_];
    UInt8 *buffer_ptr = send_buffer; // create a pointer alias for traversing through the buffer
    ssize_t remaining_size, bytes_sent;

    // Define data related variables
    UInt8 start_bit = 0;
    CRadians z_rot, y_rot, x_rot;
    Real x, y;
    std::string robot_name;

    while (!shutdown_flag_.load(std::memory_order_acquire))
    {
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);

            robot_name = robot_ip_name_map_[client_ip];

            // Extract the Euler angles of the robot's orientation (originally in quaternion)
            robot_name_info_map_[robot_name].RobotOrientation.ToEulerAngles(z_rot, y_rot, x_rot);

            // Extract linear positions
            x = robot_name_info_map_[robot_name].RobotPosition.GetX();
            y = robot_name_info_map_[robot_name].RobotPosition.GetY();
            start_bit = robot_start_flag_;
        }

        // Serialize data
        byte_arr << start_bit;        // start flag
        byte_arr << robot_name;       // robot name
        byte_arr << x;                // robot x
        byte_arr << y;                // robot y
        byte_arr << z_rot.GetValue(); // robot theta

        data_to_send.PopulateMessage(byte_arr);
        data_to_send.Serialize(buffer_ptr);

        // Ensure that the buffer is completely sent
        remaining_size = server_to_robot_msg_size_;

        while (remaining_size > 0)
        {
            bytes_sent = ::send(client_socket, buffer_ptr, remaining_size, 0);

            if (bytes_sent < 0)
            {
                THROW_ARGOSEXCEPTION("Error sending information to " << client_ip << ": " << ::strerror(errno));
            }

            remaining_size -= bytes_sent;
            buffer_ptr += bytes_sent;
        }

        // Reset pointer position to the start of the buffer
        buffer_ptr = send_buffer;

        // Clean up
        data_to_send.CleanUp();
        byte_arr.Clear();

        // Run thread in 20 Hz
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    delete[] send_buffer;
}

void RealKheperaIVExperimentLoopFunctions::RobotThreadRx(SInt32 client_socket, const std::string &client_ip)
{
    // Define transmission related variables
    RobotServerMessage data_received;
    CByteArray byte_arr;
    UInt8 *receive_buffer = new UInt8[robot_to_server_msg_size_];
    UInt8 *buffer_ptr = receive_buffer; // create a pointer alias for traversing through the buffer
    ssize_t remaining_size, bytes_received;

    // Define data related variables
    std::vector<Real> robot_data_vec;
    UInt16 num_data_elements;

    /*
        Brief explanation of `bytes_received = ::recv(client_socket, buffer_ptr, remaining_size, 0);`:

        client_socket: The socket from which data is received.
        buffer_ptr: A pointer to the memory where received data will be stored.
        remaining_size: The maximum number of (remaining) bytes we expect to receive.
        0: Flags (we use 0 for default behavior).
        bytes_received: The actual number of bytes received from the socket.

    */

    while (!shutdown_flag_.load(std::memory_order_acquire))
    {
        remaining_size = robot_to_server_msg_size_;

        buffer_ptr = receive_buffer; // reset buffer_ptr to the start of the buffer

        // Keep receiving until the complete data is received
        while (remaining_size > 0)
        {
            bytes_received = ::recv(client_socket, buffer_ptr, remaining_size, 0);

            if (bytes_received < 0)
            {
                THROW_ARGOSEXCEPTION("Error receiving data from " << client_ip << ": " << strerror(errno));
            }

            remaining_size -= bytes_received;
            buffer_ptr += bytes_received;
        }

        // Reset pointer position to the start of the buffer
        buffer_ptr = receive_buffer;

        // Deserialize data
        data_received.Deserialize(buffer_ptr);

        byte_arr = data_received.GetPayload();

        byte_arr >> num_data_elements; // get the number of elements in the data vector

        robot_data_vec.resize(num_data_elements);

        for (auto itr = robot_data_vec.begin(); itr != robot_data_vec.end(); ++itr)
        {
            byte_arr >> (*itr);
        }

        // Store the data
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            robot_ip_data_map_[client_ip].push_back(ConvertDataToString(robot_data_vec));
        }

        // Clean up
        data_received.CleanUp();
        robot_data_vec.clear();

        // Run thread in 20 Hz
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    delete[] receive_buffer;
}

void RealKheperaIVExperimentLoopFunctions::CollectTrackedRobotPoses()
{
    /*
        The code here is adapted from
        https://github.com/NESTLab/Vicon/blob/38ca8d7b52a7a727e8d37c2fb49c1b2058a8ead7/argos3/plugins/simulator/physics_engines/tracking/tracking_updaters/vicon_updater.cpp
    */
    std::string subject_name, segment_name, robot_name, robot_type, robot_options;
    UInt16 subject_count, segment_count;

    vicon_client_.GetFrame();

    ViconSDK::Output_GetSubjectCount vicon_subject_count_output = vicon_client_.GetSubjectCount();
    subject_count = vicon_subject_count_output.SubjectCount;

    // LOG << "There are " << subject_count << " subjects." << std::endl;
    for (UInt16 subject_ind = 0; subject_ind < subject_count; subject_ind++)
    {
        subject_name = vicon_client_.GetSubjectName(subject_ind).SubjectName;
        segment_count = vicon_client_.GetSegmentCount(subject_name).SegmentCount;

        if (segment_count > 0)
        {
            segment_name = vicon_client_.GetSegmentName(subject_name, 0).SegmentName;

            // Get position in mm
            ViconSDK::Output_GetSegmentGlobalTranslation translation =
                vicon_client_.GetSegmentGlobalTranslation(subject_name,
                                                          segment_name);

            // Get orientation
            ViconSDK::Output_GetSegmentGlobalRotationQuaternion orientation =
                vicon_client_.GetSegmentGlobalRotationQuaternion(subject_name,
                                                                 segment_name);

            CVector3 origin;
            CQuaternion rotation;

            if (translation.Result == ViconSDK::Result::Success &&
                orientation.Result == ViconSDK::Result::Success &&
                !translation.Occluded && !orientation.Occluded)
            {

                origin.Set(
                    translation.Translation[0] / 1000.0,
                    translation.Translation[1] / 1000.0,
                    translation.Translation[2] / 1000.0);

                rotation.SetW(orientation.Rotation[3]);
                rotation.SetX(orientation.Rotation[0]);
                rotation.SetY(orientation.Rotation[1]);
                rotation.SetZ(orientation.Rotation[2]);

                GetRobotTypeFromName(segment_name, robot_type, robot_name, robot_options);

                // Store the robot info (pose and orientation)
                {
                    std::lock_guard<std::mutex> lock(pose_mutex_);

                    // Ensure that the robot names are not erroneous (as a sanity check); the unordered map is initialized in Init()
                    if (robot_name_info_map_.find(robot_name) != robot_name_info_map_.end())
                    {
                        robot_name_info_map_[robot_name] = RobotInfo(robot_type, robot_name, origin, rotation, robot_options);
                    }
                    else
                    {
                        THROW_ARGOSEXCEPTION("Robot name: " << robot_name << " from the Vicon server not found in the ARGoS server. Did you add it to the .argos file?");
                    }
                }
            }
        }
    }
}

void RealKheperaIVExperimentLoopFunctions::GetRobotTypeFromName(const std::string &original_name_str,
                                                                std::string &type_str,
                                                                std::string &name_str,
                                                                std::string &options_str)
{
    /*
        The code here is adapted from
        https://github.com/NESTLab/Vicon/blob/38ca8d7b52a7a727e8d37c2fb49c1b2058a8ead7/argos3/plugins/simulator/physics_engines/tracking/tracking_updaters/vicon_updater.cpp
    */
    if (original_name_str.empty())
    {
        return;
    }
    std::stringstream ss_original_name_options, ss_name;

    ss_original_name_options.str(original_name_str);
    std::getline(ss_original_name_options, name_str, '\'');
    std::getline(ss_original_name_options, options_str);

    if (name_str.empty())
    {
        return;
    }
    ss_name.str(name_str);
    std::getline(ss_name, type_str, '_');
}

void RealKheperaIVExperimentLoopFunctions::InitializeExperimentalParameters(TConfigurationNode &t_tree)
{
    // Extract XML information
    try
    {
        // Call parent's Init
        CLoopFunctions::Init(t_tree);

        exp_params_.NumTrials = 1; // only one trial can be run at a time with real robot experiments

        // Grab target fill ratio
        GetNodeAttribute(GetNode(t_tree, "target_fill_ratio"), "value", exp_params_.TargetFillRatio);

        TConfigurationNode &flawed_robots_node = GetNode(t_tree, "flawed_robots");
        GetNodeAttribute(flawed_robots_node, "num", exp_params_.NumFlawedRobots);
        GetNodeAttribute(flawed_robots_node, "activate_filter_for_all", exp_params_.FilterActiveForAll);

        // Grab controller parameters
        TConfigurationNode &controller_params_node = GetNode(GetNode(t_tree, "controller_params"), "params");

        // Grab robot speeds
        GetNodeAttribute(GetNode(controller_params_node, "wheel_turning"), "max_speed", exp_params_.RobotSpeed);

        // Grab sensing and communications period
        GetNodeAttribute(GetNode(controller_params_node, "comms"), "period_ticks", exp_params_.CommsPeriod);
        GetNodeAttribute(GetNode(controller_params_node, "comms"), "single_hop_radius", exp_params_.CommsRange);

        // Grab filter parameters
        GetNodeAttribute(GetNode(controller_params_node, "sensor_degradation_filter"), "method", exp_params_.FilterMethod);
        GetNodeAttribute(GetNode(controller_params_node, "sensor_degradation_filter"), "period_ticks", exp_params_.FilterPeriod);
        GetNodeAttribute(GetNode(controller_params_node, "sensor_degradation_filter"), "dynamic_observation_queue", exp_params_.DynamicObservationQueue);
        GetNodeAttribute(GetNode(controller_params_node, "sensor_degradation_filter"), "dynamic_observation_queue_window_size", exp_params_.DynamicObservationQueueWindowSize);

        // Grab filter specific parameters
        if (exp_params_.FilterMethod == "DELTA")
        {
            std::string pred_deg_model_B_str, pred_deg_var_R_str, init_mean_str, init_var_str, variant_str, lowest_assumed_acc_lvl_str;

            // Check to see if the lowest assumed accuracy level possible is provided
            GetNodeAttribute(GetNode(GetNode(controller_params_node, "sensor_degradation_filter"), "params"), "pred_deg_model_B", pred_deg_model_B_str);
            GetNodeAttribute(GetNode(GetNode(controller_params_node, "sensor_degradation_filter"), "params"), "pred_deg_var_R", pred_deg_var_R_str);
            GetNodeAttribute(GetNode(GetNode(controller_params_node, "sensor_degradation_filter"), "params"), "init_mean", init_mean_str);
            GetNodeAttribute(GetNode(GetNode(controller_params_node, "sensor_degradation_filter"), "params"), "init_var", init_var_str);
            GetNodeAttribute(GetNode(GetNode(controller_params_node, "sensor_degradation_filter"), "params"), "lowest_assumed_acc_lvl", lowest_assumed_acc_lvl_str);
            GetNodeAttribute(GetNode(GetNode(controller_params_node, "sensor_degradation_filter"), "params"), "variant", variant_str);

            exp_params_.FilterSpecificParams = {{"pred_deg_model_B", pred_deg_model_B_str},
                                                {"pred_deg_var_R", pred_deg_var_R_str},
                                                {"init_mean", init_mean_str},
                                                {"init_var", init_var_str},
                                                {"variant", variant_str},
                                                {"lowest_assumed_acc_lvl", lowest_assumed_acc_lvl_str}};
        }
        else
        {
            THROW_ARGOSEXCEPTION("Other filter methods are not implemented in the real robot loop functions yet.");
        }

        // Grab ground sensor parameters
        GetNodeAttribute(GetNode(controller_params_node, "ground_sensor"), "period_ticks", exp_params_.MeasurementPeriod);
        GetNodeAttribute(GetNode(controller_params_node, "ground_sensor"), "sensor_acc_b", exp_params_.ActualSensorAcc.at("b"));
        GetNodeAttribute(GetNode(controller_params_node, "ground_sensor"), "sensor_acc_w", exp_params_.ActualSensorAcc.at("w"));
        GetNodeAttribute(GetNode(controller_params_node, "ground_sensor"), "assumed_sensor_acc_b", exp_params_.AssumedSensorAcc.at("b"));
        GetNodeAttribute(GetNode(controller_params_node, "ground_sensor"), "assumed_sensor_acc_w", exp_params_.AssumedSensorAcc.at("w"));
        GetNodeAttribute(GetNode(controller_params_node, "ground_sensor"), "dynamic", exp_params_.DynamicDegradation);
        GetNodeAttribute(GetNode(controller_params_node, "ground_sensor"), "true_deg_drift_coeff", exp_params_.GroundSensorDriftCoeff);
        GetNodeAttribute(GetNode(controller_params_node, "ground_sensor"), "true_deg_diffusion_coeff", exp_params_.GroundSensorDiffusionCoeff);
        GetNodeAttribute(GetNode(controller_params_node, "ground_sensor"), "lowest_degraded_acc_lvl", exp_params_.LowestDegradedAccuracyLevel);

        // Grab observation queue size (0 if no queue is used)
        GetNodeAttribute(GetNode(controller_params_node, "sensor_degradation_filter"), "observation_queue_size", exp_params_.ObservationQueueSize);
        exp_params_.NumRobots = robot_ip_socket_map_.size(); // the number of range and bearing sensors is the same as the number of robots
        exp_params_.Density = -1.0;                          // the density can only be calculated with real world measurements

        // Grab number of steps
        exp_params_.NumSteps = GetSimulator().GetMaxSimulationClock();

        // Grab number of ticks in a second
        TConfigurationNode &framework_experiment_node = GetNode(GetNode(GetSimulator().GetConfigurationRoot(), "framework"), "experiment");
        GetNodeAttribute(framework_experiment_node, "ticks_per_second", ticks_per_sec_);

        // Grab position and orientation distribution if applicable
        TConfigurationNode &arena_node = GetNode(GetSimulator().GetConfigurationRoot(), "arena");

        // Grab save path
        GetNodeAttribute(GetNode(t_tree, "path"), "folder", exp_params_.SaveFolder);

        // Check if folder exists
        if (!std::filesystem::exists(exp_params_.SaveFolder))
        {
            THROW_ARGOSEXCEPTION("Save folder " + exp_params_.SaveFolder + " doesn't exist.");
        }

        LOG << "[INFO] Specifying number of robots = " << exp_params_.NumRobots << std::endl;
        LOG << "[INFO] Specifying number of flawed robots = " << exp_params_.NumFlawedRobots << std::endl;
        LOG << "[INFO] Specifying number of time steps = " << exp_params_.NumSteps << std::endl;
        LOG << "[INFO] Specifying flawed assumed accuracies (b,w) = " << exp_params_.AssumedSensorAcc["b"] << "," << exp_params_.AssumedSensorAcc["w"] << std::endl;
        LOG << "[INFO] Specifying actual accuracies (b,w) = " << exp_params_.ActualSensorAcc["b"] << "," << exp_params_.ActualSensorAcc["w"] << std::endl;
        LOG << "[INFO] Specifying communication range = " << exp_params_.CommsRange << " m" << std::endl;
        LOG << "[INFO] Specifying communication period = " << exp_params_.CommsPeriod << " ticks" << std::endl;
        LOG << "[INFO] Specifying measurement period = " << exp_params_.MeasurementPeriod << " ticks" << std::endl;
        LOG << "[INFO] Specifying robot speed = " << exp_params_.RobotSpeed << " cm/s" << std::endl;
        LOG << "[INFO] Computed swarm density = " << exp_params_.Density << std::endl;
        LOG << "[INFO] Specifying output folder = \"" << exp_params_.SaveFolder << "\"" << std::endl;
        LOG << "[INFO] Running trial 1" << std::endl;
    }
    catch (CARGoSException &ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
    }

    // Initialize the JSON data file
    InitializeJSON();

    // // Replace some JSON values
    curr_json_["sim_type"] = "real_robot_dynamic_deg_1d";
    curr_json_["seed"] = 8204;
}

void RealKheperaIVExperimentLoopFunctions::PreStep()
{
    // Raise flag to indicate starting of experiment to permit robot operation
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (robot_start_flag_ != 1)
        {
            LOG << "[INFO] Experiment started, robots will now begin operation." << std::endl;
            robot_start_flag_ = UInt8(1);
        }
    }
}

void RealKheperaIVExperimentLoopFunctions::PostExperiment()
{
    // Send stop signal to the robots
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);

        robot_start_flag_ = UInt8(0);
    }

    LOG << "[INFO] Experiment completed." << std::endl;
    LOG << "[INFO] Waiting for robots to complete sending data. Do not terminate ARGoS." << std::endl;
    LOG.Flush();

    // Store the data in the JSON object
    std::vector<std::vector<std::string>> vec;

    vec.reserve(exp_params_.NumRobots);

    // Check to see if all the data has been received (should be all of the tick count plus the initial data at time 0)
    UInt32 final_tick_count = GetSimulator().GetSpace().GetSimulationClock();
    UInt32 remainder = 0;

    bool complete = false;

    while (!complete)
    {
        // Test using an AND logic (assume it's complete first and see if that's true)
        complete = true;

        // Iterate through each data vector
        {
            std::lock_guard<std::mutex> lock(data_mutex_);

            for (auto itr = robot_ip_data_map_.begin(); itr != robot_ip_data_map_.end(); ++itr)
            {
                if (itr->second.size() <= final_tick_count)
                {
                    complete = false;
                    break;
                }
            }
        }

        // Sleep for a while to let the messages come in since it's incomplete
        if (!complete)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // wait for 2 seconds
        }
        else // clean up and store the data (sometimes more than needed would be transmitted)
        {
            std::lock_guard<std::mutex> lock(data_mutex_);

            for (auto itr = sorted_robot_ids_.begin(); itr != sorted_robot_ids_.end(); ++itr)
            {
                // Find the IP based on the name
                auto itr_ip_name = std::find_if(
                    robot_ip_name_map_.begin(), robot_ip_name_map_.end(), [&itr](const auto &pair)
                    { return pair.second == *itr; });

                // Store into the data vector of only the expected size (removing extra data)
                remainder = robot_ip_data_map_.at(itr_ip_name->first).size() - (final_tick_count + 1);

                std::vector<std::string> correct_sized_data{robot_ip_data_map_.at(itr_ip_name->first).begin(),
                                                            robot_ip_data_map_.at(itr_ip_name->first).end() - remainder};

                vec.push_back(correct_sized_data); // store the values from the map, which are vectors of strings
            }
        }
    }

    // Store the data into disk
    curr_json_["data_str"] = vec;

    json_data_vec_.push_back(curr_json_);

    SaveData();

    LOG << "[INFO] Data saved to disk." << std::endl;
    LOG << "[INFO] It is now safe to terminate ARGoS." << std::endl;
}

REGISTER_LOOP_FUNCTIONS(RealKheperaIVExperimentLoopFunctions, "real_kheperaiv_experiment_loop_functions")