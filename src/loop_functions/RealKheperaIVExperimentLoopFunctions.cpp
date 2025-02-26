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

void RealKheperaIVExperimentLoopFunctions::Init(TConfigurationNode &t_tree)
{
    /*
        The code here is adapted from
        https://github.com/NESTLab/Vicon/blob/38ca8d7b52a7a727e8d37c2fb49c1b2058a8ead7/argos3/plugins/simulator/physics_engines/tracking/tracking_updaters/vicon_updater.cpp
    */

    // Connect to the Vicon DataStream server
    std::string vicon_addr_str;
    SInt32 vicon_port_number;
    GetNodeAttributeOrDefault(GetNode(t_tree, "vicon_server"), "address", vicon_addr_str, std::string("192.168.1.211"));
    GetNodeAttributeOrDefault(GetNode(t_tree, "vicon_server"), "port", vicon_port_number, 801);

    ConnectToViconServer(vicon_addr_str, vicon_port_number);

    // Create a server for robots to connect
    SInt32 server_port_number;
    GetNodeAttribute(GetNode(t_tree, "argos_server"), "port", server_port_number);
    GetNodeAttribute(GetNode(t_tree, "argos_server"), "num_robots", expected_num_robot_connections_);
    GetNodeAttribute(GetNode(t_tree, "argos_server"), "robot_data_size_in_bytes", robot_data_size_);

    StartServer(server_port_number);

    /* Wait to connect */
    // TODO: is it connected at this point? How to check? Is `StartServer` blocking (which is what we want, so that we don't proceed until all intended robots are onboard)

    // Iterate over child elements with name "kheperaiv"
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
    }
}

void RealKheperaIVExperimentLoopFunctions::ConnectToViconServer(std::string vicon_addr_str, SInt32 port_number)
{
    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = 100000000; // .1 sec sleep

    nanosleep(&ts, nullptr);

    std::string full_hostname = vicon_addr_str + ":" + std::to_string(port_number);

    LOG << "Connecting to Vicon DataStream server at " << full_hostname << std::endl;
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
        LOG << "Failed to connect. " << retries << "retries remaining" << std::endl;
        LOG.Flush();
    }
    if (!vicon_client_.IsConnected().Connected)
        THROW_ARGOSEXCEPTION("Could not reconnect to Vicon Datastream "
                             << "Server at " << vicon_addr_str);

    LOG << "Successfully connected." << std::endl;

    vicon_client_.SetStreamMode(ViconSDK::StreamMode::ClientPull);
    vicon_client_.SetAxisMapping(ViconSDK::Direction::Forward,
                                 ViconSDK::Direction::Left,
                                 ViconSDK::Direction::Up);
    vicon_client_.EnableSegmentData(); // necessary?
    vicon_client_.EnableMarkerData();  // necessary?

    nanosleep(&ts, nullptr); // necessary?
}

void RealKheperaIVExperimentLoopFunctions::StartServer(SInt32 port_number)
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
    server_addr.sin_port = ::htons(port_number); // Convert port to network byte order (implicitly converting to SInt16)

    if (::bind(server_socket_, (struct ::sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        THROW_ARGOSEXCEPTION("Error binding socket server: " << ::strerror(errno));
    }

    // Start listening for connections (max 5 queued connections)
    if (::listen(server_socket_, 5) < 0)
    {
        THROW_ARGOSEXCEPTION("Error marking server socket as passive (to listen to incoming connection requests): " << ::strerror(errno));
    }

    LOG << "Server started on port " << port_number << ". Waiting for connections...\n";

    // Start a separate thread to accept connections
    std::thread accept_thread(&RealKheperaIVExperimentLoopFunctions::AcceptConnections, this);

    // Wait for the accept thread to finish
    accept_thread.join();
}

void RealKheperaIVExperimentLoopFunctions::AcceptConnections()
{
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
        robot_ip_socket_map_[client_ip] = client_socket;

        LOG << "Robot connected from IP: " << client_ip << ". Total connected: " << num_connected_robots_ + 1 << std::endl;

        // Once a robot connects, spawn a new thread to handle communication with it
        std::thread listener_thread(&RealKheperaIVExperimentLoopFunctions::ListenToRobot, this, client_socket, client_ip);
        listener_thread.detach();
    }

    // Once the required number of robots are connected, stop accepting new connections
    LOG << "Max robots connected. Stopping connection acceptance." << std::endl;
}

void RealKheperaIVExperimentLoopFunctions::ListenToRobot(SInt32 client_socket, const std::string &client_ip)
{
    UInt8 *buffer = new UInt8[robot_data_size_];
    ssize_t bytes_received;
    UInt32 num_elements_in_data;
    CByteArray received_data;

    std::vector<Real> robot_data_vec;

    /*
        Brief explanation of `bytes_received = ::recv(client_socket, buffer, robot_data_size_, 0);`:

        client_socket: The socket from which data is received.
        buffer: A pointer to the memory where received data will be stored.
        robot_data_size_: The maximum number of bytes we expect to receive.
        0: Flags (we use 0 for default behavior).
        bytes_received: The actual number of bytes received from the socket.

    */

    while (!shutdown_flag_.load(std::memory_order_acquire))
    {
        // Ensure that the buffer is completely empty (recv() should overwrite it anyway, but just in case)
        ::memset(buffer, 0, robot_data_size_);

        // Receive data from the robot
        bytes_received = ::recv(client_socket, buffer, robot_data_size_, 0);

        // Check if the connection is closed (0 bytes received)
        if (bytes_received == 0)
        {
            THROW_ARGOSEXCEPTION("Robot " << client_ip << " disconnected.");
        }

        // Check for errors during receiving data
        if (bytes_received < 0)
        {
            THROW_ARGOSEXCEPTION("Error receiving data from robot " << client_ip);
        }

        // Store the received data
        received_data = CByteArray(buffer, bytes_received);

        // Extract the number of elements
        received_data >> num_elements_in_data;

        // Resize vector to hold received floats
        robot_data_vec.resize(num_elements_in_data);

        for (size_t i = 0; i < num_elements_in_data; ++i)
        {
            received_data >> robot_data_vec[i];
        }

        // Store the data
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            robot_ip_data_map_[client_ip].push_back(ConvertDataToString(robot_data_vec));
        }

        // Clear the vector
        robot_data_vec.clear();
    }

    // After exiting the loop, close the socket
    LOG << "Closing connection with robot " << client_ip << std::endl;
    ::close(client_socket); // close the socket to release resources

    delete[] buffer;
}

void RealKheperaIVExperimentLoopFunctions::CollectTrackedRobotPoses()
{
    /*
        The code here is adapted from
        https://github.com/NESTLab/Vicon/blob/38ca8d7b52a7a727e8d37c2fb49c1b2058a8ead7/argos3/plugins/simulator/physics_engines/tracking/tracking_updaters/vicon_updater.cpp
    */
    std::string subject_name, segment_name, robot_name, robot_type, robot_options;
    UInt16 subject_count, segment_count;

    robot_name_info_map_.clear();

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
                if (robot_name_info_map_.find(robot_name) != robot_name_info_map_.end())
                {
                    std::lock_guard<std::mutex> lock(pose_mutex_);
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

void RealKheperaIVExperimentLoopFunctions::SendPoseInfoToRobots()
{
    std::lock_guard<std::mutex> lock(pose_mutex_);

    CByteArray data;
    CRadians z_rot, y_rot, x_rot;

    // Populate and send the data
    for (auto itr = robot_ip_name_map_.begin(); itr != robot_ip_name_map_.end(); ++itr)
    {
        // Extract the Euler angles of the robot's orientation (originally in quaternion)
        robot_name_info_map_[itr->second].RobotOrientation.ToEulerAngles(z_rot, y_rot, x_rot);

        data << robot_start_flag_;                                      // start flag
        data << itr->second;                                            // robot name
        data << robot_name_info_map_[itr->second].RobotPosition.GetX(); // robot x
        data << robot_name_info_map_[itr->second].RobotPosition.GetY(); // robot y
        data << z_rot.GetValue();                                       // robot theta

        if (::send(robot_ip_socket_map_[itr->first], data.ToCArray(), data.Size(), 0))
        {
            THROW_ARGOSEXCEPTION("Error sending information to " << itr->second << " at " << itr->first);
        }
    }
}

void RealKheperaIVExperimentLoopFunctions::PreStep()
{
    // Raise flag to indicate starting of experiment to permit robot operation
    if (robot_start_flag_ != 1)
    {
        LOG << "Experiment started, robot will now begin operation." << std::endl;
        robot_start_flag_ = UInt8(1);
    }

    // Get the robot poses
    CollectTrackedRobotPoses(); // TODO: PUT THIS INTO A SENDER THREAD

    // Send the poses and operation flag
    SendPoseInfoToRobots(); // TODO: PUT THIS IN A SENDER THREAD SO THAT WE CAN SEND MORE OFTEN AT 100HZ
}

void RealKheperaIVExperimentLoopFunctions::PostStep()
{
    // Process the collected data
}

void RealKheperaIVExperimentLoopFunctions::PostExperiment()
{
    // Save data
    // Send the stop flag
}

REGISTER_LOOP_FUNCTIONS(RealKheperaIVExperimentLoopFunctions, "real_kheperaiv_experiment_loop_functions")