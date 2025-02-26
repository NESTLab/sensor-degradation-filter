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
#include "loop_functions/RealKheperaIVExperimentLoopFunctions.hpp"

void RealKheperaIVExperimentLoopFunctions::Init(TConfigurationNode &t_tree)
{
    /*
        The code here is adapted from
        https://github.com/NESTLab/Vicon/blob/38ca8d7b52a7a727e8d37c2fb49c1b2058a8ead7/argos3/plugins/simulator/physics_engines/tracking/tracking_updaters/vicon_updater.cpp
    */

    // Connect to the Vicon DataStream server
    std::string host_ip_str;
    SInt32 port_number;
    GetNodeAttributeOrDefault(GetNode(t_tree, "vicon_server"), "host", host_ip_str, std::string("192.168.1.211"));
    GetNodeAttributeOrDefault(GetNode(t_tree, "vicon_server"), "port", port_number, 801);

    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = 100000000; // .1 sec sleep

    nanosleep(&ts, nullptr);

    std::string full_hostname = host_ip_str + ":" + std::to_string(port_number);

    LOG << "Connecting to Vicon DataStream server at " << full_hostname << std::endl;
    LOG.Flush();

    UInt8 retries = 10;
    while (retries > 0)
    {
        vicon_client.Connect(full_hostname);
        if (vicon_client.IsConnected().Connected)
        {
            break;
        }
        nanosleep(&ts, nullptr);
        retries--;
        LOG << "Failed to connect. " << retries << "retries remaining" << std::endl;
        LOG.Flush();
    }
    if (!vicon_client.IsConnected().Connected)
        THROW_ARGOSEXCEPTION("Could not reconnect to Vicon Datastream "
                             << "Server at " << host_ip_str);

    LOG << "Successfully connected." << std::endl;

    vicon_client.SetStreamMode(ViconSDK::StreamMode::ClientPull);
    vicon_client.SetAxisMapping(ViconSDK::Direction::Forward,
                                ViconSDK::Direction::Left,
                                ViconSDK::Direction::Up);
    vicon_client.EnableSegmentData(); // necessary?
    vicon_client.EnableMarkerData();  // necessary?

    nanosleep(&ts, nullptr); // necessary?

    // Create threads to connect to the robots

    /* Wait to connect */
}

std::vector<RobotInfo> RealKheperaIVExperimentLoopFunctions::CollectTrackedRobotPoses()
{
    /*
        The code here is adapted from
        https://github.com/NESTLab/Vicon/blob/38ca8d7b52a7a727e8d37c2fb49c1b2058a8ead7/argos3/plugins/simulator/physics_engines/tracking/tracking_updaters/vicon_updater.cpp
    */
    std::string subject_name, segment_name, robot_name, robot_type, robot_options;
    UInt16 subject_count, segment_count;
    std::vector<RobotInfo> robot_info_vec;
    vicon_client.GetFrame();

    ViconSDK::Output_GetSubjectCount vicon_subject_count_output = vicon_client.GetSubjectCount();
    subject_count = vicon_subject_count_output.SubjectCount;

    // LOG << "There are " << subject_count << " subjects." << std::endl;
    for (UInt16 subject_ind = 0; subject_ind < subject_count; subject_ind++)
    {
        subject_name = vicon_client.GetSubjectName(subject_ind).SubjectName;
        segment_count = vicon_client.GetSegmentCount(subject_name).SegmentCount;

        if (segment_count > 0)
        {
            segment_name = vicon_client.GetSegmentName(subject_name, 0).SegmentName;

            // Get position in mm
            ViconSDK::Output_GetSegmentGlobalTranslation translation =
                vicon_client.GetSegmentGlobalTranslation(subject_name,
                                                         segment_name);

            // Get orientation
            ViconSDK::Output_GetSegmentGlobalRotationQuaternion orientation =
                vicon_client.GetSegmentGlobalRotationQuaternion(subject_name,
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

                RobotInfo robot_info(robot_type, robot_name, origin, rotation, robot_options);
                LOG << "debug segment_name=" << segment_name << " subject_name=" << subject_name << std::endl;
                robot_info_vec.push_back(robot_info);
                LOG << subject_name << " is at:" << origin.GetX() << "," << origin.GetY() << std::endl
                    << " with rotation " << rotation << std::endl;
            }
        }
    }
    return robot_info_vec;
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
// void RealKheperaIVExperimentLoopFunctions::PopulateSocketAddresses()
// {

// std::string strAddress, strId;
// UInt32 unEntityIndex;

// for (UInt32 unInd = 0; unInd < m_ptMeta->NumRobots; ++unInd) {
//    SPose& sSinglePose = m_ptPoses[unInd];

//    // Create socket address
//    strId = std::to_string(unInd);
//    strAddress = SOCKET_PREFIX + strId;

//    // Populate socket addresses into maps
//    m_tMapIdToIndexAddr[strId].second = strAddress;
//    m_mapSocketAddrToFileDesc[strAddress] = -1; // negative value to indicate uninitialized file descriptors
//    m_mapSocketAddrToMutexesRx[strAddress];
//    m_mapSocketAddrToMutexesTx[strAddress];
//    m_mapSocketAddrToDataRx[strAddress];
//    m_mapSocketAddrToDataTx[strAddress];

//    /* Get a reference to the EMANE entity with the ID same as the current loop index */
//    unEntityIndex = m_tMapIdToIndexAddr[strId].first; // get global entity index
//    CEMANEEquippedEntity& cEMANEEntity = *reinterpret_cast<CEMANEEquippedEntity *>(GetSpace().GetEntityVector()[unEntityIndex]);

//    // Populate struct
//    sSinglePose.Id = unInd;
//    strcpy(sSinglePose.Addr, strAddress.c_str());
// }
// }

void RealKheperaIVExperimentLoopFunctions::PreStep()
{
    CollectTrackedRobotPoses();
}

void RealKheperaIVExperimentLoopFunctions::PostStep()
{
}

void RealKheperaIVExperimentLoopFunctions::PostExperiment()
{
}

void RealKheperaIVExperimentLoopFunctions::SetupExperiment()
{
}

void RealKheperaIVExperimentLoopFunctions::CreateThreads()
{

    SInt32 socket_file_descriptor;

    // Create RX threads (receive data to log locally on the ARGoS server)

    // Create TX threads (send pose data to robots)
}

REGISTER_LOOP_FUNCTIONS(RealKheperaIVExperimentLoopFunctions, "real_kheperaiv_experiment_loop_functions")