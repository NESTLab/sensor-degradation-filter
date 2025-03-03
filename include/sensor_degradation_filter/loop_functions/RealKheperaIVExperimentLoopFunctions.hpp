#ifndef REAL_KHEPERAIV_EXPERIMENT_LOOP_FUNCTIONS_HPP
#define REAL_KHEPERAIV_EXPERIMENT_LOOP_FUNCTIONS_HPP

#include <mutex>
#include <vector>
#include <atomic>

#include "vicon_sdk/DataStreamClient.h"
#include "loop_functions/SensorDegLoopFunctions.hpp"
#include "messages/RobotServerMessage.hpp"

using namespace argos;
using namespace nlohmann;

namespace ViconSDK = ViconDataStreamSDK::CPP;

using FilterSpecificParams = std::unordered_map<std::string, std::string>;

struct RobotInfo
{
    std::string RobotType;
    std::string RobotName;
    CVector3 RobotPosition;
    CQuaternion RobotOrientation;
    std::string Options;

    RobotInfo() {}

    RobotInfo(std::string type_str,
              std::string name_str,
              const CVector3 &origin,
              const CQuaternion &rotation,
              std::string options_str = "")
        : RobotType(std::move(type_str)),
          RobotName(std::move(name_str)),
          RobotPosition(origin),
          RobotOrientation(rotation),
          Options(std::move(options_str)) {};
};

class RealKheperaIVExperimentLoopFunctions : public SensorDegLoopFunctions
{
public:
    virtual ~RealKheperaIVExperimentLoopFunctions();

    virtual void Init(TConfigurationNode &t_tree) override;

    inline void Reset() override { THROW_ARGOSEXCEPTION("`Reset` functionality is simulated and is not available for real robot experiments."); }

    virtual void PreStep();

    virtual void PostStep() {}

    virtual void PostExperiment() override;

    virtual void SetupExperiment() override { THROW_ARGOSEXCEPTION("`Setup` functionality is simulated and is not available for real robot experiments."); }

    void InitializeExperimentalParameters(TConfigurationNode &t_tree);

    virtual CColor GetFloorColor(const CVector2 &c_position_on_plane) override { THROW_ARGOSEXCEPTION("`GetFloorColor` functionality is simulated and is not available for real robot experiments."); }

protected:
    void ConnectToViconServer(std::string host_ip_str, SInt32 port_number);

    void StartARGoSServer(SInt32 port_number);

    void AcceptConnections();

    void SendPoseInfoToRobots();

    void RobotThreadRx(SInt32 client_socket, const std::string &client_ip);

    void RobotThreadTx(SInt32 client_socket, const std::string &client_ip);

    void ViconThreadRx();

    void GetRobotTypeFromName(const std::string &original_name_str,
                              std::string &type_str,
                              std::string &name_str,
                              std::string &options_str);

    void CreateThreads();

    void ResetRobotPositions() = delete;

    void CollectTrackedRobotPoses();

private:
    ViconSDK::Client vicon_client_;

    std::atomic<bool> shutdown_flag_{false};

    std::mutex connection_mutex_; // mutex to protect num_connected_robots_ and robot_socket_vec_

    std::mutex data_mutex_; // mutex to protect robot_ip_data_map_

    std::mutex pose_mutex_; // mutex to protect robot_name_info_map_

    std::unordered_map<std::string, SInt32> robot_ip_socket_map_; // map that connects robot IP to its TCP socket

    std::unordered_map<std::string, std::vector<std::string>> robot_ip_data_map_; // map that connects robot IP to its logged data

    std::unordered_map<std::string, std::string> robot_ip_name_map_; // map that connects robot IP to its network name

    std::unordered_map<std::string, RobotInfo> robot_name_info_map_; // map that connects robot name to its pose information

    size_t robot_to_server_msg_size_; // size in bytes the message packet from the robot to the server

    size_t server_to_robot_msg_size_; // size in bytes the message packet from the server to the robot

    UInt8 robot_start_flag_ = 0; // flag to indicate whether the robot should start moving (i.e., whether the experiment has begun)

    SInt32 server_socket_; // ARGoS socket server

    SInt32 num_connected_robots_ = 0; // amount of robots connected to the ARGoS server

    SInt32 expected_num_robot_connections_ = 0; // number of robots to expected to connect
};

#endif