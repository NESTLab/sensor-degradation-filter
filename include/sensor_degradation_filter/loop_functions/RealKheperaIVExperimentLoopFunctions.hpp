#ifndef REAL_KHEPERAIV_EXPERIMENT_LOOP_FUNCTIONS_HPP
#define REAL_KHEPERAIV_EXPERIMENT_LOOP_FUNCTIONS_HPP

#include <mutex>
#include <vector>
#include <atomic>

#include "vicon_sdk/DataStreamClient.h"
#include "loop_functions/SensorDegLoopFunctions.hpp"

using namespace argos;
using namespace nlohmann;

namespace ViconSDK = ViconDataStreamSDK::CPP;

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

struct Message
{
    UInt16 StartFlag;
    CByteArray Payload;
};

class RealKheperaIVExperimentLoopFunctions : public SensorDegLoopFunctions
{
public:
    virtual ~RealKheperaIVExperimentLoopFunctions()
    {
        // Set shutdown flag to stop the listener thread
        shutdown_flag_.store(true, std::memory_order_release);

        ::close(server_socket_);
    }

    virtual void Init(TConfigurationNode &t_tree) override;

    inline void Reset() override { THROW_ARGOSEXCEPTION("Reset functionality is not available for experiments with real robots."); }

    virtual void SetupExperiment() override;

    virtual void PreStep();

    virtual void PostStep() override;

    virtual void PostExperiment() override;

    // virtual void SaveData() override;

    virtual CColor GetFloorColor() = delete;

    /**
     * @brief Convert the data into a string so that it can be stored into JSON later
     *
     * @param data Vector of Real values
     * @return Data string
     */
    std::string ConvertDataToString(const std::vector<Real> &data);

protected:
    void ConnectToViconServer(std::string host_ip_str, SInt32 port_number);

    void StartServer(SInt32 port_number);

    void AcceptConnections();

    void SendPoseInfoToRobots();

    void ListenToRobot(SInt32 client_socket, const std::string &client_ip);

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

    std::unordered_map<std::string, SInt32> robot_ip_socket_map_;

    std::unordered_map<std::string, std::vector<std::string>> robot_ip_data_map_;

    std::unordered_map<std::string, std::string> robot_ip_name_map_;

    std::unordered_map<std::string, RobotInfo> robot_name_info_map_;

    std::vector<std::thread> robot_thread_vec_;

    size_t robot_data_size_;

    size_t num_robots_;

    UInt8 robot_start_flag_ = 0;

    SInt32 server_socket_;

    SInt32 num_connected_robots_ = 0;

    SInt32 expected_num_robot_connections_ = 0;
};

#endif