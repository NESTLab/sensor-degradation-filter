#ifndef REAL_KHEPERAIV_EXPERIMENT_LOOP_FUNCTIONS_HPP
#define REAL_KHEPERAIV_EXPERIMENT_LOOP_FUNCTIONS_HPP

#include "vicon_sdk/DataStreamClient.h"

#include "loop_functions/SensorDegLoopFunctions.hpp"

using namespace argos;
using namespace nlohmann;

namespace ViconSDK = ViconDataStreamSDK::CPP;

struct RobotInfo
{
    const std::string RobotType;
    const std::string RobotName;
    const CVector3 RobotPosition;
    const CQuaternion RobotOrientation;
    const std::string Options;

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
    virtual ~RealKheperaIVExperimentLoopFunctions()
    {
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
    void GetRobotTypeFromName(const std::string &original_name_str,
                              std::string &type_str,
                              std::string &name_str,
                              std::string &options_str);

    void CreateThreads();

    void ResetRobotPositions() = delete;

    std::vector<RobotInfo> CollectTrackedRobotPoses();

private:
    ViconSDK::Client vicon_client;
};

#endif