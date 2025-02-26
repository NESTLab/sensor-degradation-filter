#ifndef UTIL_HPP
#define UTIL_HPP

#include <vector>
#include <string>
#include <ctime>
#include <random>    // mt19937, uniform_real_distribution, normal_distribution
#include <algorithm> // sample
#include <stdexcept>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/math/vector3.h>

using namespace argos;

template <typename T>
std::vector<T> GenerateLinspace(const T &min, const T &max, const size_t &steps)
{
    // Compute increment
    T inc = (max - min) / static_cast<T>(steps - 1);

    // Populate vector
    std::vector<T> output(steps);
    T val;

    for (auto itr = output.begin(), val = min; itr != output.end(); ++itr, val += inc)
    {
        *itr = val;
    }

    return output;
}

inline std::string GetCurrentTimeStr()
{
    // Grab current local time
    time_t curr_time;
    time(&curr_time);
    tm *curr_tm = localtime(&curr_time);

    std::string datetime;
    datetime.resize(100);

    // Convert to string
    strftime(&(datetime[0]), datetime.size(), "%m%d%y_%H%M%S", curr_tm);
    return std::string(datetime.c_str());
}

inline std::string Round1000DoubleToStr(const double &val)
{
    return std::to_string(static_cast<int>(std::round(val * 1e3)));
}

/**
 * @brief Draw a sample of robot IDs without replacement
 *
 * @param num_robots_to_sample Number of robots to sample
 * @param robot_id_vec Vector of robot IDs to sample from
 * @return std::vector<std::string> Drawn robot IDs
 */
inline std::vector<std::string> SampleRobotIdsWithoutReplacement(const unsigned int &num_robots_to_sample,
                                                                 const std::vector<std::string> &robot_id_vec)
{
    // Sample random robot IDs (without replacement)
    std::vector<std::string> sampled_robot_ids;

    std::sample(robot_id_vec.begin(),
                robot_id_vec.end(),
                std::back_inserter(sampled_robot_ids),
                num_robots_to_sample,
                std::mt19937{std::random_device{}()});

    return sampled_robot_ids;
}

/*
    RNG copied from the CSpace class:
    https://github.com/ilpincy/argos3/blob/603f6276ffce650c15e7dcaba900bac0bec01d51/src/core/simulator/simulator.cpp
*/
class RealNumberGenerator
{
public:
    virtual ~RealNumberGenerator() {}
    virtual CVector3 operator()(bool b_is_retry) = 0;
};

class ConstantGenerator : public RealNumberGenerator
{
public:
    ConstantGenerator(const CVector3 &c_value) : m_cValue(c_value) {}

    inline virtual CVector3 operator()(bool b_is_retry)
    {
        return m_cValue;
    }

private:
    CVector3 m_cValue;
};

class UniformGenerator : public RealNumberGenerator
{
public:
    UniformGenerator(const CVector3 &c_min,
                     const CVector3 &c_max) : m_cMin(c_min),
                                              m_cMax(c_max) {}
    inline virtual CVector3 operator()(bool b_is_retry)
    {
        Real fRandX =
            m_cMax.GetX() > m_cMin.GetX() ? CSimulator::GetInstance().GetRNG()->Uniform(CRange<Real>(m_cMin.GetX(), m_cMax.GetX())) : m_cMax.GetX();
        Real fRandY =
            m_cMax.GetY() > m_cMin.GetY() ? CSimulator::GetInstance().GetRNG()->Uniform(CRange<Real>(m_cMin.GetY(), m_cMax.GetY())) : m_cMax.GetY();
        Real fRandZ =
            m_cMax.GetZ() > m_cMin.GetZ() ? CSimulator::GetInstance().GetRNG()->Uniform(CRange<Real>(m_cMin.GetZ(), m_cMax.GetZ())) : m_cMax.GetZ();
        return CVector3(fRandX, fRandY, fRandZ);
    }

private:
    CVector3 m_cMin;
    CVector3 m_cMax;
};

class GaussianGenerator : public RealNumberGenerator
{
public:
    GaussianGenerator(const CVector3 &c_mean,
                      const CVector3 &c_std_dev) : m_cMean(c_mean),
                                                   m_cStdDev(c_std_dev) {}
    inline virtual CVector3 operator()(bool b_is_retry)
    {
        return CVector3(CSimulator::GetInstance().GetRNG()->Gaussian(m_cStdDev.GetX(), m_cMean.GetX()),
                        CSimulator::GetInstance().GetRNG()->Gaussian(m_cStdDev.GetY(), m_cMean.GetY()),
                        CSimulator::GetInstance().GetRNG()->Gaussian(m_cStdDev.GetZ(), m_cMean.GetZ()));
    }

private:
    CVector3 m_cMean;
    CVector3 m_cStdDev;
};
class GridGenerator : public RealNumberGenerator
{
public:
    GridGenerator(const CVector3 c_center,
                  const UInt32 un_layout[],
                  const CVector3 c_distances) : m_cCenter(c_center),
                                                m_cDistances(c_distances),
                                                m_unNumEntityPlaced(0)
    {
        m_unLayout[0] = un_layout[0];
        m_unLayout[1] = un_layout[1];
        m_unLayout[2] = un_layout[2];
        /* Check if layout is sane */
        if (m_unLayout[0] == 0 || m_unLayout[1] == 0 || m_unLayout[2] == 0)
        {
            THROW_ARGOSEXCEPTION("'layout' values (distribute position, method 'grid') must all be different than 0");
        }
    }

    virtual CVector3 operator()(bool b_is_retry)
    {
        if (b_is_retry)
        {
            THROW_ARGOSEXCEPTION("Impossible to place entity #" << m_unNumEntityPlaced << " in grid");
        }
        CVector3 cReturn;
        if (m_unNumEntityPlaced < m_unLayout[0] * m_unLayout[1] * m_unLayout[2])
        {
            cReturn.SetX(m_cCenter.GetX() + (m_unLayout[0] - 1) * m_cDistances.GetX() * 0.5 - (m_unNumEntityPlaced % m_unLayout[0]) * m_cDistances.GetX());
            cReturn.SetY(m_cCenter.GetY() + (m_unLayout[1] - 1) * m_cDistances.GetY() * 0.5 - (m_unNumEntityPlaced / m_unLayout[0]) % m_unLayout[1] * m_cDistances.GetY());
            cReturn.SetZ(m_cCenter.GetZ() + (m_unLayout[2] - 1) * m_cDistances.GetZ() * 0.5 - (m_unNumEntityPlaced / (m_unLayout[0] * m_unLayout[1])) * m_cDistances.GetZ());
            ++m_unNumEntityPlaced;
        }
        else
        {
            THROW_ARGOSEXCEPTION("Distribute position, method 'grid': trying to place more entities than allowed "
                                 "by the 'layout', check your 'quantity' tag");
        }
        return cReturn;
    }

private:
    CVector3 m_cCenter;
    UInt32 m_unLayout[3];
    CVector3 m_cDistances;
    UInt32 m_unNumEntityPlaced;
};

/****************************************/
/****************************************/

inline RealNumberGenerator *CreateGenerator(TConfigurationNode &t_tree)
{
    std::string strMethod;
    GetNodeAttribute(t_tree, "method", strMethod);
    if (strMethod == "uniform")
    {
        CVector3 cMin, cMax;
        GetNodeAttribute(t_tree, "min", cMin);
        GetNodeAttribute(t_tree, "max", cMax);
        if (!(cMin <= cMax))
        {
            THROW_ARGOSEXCEPTION("Uniform generator: the min is not less than or equal to max: " << cMin << " / " << cMax);
        }
        return new UniformGenerator(cMin, cMax);
    }
    else if (strMethod == "gaussian")
    {
        CVector3 cMean, cStdDev;
        GetNodeAttribute(t_tree, "mean", cMean);
        GetNodeAttribute(t_tree, "std_dev", cStdDev);
        return new GaussianGenerator(cMean, cStdDev);
    }
    else if (strMethod == "constant")
    {
        CVector3 cValues;
        GetNodeAttribute(t_tree, "values", cValues);
        return new ConstantGenerator(cValues);
    }
    else if (strMethod == "grid")
    {
        CVector3 cCenter, cDistances;
        GetNodeAttribute(t_tree, "center", cCenter);
        GetNodeAttribute(t_tree, "distances", cDistances);
        UInt32 unLayout[3];
        std::string strLayout;
        GetNodeAttribute(t_tree, "layout", strLayout);
        ParseValues<UInt32>(strLayout, 3, unLayout, ',');
        return new GridGenerator(cCenter, unLayout, cDistances);
    }
    else
    {
        THROW_ARGOSEXCEPTION("Unknown distribution method \"" << strMethod << "\"");
    }
}

#endif