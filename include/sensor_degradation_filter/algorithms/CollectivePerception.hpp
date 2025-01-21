#ifndef COLLECTIVE_PERCEPTION_HPP
#define COLLECTIVE_PERCEPTION_HPP

#include <string>
#include <vector>
#include <cmath>
#include <numeric>
#include <unordered_map>
#include <memory>
#include <deque>
#include <stdexcept>

#define ZERO_APPROX 1.0e-6

class CollectivePerception
{
public:
    struct EstConfPair
    {
        /**
         * @brief Construct a new EstConfPair struct
         *
         */
        EstConfPair() : Id(""), X(0.0), Confidence(ZERO_APPROX) {}

        /**
         * @brief Construct a new EstConfPair struct
         *
         * @param x
         * @param confidence
         */
        EstConfPair(const double &x, const double &confidence) : Id(""), X(x), Confidence(confidence) {}

        inline EstConfPair operator/(const double &val)
        {
            return EstConfPair(X / val, Confidence / val);
        }

        inline EstConfPair operator+(const EstConfPair &obj)
        {
            return EstConfPair(this->X + obj.X, this->Confidence + obj.Confidence);
        }

        inline EstConfPair operator-(const EstConfPair &obj)
        {
            return EstConfPair(this->X - obj.X, this->Confidence - obj.Confidence);
        }

        friend inline bool operator==(const EstConfPair &lhs, const EstConfPair &rhs)
        {
            if (lhs.X == rhs.X && lhs.Confidence == rhs.Confidence)
            {
                return true;
            }

            else
            {
                return false;
            }
        }

        std::string Id; ///< Robot ID associated with the X and Confidence values

        double X; ///< Estimate value

        double Confidence; ///< Confidence value
    };

    struct Params
    {
        void Reset()
        {
            NumBlackTilesSeen = 0;
            NumObservations = 0;
            ObservationQueue.clear();
            InformedEstimateHistory.clear();
        }

        void AddToQueue(unsigned int obs, int dynamic_queue_size)
        {
            /*
                The way this works is that the actual queue (ObservationQueue) that stores observation grows in size until MaxObservationQueueSize.
                If a dynamic queue is desired, the dynamic queue size is used to accumulate the observations from ObservationQueue without
                shrinking or growing ObservationQueue.
            */

            ObservationQueue.push_back(obs);

            // Adjust queue items to the maximum queue size
            if (ObservationQueue.size() > MaxObservationQueueSize)
            {
                ObservationQueue.pop_front();
            }

            // Adjust the number of black tiles seen depending on whether a dynamic queue is used
            if (dynamic_queue_size > 0 && dynamic_queue_size < ObservationQueue.size())
            {
                NumBlackTilesSeen = std::accumulate(ObservationQueue.end() - dynamic_queue_size, ObservationQueue.end(), 0);
                NumObservations = dynamic_queue_size;
            }
            else if (dynamic_queue_size = -1) // not using dynamic queue sizes
            {
                // Calculate number of black tiles seen
                NumBlackTilesSeen = std::accumulate(ObservationQueue.begin(), ObservationQueue.end(), 0);
                NumObservations = ObservationQueue.size();
            }
            else
            {
                throw std::runtime_error("Unknown dynamic queue size = " + std::to_string(dynamic_queue_size));
            }
        }

        void ComputeWeightedAverageFillRatioReference(double latest_informed_estimate)
        {
            // Store the most recent informed estimate
            InformedEstimateHistory.push_back(latest_informed_estimate);

            if (InformedEstimateHistory.size() > MaxInformedEstimateHistoryLength)
            {
                InformedEstimateHistory.pop_front();
            }

            // Compute the weighted average (weighing older estimates as heavier)
            double numerator = 0.0;
            double denominator = InformedEstimateHistory.size() * (InformedEstimateHistory.size() + 1.0) / 2.0;

            for (size_t i = InformedEstimateHistory.size(); i > 0; --i)
            {
                numerator += i * InformedEstimateHistory[InformedEstimateHistory.size() - i];
            }

            WeightedAverageInformedEstimate = numerator / denominator;
        }

        unsigned int NumBlackTilesSeen = 0;

        unsigned int NumObservations = 0;

        unsigned int MaxObservationQueueSize = 0;

        std::deque<unsigned int> ObservationQueue;

        std::deque<double> InformedEstimateHistory;

        size_t MaxInformedEstimateHistoryLength = 0;

        std::vector<EstConfPair> MostRecentNeighborEstimates;

        bool UseObservationQueue = false;

        double WeightedAverageInformedEstimate = 0.0;

        double PastWeightedAverageInformedEstimate = 0.0;
    };

    CollectivePerception() : params_ptr_(std::make_shared<Params>()) {}

    void Reset()
    {
        local_vals_ = EstConfPair(0.5, ZERO_APPROX);
        social_vals_ = EstConfPair();
        informed_vals_ = EstConfPair();
        params_ptr_->Reset();
    }

    void ComputeLocalEstimate(const double &sensor_acc_b, const double &sensor_acc_w);

    void ComputeSocialEstimate(const std::vector<EstConfPair> &neighbor_vals);

    void ComputeInformedEstimate();

    EstConfPair GetLocalVals() const { return local_vals_; }

    EstConfPair GetSocialVals() const { return social_vals_; }

    EstConfPair GetInformedVals() const { return informed_vals_; }

    std::shared_ptr<Params> GetParamsPtr() const { return params_ptr_; }

private:
    EstConfPair local_vals_ = EstConfPair(0.5, ZERO_APPROX); ///< Local values

    EstConfPair social_vals_ = EstConfPair(); ///< Social values

    EstConfPair informed_vals_ = EstConfPair(0.5, ZERO_APPROX); ///< Informed values

    std::shared_ptr<Params> params_ptr_; ///< Pointer to parameters struct
};

#endif