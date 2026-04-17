#ifndef HEEL_STRIKE_FILTERING_HPP
#define HEEL_STRIKE_FILTERING_HPP
#include <vector>
#include <numeric>
#include <cmath>
#include <lsm6dsv.hpp>

/// \brief Enum describing possible outputs of heel strike classification
enum class Classification {
    RIGHT = 1,
    LEFT = -1,
    UNKNOWN = 0
};

/// \brief Struct for holding results of heel strike detection and classification
struct HeelStrikeResult {
    int count;
    Classification classification;
    float period;
    int confidence;
    float sum;
};

/// \brief Flip the expected value of the classification
/// \param expectation - The expectation that should be flipped (can be used externally)
/// \returns The flipped expectation
static Classification flip_expectation(Classification expectation)
{
  if (expectation == Classification::RIGHT) {
    return Classification::LEFT;
  }
  else if (expectation == Classification::LEFT) {
    return Classification::RIGHT;
  } else {
    // not sure if this should be the case
    return Classification::UNKNOWN;
  }
}


/// \brief Real time step detection and left/right classification
class HeelStrikeFilter
{
public:
/// \brief Basic constructor for step detection class
/// \param window_size - The size of the moving average window
/// \param threshold - The IMU threshold for step detection
/// \param refractory_period - The minimum time in seconds that steps must be apart
/// \param frequency - The frequency at which this class is updated in Hz
HeelStrikeFilter(int window_size, float theshold, float refractory_period = 0.4, int frequency = 100);

/// \brief Update the moving average filter
/// \param acc - The new accelerations to add in to moving average window
void filter_update(accelerations acc);

/// \brief Get the most recent heel strike results
/// \note A classification of 1 indiciates right, -1 idicates left, 0 indicates uncertainty
/// \return A HeelStrikeResult struct containing the count of steps, period, and classification of most recent heel strikes
const HeelStrikeResult get_result();

/// \brief Calculate the moving average from acc buffer
/// \returns The moving average filter output
float get_mav_result();

/// \brief Calculate the moving average from medio_lateral_acc buffer
/// \returns The moving average filter output
float get_medio_lateral_mav_result();

private:
/// \brief Update the confidence in our alternating assumption of heel strike classification
void update_confidence();

/// \brief Performs simple thresholding to detect steps on filtered data
/// \note Calls classify_step if step is detected
/// \returns Boolean indicating if a step has been detected
bool detect_steps();

/// \brief Update the period using the times since last step
/// \note Will reset period to zero if step is not detected within 2 seconds
void update_period();

/// \brief Calculates likelyhood ratio of right heel strike by sampling from distributions of right and left anterio-posterior accelerations
void classify_step();

/// \brief Calculates likelyhood ratio of right heel strike by sampling from distributions of right and left anterio-posterior accelerations
void classify_step2();

/// \brief Calculate the magnitude of an acceleration signal
float mag(accelerations acc);

/// \brief Calculate the medio-lateral (left to right) of an acceleration signal with embedded rotation from IMU position
float medio_lateral_acc(accelerations acc);

/// \brief Calculates likelyhood of a data point from a gaussian distribution with specified mean and standard deviation
/// \returns The likelyhood of the point occuring in the given distribution
float gaussian_pdf(float dp, float u, float stdv);

/// \brief The size of the moving average window
int window_size_;
/// \brief The IMU threshold for step detection
float threshold_;
/// \brief The minimum time in seconds that steps must be apart
float refractory_period_;
/// \brief The frequency at which this class is updated in Hz
int frequency_;
/// \brief The count since a step was detected
int count_since_step_;
/// \brief The number of data points stored in the mag_buffer_
int data_points_;
/// \brief The current head of the circular buffer, where the next datapoint will be saved
int head_;
/// \brief The result of the step detection algorithm
bool result_;
/// \brief The period between successive steps in seconds
float period_;
/// \brief The most recent heel strike classification
Classification hs_classification_;
/// \brief The buffer of size window_size_ that holds all the acc_mag results to be used in the mav filter
std::vector<float> mag_buffer_;
/// \brief The buffer of size window_size_ that holds all the medio_lateral_acc results to be used in the mav filter
std::vector<float> mediolat_buffer_;
/// \brief The previous, current, and next mav result
float prev_mav_, curr_mav_, next_mav_;
/// \brief The parameters describing the distributions of anterio-posterior accelerations for right and left heel strikes
float l_mu_, r_mu_, l_sigma_, r_sigma_;
/// \brief The number of heel strikes detected
int heel_strike_count_;
/// \brief The expected output of the next classification
Classification expectation_;
/// \brief The count of confidence in our assumption about the classification
int confidence_;
/// \brief The sum of medio-lateral imu signal since last heel strike
float sum_;
/// \brief The previous sum of medio-lateral imu signal since last heel strike
float prev_sum_;
/// \brief The normalized difference between current and prev sum used for classification
float feature_;
/// \brief The size of the mav_result_buffer_
static constexpr int step_detect_buff_size_ = 3;
/// \brief The period in seconds for which the period is set to 0 if it exceeds this time
static constexpr float reset_period_ = 2.0;
};

#endif