#ifndef HEEL_STRIKE_FILTERING_HPP
#define HEEL_STRIKE_FILTERING_HPP
#include <vector>
#include <numeric>
#include <cmath>
#include <lsm6dsv.hpp>
/// \brief Real time step detection and left/right classification
class HeelStrikeFilter
{
public:
/// \brief Basic constructor for step detection class
/// \param window_size - The size of the moving average window
/// \param threshold - The IMU threshold for step detection
/// \param refractory_period - The minimum time in seconds that steps must be apart
/// \param frequency - The frequency at which this class is updated in Hz
HeelStrikeFilter(int window_size, double theshold, double refractory_period = 0.2, int frequency = 100);

/// \brief Update the moving average filter
/// \param acc - The new accelerations to add in to moving average window
void filter_update(accelerations acc);

/// \brief Get the cadence or time between steps
/// \return The candence in steps per second
double get_cadence();

/// \brief Get the classification of the last heel strike 
/// \return Integer classification of heel strike -1 indicates left, 1 indicates right, 0 indicates uncertainty
int get_classification();

private:

/// \brief Performs simple thresholding to detect steps on filtered data
/// \note Calls classify_step if step is detected
/// \returns Boolean indicating if a step has been detected
bool detect_steps();

/// \brief Update the cadence using the times since last step
/// \note Will reset cadence to zero if step is not detected within 2 seconds
void update_cadence();

/// \brief Calculates likelyhood ratio of right heel strike by sampling from distributions of right and left anterio-posterior accelerations
void classify_step();

/// \brief Calculate the magnitude of an acceleration signal
double mag(accelerations acc);

/// \brief Calculate the anterio-posterior (front to back) of an acceleration signal with embedded rotation from IMU position
double anterio_posterior_acc(accelerations acc);

/// \brief Calculates likelyhood of a data point from a gaussian distribution with specified mean and standard deviation
/// \returns The likelyhood of the point occuring in the given distribution
double gaussian_pdf(double dp, double u, double stdv);

/// \brief Calculate the moving average from acc buffer
/// \returns The moving average filter output
double get_mav_result();

/// \brief The size of the moving average window
int window_size_;
/// \brief The IMU threshold for step detection
double threshold_;
/// \brief The minimum time in seconds that steps must be apart
double refractory_period_;
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
/// \brief The cadence of the user in steps per second
double cadence_;
/// \brief The most recent heel strike classification
double hs_classification_;
/// \brief The buffer of size window_size_ that holds all the acc_mag results to be used in the mav filter
std::vector<double> mag_buffer_;
/// \brief The buffer of size window_size_ that holds all the anterio_posterior_acc results to be used in the mav filter
std::vector<double> antpost_buffer_;
/// \brief The previous, current, and next mav result
double prev_mav_, curr_mav_, next_mav_;
/// \brief The parameters describing the distributions of anterio-posterior accelerations for right and left heel strikes
double l_mu_, r_mu_, l_sigma_, r_sigma_;


/// \brief The size of the mav_result_buffer_
static constexpr int step_detect_buff_size_ = 3;
/// \brief The period in seconds for which the cadence is set to 0 if it exceeds this time
static constexpr double reset_period_ = 2.0;
};

#endif