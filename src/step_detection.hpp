#ifndef STEP_DETECTION_HPP
#define STEP_DETECTION_HPP
#include <vector>
#include <numeric>
#include <cmath>
#include <lsm6dsv.hpp>
/// \brief Real time step detection class
class StepDetector
{
public:
/// \brief Basic constructor for step detection class
/// \param window_size - The size of the moving average window
/// \param threshold - The IMU threshold for step detection
/// \param refractory_period - The minimum time in seconds that steps must be apart
/// \param frequency - The frequency at which this class is updated in Hz
StepDetector(int window_size, double theshold, double refractory_period = 0.2, int frequency = 100);

/// \brief Update the moving average filter
/// \param acc - The new accelerations to add in to moving average window
void filter_update(accelerations acc);

/// \brief Get the cadence or time between steps
/// \return The candence in steps per second
double get_cadence();

private:

/// \brief Performs simple thresholding to detect steps on filtered data
/// \returns Boolean indicating if a step has been detected
bool detect_steps();

/// \brief Update the cadence using the times since last step
/// \note Will reset cadence to zero if step is not detected within 2 seconds
void update_cadence();

/// \brief Calculate the magnitude of an acceleration signal
double mag(accelerations acc);

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
/// \brief The number of data points stored in the acc_buffer_
int data_points_;
/// \brief The current head of the circular buffer, where the next datapoint will be saved
int head_;
/// \brief The result of the step detection algorithm
bool result_;
/// \brief The cadence of the user in steps per second
double cadence_;
/// \brief The buffer of size window_size_ that holds all the acc_mag results to be used in the mav filter
std::vector<double> acc_buffer_;
/// \brief The previous, current, and next mav result
double prev_mav_, curr_mav_, next_mav_;


/// \brief The size of the mav_result_buffer_
static constexpr int step_detect_buff_size_ = 3;
/// \brief The period in seconds for which the cadence is set to 0 if it exceeds this time
static constexpr double reset_period_ = 2.0;
};
#endif