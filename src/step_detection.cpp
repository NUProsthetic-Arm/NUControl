#include <step_detection.hpp>

StepDetector::StepDetector(int window_size, double threshold, double refractory_period, int frequency)
: window_size_ (window_size),
  threshold_ (threshold),
  refractory_period_ (refractory_period),
  frequency_ (frequency),
  count_since_step_ (0),
  data_points_ (0),
  head_ (0),
  result_ (false),
  cadence_ (0.0),
  prev_mav_ (0.0),
  curr_mav_ (0.0),
  next_mav_ (0.0)
{
  // resize buffer to set correct length and fill with zeros
  acc_buffer_.resize(window_size_);
}

void StepDetector::filter_update(accelerations acc)
{
  // add the magnitude of the acc into the buffer
  acc_buffer_.at(head_) = mag(acc);

  // update head position
  head_ = (head_ + 1) % window_size_;

  // update mav variables
  prev_mav_ = curr_mav_;
  curr_mav_ = next_mav_;
  next_mav_ = get_mav_result();

  // check if buffer is not yet filled
  if (data_points_ < window_size_) {
    // do not perform step detection while mav is not filled
    result_ = false;

    // increment data_points_
    data_points_++;
  } else {
    // run step detection algorithm
    result_ = detect_steps();
  }  

  // update cadence
  update_cadence();
}

double StepDetector::get_cadence()
{
  return cadence_;
}

bool StepDetector::detect_steps()
{
  // check if threshold requirement is satisfied
  if (curr_mav_ > threshold_)
  {
    // check if peak requirement is satisfied
    if ((curr_mav_ > prev_mav_) && (curr_mav_ > next_mav_)) {
      return true;
    }
  }

  return false;
}

void StepDetector::update_cadence()
{
  // check for timeout on cadence
  if (count_since_step_> frequency_ * reset_period_) {
    cadence_ = 0.0;
  }
  // update count_since_step accordinly
  if (result_) {
    // enforce refactory period for step detection
    if (count_since_step_ > frequency_ * refractory_period_) {
        cadence_ = double(frequency_) / double(count_since_step_);
        count_since_step_ = 0;
    } else {
        // increment since we aren't counting this step
        count_since_step_++;

        // flip back the result since we aren't counting this step
        result_ = false;
    }
  } else {
    count_since_step_++;
  }

}


double StepDetector::mag(accelerations acc)
{
  return std::sqrt(std::pow(acc.x, 2) + std::pow(acc.y, 2) + std::pow(acc.z, 2));
}

double StepDetector::get_mav_result()
{
  return std::accumulate(acc_buffer_.begin(), acc_buffer_.end(), 0.0) / double(acc_buffer_.size());
}
