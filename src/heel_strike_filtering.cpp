#include <step_detection.hpp>

HeelStrikeFilter::HeelStrikeFilter(int window_size, double threshold, double refractory_period, int frequency)
: window_size_ (window_size),
  threshold_ (threshold),
  refractory_period_ (refractory_period),
  frequency_ (frequency),
  count_since_step_ (0),
  data_points_ (0),
  head_ (0),
  result_ (false),
  cadence_ (0.0),
  hs_classification_ (0),
  prev_mav_ (0.0),
  curr_mav_ (0.0),
  next_mav_ (0.0),
  l_mu_ (-0.003059),
  r_mu_ (0.101622),
  l_sigma_ (0.029304),
  r_sigma_ (0.031433)
{
  // resize buffers to set correct length and fill with zeros
  mag_buffer_.resize(window_size_);
  antpost_buffer_.resize(window_size_);
}

void HeelStrikeFilter::filter_update(accelerations acc)
{
  // add the magnitude of the acc into the buffer
  mag_buffer_.at(head_) = mag(acc);
  antpost_buffer_.at(head_) = anterio_posterior_acc(acc);

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

    // classify if step detected
    if (result_) {
      classify_step();
    }
  }  

  // update cadence
  update_cadence();
}

double HeelStrikeFilter::get_cadence()
{
  return cadence_;
}

int HeelStrikeFilter::get_classification()
{
  return hs_classification_;
}


bool HeelStrikeFilter::detect_steps()
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

void HeelStrikeFilter::update_cadence()
{
  // check for timeout on cadence
  if (count_since_step_> frequency_ * reset_period_) {
    cadence_ = 0.0;
  }
  // update count_since_step accordingly
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

void HeelStrikeFilter::classify_step()
{
  // get mav result
  auto data_point = get_mav_result();

  // get likelyhood from gaussian pdf
  auto likelyhood_R = gaussian_pdf(data_point, r_mu_, r_sigma_)
  auto likelyhood_L = gaussian_pdf(data_point, l_mu_, l_sigma_)

  // calculate likelyhood ratio
  auto likelyhood_ratio = likelyhood_R/likelyhood_L;

  // classify based on result
  if (likelyhood_ratio > 2.0) {
    hs_classification_ = 1;
  }
  else if (likelyhood_ratio < 0.5) {
    hs_classification_ = -1;
  } else {
    hs_classification_ = 0;
  }
}


double HeelStrikeFilter::mag(accelerations acc)
{
  return std::sqrt(std::pow(acc.x, 2) + std::pow(acc.y, 2) + std::pow(acc.z, 2));
}

double HeelStrikeFilter::anterio_posterior_acc(accelerations acc)
{
  return acc.x * std::sin(65) + acc.y * std::sin(65);
}

double HeelStrikeFilter::gaussian_pdf(double dp, double mu, double sigma)
{
  return std::exp(-0.5 * std::pow((x - mu) / sigma, 2)) / (sigma * std::sqrt(2.0 * M_PI));
}

double HeelStrikeFilter::get_mav_result()
{
  return std::accumulate(mag_buffer_.begin(), mag_buffer_.end(), 0.0) / double(mag_buffer_.size());
}
