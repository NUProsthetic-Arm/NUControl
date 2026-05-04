#include <heel_strike_filtering.hpp>

HeelStrikeFilter::HeelStrikeFilter(int window_size, float threshold, float refractory_period, int frequency)
: window_size_ (window_size),
  threshold_ (threshold),
  refractory_period_ (refractory_period),
  frequency_ (frequency),
  count_since_step_ (0),
  data_points_ (0),
  head_ (0),
  result_ (false),
  period_ (0.0f),
  hs_classification_ (Classification::UNKNOWN),
  prev_mav_ (0.0f),
  curr_mav_ (0.0f),
  next_mav_ (0.0f),
  l_mu_ (-0.003059f),
  r_mu_ (0.101622f),
  l_sigma_ (0.029304f),
  r_sigma_ (0.031433f),
  expectation_ (Classification::LEFT),
  confidence_ (0),
  max_confidence_ (10),
  sum_ (0.0),
  prev_sum_ (0.0),
  feature_ (0.0)
{
  // resize buffers to set correct length and fill with zeros
  mag_buffer_.resize(window_size_);
  mediolat_buffer_.resize(window_size_);
}

void HeelStrikeFilter::filter_update(accelerations acc)
{
  // add the magnitude of the acc into the buffer
  mag_buffer_.at(head_) = mag(acc);
  mediolat_buffer_.at(head_) = medio_lateral_acc(acc);

  // update head position
  head_ = (head_ + 1) % window_size_;

  // update mav variables
  prev_mav_ = curr_mav_;
  curr_mav_ = next_mav_;
  next_mav_ = get_mav_result();

  // integrate medio lateral
  sum_ += get_medio_lateral_mav_result();

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
      // classify_step2();
      // update_confidence();
    }
  }  

  // update period
  update_period();
}

const HeelStrikeResult HeelStrikeFilter::get_result()
{
  return HeelStrikeResult{heel_strike_count_, hs_classification_, period_, confidence_, feature_};
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

// update the confidence in our classifications
void HeelStrikeFilter::update_confidence() {
  if (expectation_ == hs_classification_) {
    // if (confidence_ > 1) {
    //   // extra reward for successive correct expectations
    //   confidence_++;
    // }
    // if expectation aligns with result, increment confidence
    confidence_++;

    // flip expectation
    expectation_ = flip_expectation(expectation_);
  } else {
    if (hs_classification_ == Classification::UNKNOWN){
      // if heel strike result is unknown, decrement by 1
      // confidence_--;  // DON"T Decrement
    } else {
      // if heel strike result is OPPOSITE, decrement by 2
      confidence_-=1;  // actually decrement by 1
    }

    if (confidence_ > 0) {
      // if we still have some confidence left, flip expectation. otherwise don't because we might be wrong at this point
      expectation_ = flip_expectation(expectation_);
    }
  }

  // clamp value of confidence
  if (confidence_ < 0) {
    confidence_ = 0;
  } else if (confidence_ > max_confidence_) {
    confidence_ = max_confidence_;
  }
}

void HeelStrikeFilter::update_period()
{
  // check for timeout on cadence
  if (count_since_step_> frequency_ * reset_period_) {
    period_ = 0.0f;
    confidence_ = 0;
    sum_ = 0.0f;
    prev_sum_ = 0.0f;
    hs_classification_ = Classification::UNKNOWN;
  }
  // update count_since_step accordingly
  if (result_) {
    // enforce refactory period for step detection
    if (count_since_step_ > frequency_ * refractory_period_) {
        // step is now confirmed, classify and update period
        period_ = float(count_since_step_) / float(frequency_);
        classify_step2();
        update_confidence();
        heel_strike_count_++;
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
  auto data_point = get_medio_lateral_mav_result();

  // get likelyhood from gaussian pdf
  auto likelyhood_R = gaussian_pdf(data_point, r_mu_, r_sigma_);
  auto likelyhood_L = gaussian_pdf(data_point, l_mu_, l_sigma_);

  // calculate likelyhood ratio
  auto likelyhood_ratio = likelyhood_R/likelyhood_L;

  // classify based on result
  if (likelyhood_ratio > 2.0f) {
    hs_classification_ = Classification::RIGHT;
  }
  else if (likelyhood_ratio < 0.5f) {
    hs_classification_ = Classification::LEFT;
  } else {
    hs_classification_ = Classification::UNKNOWN;
  }
}

void HeelStrikeFilter::classify_step2()
{
  // normalize
  feature_ = (sum_ - prev_sum_) / count_since_step_;

  // reset sum
  prev_sum_ = sum_;
  sum_ = 0.0f;

  // classify baseed on result
  if (feature_ > .01f) {
    hs_classification_ = Classification::LEFT;
  }
  else if (feature_ < -.01f) {
    hs_classification_ = Classification::RIGHT;
  } else {
    hs_classification_ = Classification::UNKNOWN;
  }
}

float HeelStrikeFilter::mag(accelerations acc)
{
  return std::sqrt(std::pow(acc.x, 2) + std::pow(acc.y, 2) + std::pow(acc.z, 2));
}

float HeelStrikeFilter::medio_lateral_acc(accelerations acc)
{
  constexpr float s = 0.90630778703f;  // sin(65°)
  constexpr float c = 0.42261826174f;  // cos(65°)
  return acc.x * s + acc.y * c;
}

float HeelStrikeFilter::gaussian_pdf(float dp, float mu, float sigma)
{
  return std::exp(-0.5 * std::pow((dp - mu) / sigma, 2)) / (sigma * std::sqrt(2.0 * M_PI));
}

float HeelStrikeFilter::get_mav_result()
{
  return std::accumulate(mag_buffer_.begin(), mag_buffer_.end(), 0.0) / float(mag_buffer_.size());
}

float HeelStrikeFilter::get_medio_lateral_mav_result()
{
  return std::accumulate(mediolat_buffer_.begin(), mediolat_buffer_.end(), 0.0) / float(mediolat_buffer_.size());
}

