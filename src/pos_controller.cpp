#include "pos_controller.hpp"

PositionController::PositionController(double Kp, double Ki, double Kd, double Kff)
: Kp_(Kp),
  Ki_(Ki),
  Kd_(Kd),
  Kff_(Kff),
  err_ (0.0),
  err_int_ (0.0),
  err_der_ (0.0),
  err_prev_ (0.0),
  i_clamp_val_ (5.0),
  u_clamp_val_ (1000.0),
  feed_fwd_enable_ (false),
  ffwd_term_ (0.0)
{
    Serial.println("Initializing PID position controller...");
}

void PositionController::set_ffwd_control(bool enable)
{
    feed_fwd_enable_ = enable;
}

void PositionController::set_i_clamp_val(double clamp_val)
{
    i_clamp_val_ = clamp_val;
}

void PositionController::set_u_clamp_val(double clamp_val)
{
    u_clamp_val_ = clamp_val;
}

double PositionController::pump_controller(double setpoint, double actual, float shaft_vel)
{
    // calculate error
    err_ = setpoint - actual;
    // if (feed_fwd_enable_) {ffwd_term_ = next_cmd;}
    if (Ki_ != 0.0) {err_int_ += err_;}
    if (Kd_ != 0.0) { err_der_ = -(shaft_vel);} 
    err_prev_ = err_;

    // clamp err_int
    if ((err_int_ > i_clamp_val_) && (Ki_ != 0.0))
    {
        err_int_ = i_clamp_val_;
        Serial.println("clamping high");
    } 
    else if ((err_int_ < -i_clamp_val_) && (Ki_ != 0.0))
    {
        err_int_ = -i_clamp_val_;
        Serial.println("clamping low");

    }

    auto u = Kp_ * err_ + Ki_ * err_int_ + Kd_ * err_der_;

    // clamp command
    if (u > u_clamp_val_)
    {
        u = u_clamp_val_;
    } 
    else if (u < -u_clamp_val_)
    {
        u = -u_clamp_val_;
    }

    return u + + Kff_ * ffwd_term_;
}