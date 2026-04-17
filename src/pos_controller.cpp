#include "pos_controller.hpp"

PositionController::PositionController(float Kp, float Ki, float Kd, float Kff)
: Kp_(Kp),
  Ki_(Ki),
  Kd_(Kd),
  Kff_(Kff),
  err_ (0.0f),
  err_int_ (0.0f),
  err_der_ (0.0f),
  err_prev_ (0.0f),
  i_clamp_val_ (100.0f),
  u_clamp_val_ (1000.0f),
  gvty_fwd_enable_ (false),
  gvty_term_ (0.0f)
{
    Serial.println("Initializing PID position controller...");
}

void PositionController::set_ffwd_control(bool enable)
{
    gvty_fwd_enable_ = enable;
}

void PositionController::set_i_clamp_val(float clamp_val)
{
    i_clamp_val_ = clamp_val;
}

void PositionController::set_u_clamp_val(float clamp_val)
{
    u_clamp_val_ = clamp_val;
}

float PositionController::pump_controller(float setpoint, float actual, float shaft_vel)
{
    // calculate error
    err_ = (setpoint - actual);

    if (gvty_fwd_enable_) {gvty_term_ = std::sin(actual);}
    if (Ki_ != 0.0) {err_int_ += err_;}
    if (Kd_ != 0.0) {err_der_ = (shaft_vel);} 
    
    err_prev_ = err_;

    // clamp err_int
    if ((err_int_ > i_clamp_val_) && (Ki_ != 0.0f))
    {
        err_int_ = i_clamp_val_;
        Serial.println("clamping high");
    } 
    else if ((err_int_ < -i_clamp_val_) && (Ki_ != 0.0f))
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

    return u + Kff_ * gvty_term_;
}