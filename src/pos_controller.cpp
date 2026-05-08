#include "pos_controller.hpp"

PositionController::PositionController(float Kp, float Ki, float Kd, float Kff_gravity, float Kff_spring)
: Kp_(Kp),
  Ki_(Ki),
  Kd_(Kd),
  Kff_gvty_(Kff_gravity),
  Kff_spring_(Kff_spring),
  err_ (0.0f),
  err_int_ (0.0f),
  err_der_ (0.0f),
  err_prev_ (0.0f),
  i_clamp_val_ (5.0f),
  u_clamp_val_ (1000.0f),
  gvty_fwd_enable_ (false),
  gvty_term_ (0.0f),
  spring_fwd_enable_ (false),
  spring_term_ (0.0f),
  theta_rest_ (0.0f),
  dt_ (0.001),
  err_der_raw_ (0.0),
  err_der_filt_ (0.0),
  alpha_d_ (0.01)
{
    Serial.println("Initializing PID position controller...");
}

void PositionController::set_gvty_ffwd_control(bool enable)
{
    gvty_fwd_enable_ = enable;
}

void PositionController::set_spring_ffwd_control(bool enable)
{
    spring_fwd_enable_ = enable;
}

void PositionController::set_i_clamp_val(float clamp_val)
{
    i_clamp_val_ = clamp_val;
}

void PositionController::set_u_clamp_val(float clamp_val)
{
    u_clamp_val_ = clamp_val;
}

void PositionController::set_theta_rest_val(float theta_rest)
{
    theta_rest_ = theta_rest;
}
float PositionController::pump_controller(float setpoint, float actual, float shaft_vel)
{
    // calculate error
    err_ = (setpoint - actual);

    if (gvty_fwd_enable_) {gvty_term_ = std::sin(actual);}
    if (spring_fwd_enable_) {spring_term_ = theta_rest_ - actual;}
    if (Ki_ != 0.0) {err_int_ += err_ * dt_;}
    // if (Kd_ != 0.0) {err_der_ = -(shaft_vel);} 

    if (Kd_ != 0.0) {
        err_der_raw_ = (err_ - err_prev_) / dt_;
        err_der_filt_ = alpha_d_ * err_der_raw_ + (1.0f - alpha_d_) * err_der_filt_;
        err_der_ = err_der_filt_;
    }    
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

    auto u = Kp_ * err_ + Ki_ * err_int_ + Kd_ * err_der_ + Kff_gvty_ * gvty_term_ + Kff_spring_ * spring_term_;

    // clamp command
    if (u > u_clamp_val_)
    {
        u = u_clamp_val_;
    } 
    else if (u < -u_clamp_val_)
    {
        u = -u_clamp_val_;
    }

    return u;
}