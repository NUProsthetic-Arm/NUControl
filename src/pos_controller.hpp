#ifndef POS_CONTROLLER_HPP
#define POS_CONTROLLER_HPP

#include <Arduino.h>

/// \brief Abstracted position controller for motor control
class PositionController
{
public:
  /// \brief Initializer for a PID controller using ffwd
  /// \param Kp - The proportional gain for a PID controller
  /// \param Ki - The integral gain for a PID controller
  /// \param Kd - The derivative gain for a PID controller
  /// \param Kff - The feed forward gain
  PositionController(float Kp, float Ki, float Kd, float Kff);

  /// \brief Pump controller
  /// \param setpoint - The set point position in radians
  /// \param actual - The actual motor shaft position in radians
  /// \param shaft_vel - The motor shaft velocity used in derivative control
  float pump_controller(float setpoint, float actual, float shaft_vel);

  /// \brief Set the status of feed forward control in the class
  /// \param enable - Sets if feed forward term is used in control
  void set_ffwd_control(bool enable);

  /// \brief Set the value of the integral control error clamp
  /// \param clamp_val - The value that the integral error is clamped at
  void set_i_clamp_val(float clamp_val);

  /// \brief Set the value of the command clamp
  /// \param clamp_val - The value that the command output is clamped at
  void set_u_clamp_val(float clamp_val);

private:
  /// \brief Proportional gain
  float Kp_;
  /// \brief Integral gain
  float Ki_;
  /// \brief Derivative gain
  float Kd_;
  /// \brief Feed forward gain
  float Kff_;
  /// \brief Boolean enabling integral control
  bool Icntrl_;
  /// \brief Boolean enabling derivative control
  bool Dcntrl_;
  /// \brief Error
  float err_;
  /// \brief Integrated error
  float err_int_;
  /// \brief Derivative error
  float err_der_;
  /// \brief Previous error
  float err_prev_;
  /// \brief Integral error clamping value
  float i_clamp_val_;
  /// \brief Command clamping value
  float u_clamp_val_;
  /// \brief Boolean enabling gravity compensation control
  bool gvty_fwd_enable_;
  /// \brief Gravity compensation term
  float gvty_term_;
};

#endif
