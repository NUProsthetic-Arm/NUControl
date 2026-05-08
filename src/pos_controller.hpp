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
  /// \param Kff_gravity - The gravity feed forward gain
  /// \param Kff_spring - The spring feed forward gain
  PositionController(float Kp, float Ki, float Kd, float Kff_gravity, float Kff_spring);

  /// \brief Pump controller
  /// \param setpoint - The set point position in radians
  /// \param actual - The actual motor shaft position in radians
  /// \param shaft_vel - The motor shaft velocity used in derivative control
  float pump_controller(float setpoint, float actual, float shaft_vel);

  /// \brief Set the status of gravtiy compensation feed forward control in the class
  /// \param enable - Sets if feed forward term is used in control
  void set_gvty_ffwd_control(bool enable);

  /// \brief Set the status of spring feed forward control in the class
  /// \param enable - Sets if feed forward term is used in control
  void set_spring_ffwd_control(bool enable);

  /// \brief Set the value of the integral control error clamp
  /// \param clamp_val - The value that the integral error is clamped at
  void set_i_clamp_val(float clamp_val);

  /// \brief Set the value of the command clamp
  /// \param clamp_val - The value that the command output is clamped at
  void set_u_clamp_val(float clamp_val);

  /// \brief Set the value of the theta resting position
  /// \param theta_rest - The value that the system is at rest at
  void set_theta_rest_val(float theta_rest);


private:
  /// \brief Proportional gain
  float Kp_;
  /// \brief Integral gain
  float Ki_;
  /// \brief Derivative gain
  float Kd_;
  /// \brief Gravity feed forward gain
  float Kff_gvty_;
  /// \brief Spring feed forward gain
  float Kff_spring_;
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
  /// \brief Boolean enabling spring compensation control
  bool spring_fwd_enable_;
  /// \brief Spring compensation term
  float spring_term_;
    /// \brief Resting position from spring
  float theta_rest_;
  /// \brief Delta time
  float dt_;
  float err_der_raw_;
  float err_der_filt_;
  float alpha_d_;  // set in constructor, e.g. 0.1f for heavy filtering

};

#endif
