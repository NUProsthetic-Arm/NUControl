#include <Arduino.h>
#include <TeensyTimerTool.h>
#include <vector>
#include <math.h>
#include <functional>
#include "nu_control.hpp"
#include "pos_controller.hpp"
#include "heel_strike_filtering.hpp"
#include "lsm6dsv.hpp"

// trajectory headers
// #include "steps.hpp"
// #include "swing_traj_11.hpp"
#include "swing_lut.hpp"

#define PROSTHESIS_SIDE Classification::LEFT

TeensyTimerTool::PeriodicTimer current_control_timer_(TeensyTimerTool::TCK);
TeensyTimerTool::PeriodicTimer command_update_timer_(TeensyTimerTool::TCK);
TeensyTimerTool::PeriodicTimer position_control_timer_(TeensyTimerTool::TCK);
TeensyTimerTool::PeriodicTimer imu_timer_(TeensyTimerTool::TCK);
TeensyTimerTool::PeriodicTimer print_timer_(TeensyTimerTool::TCK);

constexpr float CURR_GAIN = 3.3333333f; // Amps / Volt
constexpr int ADC_RES = 10;

InlineCurrentSensor Current_Phase_B{A3, CURR_GAIN, ADC_RES};
InlineCurrentSensor Current_Phase_C{A8, CURR_GAIN, ADC_RES};

InlineCurrentSensorPackage Current_Sensors{{&Current_Phase_C, &Current_Phase_B}};

constexpr float PWM_FREQ = 20000.f;
constexpr int PWM_RES = 12;
constexpr float DRIVER_VOLTAGE = 16.f;

const uint16_t EncoderReadCmd = (0b11 << 14) | 0xFFFF;
SPIEncoder Encoder{EncoderReadCmd, SPI, 10};

BrushlessDriver GateDriver{{6, 5, 4}, 3, PWM_FREQ, PWM_RES, DRIVER_VOLTAGE};

BrushlessController controller_{GL40, GateDriver, Current_Sensors, Encoder};

// PositionController p_controller_{0.0, 0.0, 0.0, 1.0};
// PositionController p_controller_{10.0, 0.01, 0.35, 1.4};
PositionController p_controller_{13.0f, 0.03f, 0.35f, 1.4f};

// IMU and step detection initialization
LSM6DSV_IMU imu;
HeelStrikeFilter heel_strike_filter(15, 1.336f, 0.2f, 1000);

// init global vars
auto imu_data = accelerations{0.0f, 0.0f, 0.0f};
auto heel_strike_result = HeelStrikeResult{0, Classification::UNKNOWN, 0.0f};
auto prev_heel_strike_result = HeelStrikeResult{0, Classification::UNKNOWN, 0.0f};
auto ideal_side_state = Classification::UNKNOWN;
auto rt_side_state = Classification::UNKNOWN;
volatile auto count = 0;
volatile auto target = 0.0f;
volatile auto target_angle = 0.0f;
volatile auto next_angle = 0.0f;
volatile auto system_angle = 0.0f;
volatile auto system_vel = 0.0f;
volatile auto alpha = 1.0f;
auto offset = 1.5f;
SwingEntry trajectory;
SwingEntry prev_trajectory;
auto controller_enabler = false;

void system_state_update() {
  // check if controller should be enabled
  if (!controller_enabler && heel_strike_result.confidence == 5) {
    // if control is disabled, and confidence reaches 5 turn it on!
    controller_enabler = true;

    // save this expectation internally, this will be the running assumption
    ideal_side_state = heel_strike_result.classification;
    rt_side_state = ideal_side_state;
    // start trajectory from beginning
    count = 0;
  }
  else if (controller_enabler && heel_strike_result.confidence == 0) {
    // if control is enabled, and confidence reaches 0 turn it off!
    controller_enabler = false;
  }  
  
  // check if there has been a heel strike
  if (heel_strike_result.count != prev_heel_strike_result.count) {
    // update prev_trajectory for smoothing
    prev_trajectory = trajectory;

    // load in new trajectory
    trajectory = swing_lut_lookup(heel_strike_result.period + prev_heel_strike_result.period);
  
    // update prev_period to check for new heel strike
    prev_heel_strike_result = heel_strike_result;

    // update the trajectory in use
    if (controller_enabler) {
      // if heel strike occured before trajectory finished, flip both to match
      if (ideal_side_state == rt_side_state) {
        if (PROSTHESIS_SIDE == ideal_side_state) {
          // if heel strike was on same side as prosthesis, use fwd trajectory
          rt_side_state = PROSTHESIS_SIDE;
        } else {
          // if opposite, use bwd trajectory
          rt_side_state = flip_expectation(PROSTHESIS_SIDE);
        }
      }      
      // flip ideal expectation in both cases
      ideal_side_state = flip_expectation(ideal_side_state);

      // reset blend term to deal with changed trajectories
      alpha = 0.1;
    }
  }
}

// NUControl update loop
void current_control_loop() {
  controller_.update_sensors();
  controller_.update_control();
}

// Update the commanded position loop
void command_update_loop()
{
  // check if control is enabled 
  if (controller_enabler) {
    // use fwd if classification state is same side, and vice versa
    if (rt_side_state == PROSTHESIS_SIDE) {
      // check if trajectory has been completed, if so proceed to back half
      if (count >= int(trajectory.fwd_len)) {
        // flip rt state
        rt_side_state = flip_expectation(rt_side_state);

        // reset count
        count = 0;

        // set target angle from bwd
        target_angle = (1 - alpha) * prev_trajectory.bck[count] + alpha * trajectory.bck[count];
      } else if (count < int(prev_trajectory.fwd_len)) {
        target_angle = (1 - alpha) * prev_trajectory.fwd[count] + alpha * trajectory.fwd[count];
      } else {
        target_angle =  trajectory.fwd[count];
      }
    } else {
      // check if trajectory has been completed, if so proceed to front half
      if (count >= int(trajectory.bck_len)) {
        // flip rt state
        rt_side_state = flip_expectation(rt_side_state);

        // reset count
        count = 0;

        // set target angle from fwd
        target_angle = (1 - alpha) * prev_trajectory.fwd[count] + alpha * trajectory.fwd[count];
      } else if (count < int(prev_trajectory.bck_len)) {
        target_angle = (1 - alpha) * prev_trajectory.bck[count] + alpha * trajectory.bck[count];
      } else {
        target_angle =  trajectory.bck[count];
      }
    }
    // increment
    count++;

    // handle alpha
    if (alpha < 1.0f) {
      // increment alpha such that it reaches 1.0 a quarter way through the trajectory
      alpha += 0.25f / (trajectory.fwd_len + trajectory.bck_len);  
    }
  }
  // apply offset and get shaft angle and velocity
  system_angle =  controller_.get_shaft_angle() - offset; 
  system_vel =  controller_.get_shaft_velocity();

}

// position controller loop
void position_control_loop()
{
  // check if control is enabled
  if (controller_enabler){
    // pump position PID controller
    target = p_controller_.pump_controller(target_angle, system_angle, system_vel);
  } else {
    // set target torque as 0.0 if controller is off
    target = 0.0f;
  }

  // send command to NUControl
  controller_.set_target(target);
}

// imu and step detection loop
void imu_loop()
{ 
  // collect imu data
  imu_data = imu.get_acc();

  // update heel strike filter
  heel_strike_filter.filter_update(imu_data);

  // get most recent results
  heel_strike_result = heel_strike_filter.get_result();
}

// printing loop
void print_loop()
{
  Serial.print(">setpoint:");
  Serial.println(target_angle, 3);

  Serial.print(">system_pos:");
  Serial.println(system_angle, 3);

  Serial.print(">system_vel:");
  Serial.println(system_vel, 3);

  Serial.print(">error:");
  Serial.println(float(target_angle-system_angle), 3);

  Serial.print(">i_target:");
  Serial.println(target, 3);

  Serial.print(">ax:");
  Serial.println(imu_data.x, 3);

  Serial.print(">ay:");
  Serial.println(imu_data.y, 3);

  Serial.print(">az:");
  Serial.println(imu_data.z, 3);

  Serial.print(">period:");
  Serial.println(heel_strike_result.period);

  Serial.print(">classification:");
  Serial.println(static_cast<int>(heel_strike_result.classification));
}

void setup()
{
  while (!Serial) {}

  imu.init();

  p_controller_.set_ffwd_control(true);

  TeensyTimerTool::attachErrFunc(timer_errors);
  analogReadAveraging(1);

  Serial.println("Hell yeah!");

  if (!controller_.init_components()) {
    Serial.println("Motor controller component failed to init");
    exit(0);
  }

  Serial.println("Aligning");
  controller_.set_calibration_scan_range(0.5); // 15 degrees
  controller_.set_calibration_scan_speed(0.1);

  // auto calib = BrushlessCalibration{};
  // controller_.load_calibration();
  
  if (!controller_.align_sensors()) {
    Serial.println("Motor controller component failed to align");
    exit(0);
  }

  Serial.println("Preparing to run");
  controller_.print_calibration();
  delay(1000);

  controller_.set_control_mode(ControllerMode::TORQUE);
  controller_.set_position_filter({{0.25f, 0.25f, 0.25f, 0.25f}, {}});
  controller_.set_velocity_filter(vel_filter_200_);
  controller_.set_feedforward_state(true);
  controller_.set_feedback_state(false);
  controller_.set_back_emf_comp_state(false);

  controller_.set_target(0.0f);

  controller_.start_control(100, false);

  // begin timers, rate in micro-seconds
  current_control_timer_.begin(current_control_loop, 100);
  command_update_timer_.begin(command_update_loop, 10000);
  position_control_timer_.begin(position_control_loop, 1000);
  imu_timer_.begin(imu_loop, 1000);
  print_timer_.begin(print_loop, 10000);
  
}

void loop() {}
