#include <Arduino.h>
#include <TeensyTimerTool.h>
#include <vector>
#include <math.h>
#include <functional>
#include "nu_control.hpp"
#include "pos_controller.hpp"
#include "step_detection.hpp"
#include "lsm6dsv.hpp"

// trajectory headers
// #include "steps.hpp"
// #include "swing_traj_11.hpp"
#include "swing_traj_segments.hpp"

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
PositionController p_controller_{13.0, 0.03, 0.35, 1.4};

// IMU and step detection initialization
LSM6DSV_IMU imu;
StepDetector step_detector(15, 1.336, 0.2, 1000);

// init global vars
auto imu_data = accelerations{0.0, 0.0, 0.0};
volatile auto cadence = 0.0;
volatile auto count = 0;
volatile auto target = 0.0;
volatile auto target_angle = 0.0;
volatile auto next_angle = 0.0;
volatile auto system_angle = 0.0;
volatile auto system_vel = 0.0;
auto offset = 1.5;

auto controller_enabler = false;

std::vector<float> * trajectory;
std::vector<float> trajectory_05;
std::vector<float> trajectory_07;
std::vector<float> trajectory_09;
std::vector<float> trajectory_11;
std::vector<float> trajectory_13;
std::vector<float> trajectory_15;
std::vector<float> zero_trajectory (50, 0);
std::vector<float> cadence_boundaries = {1.0, 1.3, 1.55, 1.7, 1.9, 2.1, 2.5};
std::vector<std::vector<float>*> trajectory_options = {&trajectory_05, &trajectory_07, &trajectory_09, &trajectory_11, &trajectory_13, &trajectory_15};

void update_trajectory() {
  for (auto i = 0; i < int(cadence_boundaries.size()); i++) {
    if ((cadence > cadence_boundaries.at(i)) && (cadence < cadence_boundaries.at(i+1))){
      controller_enabler = true;
      trajectory = trajectory_options.at(i);
      return;
    }
  }
  // if it doesnt fit into any buckets, make it zeros
  // trajectory = &zero_trajectory; 
  controller_enabler = false;
}

void current_control_loop() {
  controller_.update_sensors();
  controller_.update_control();
}

void command_update_loop()
{
  if (count > int(trajectory->size())-1)
  {
    count = 0;
    update_trajectory();
  }

  target_angle = trajectory->at(count); // make negative to reverse direction
  system_angle =  controller_.get_shaft_angle() - offset; 
  system_vel =  controller_.get_shaft_velocity();

  count++;
}

void position_control_loop()
{
  if (controller_enabler){
    target = p_controller_.pump_controller(target_angle, system_angle, system_vel);
  } else {
    target = 0.0;
  }

  controller_.set_target(target);
}

void imu_loop()
{ 
  imu_data = imu.get_acc();
  step_detector.filter_update(imu_data);
  cadence = step_detector.get_cadence();
}

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

  Serial.print(">cadence:");
  Serial.println(cadence);
}

void setup()
{
  while (!Serial) {}

  imu.init();

  std::copy(trajectory_traj_segment_traj_05_swing_47.begin(), trajectory_traj_segment_traj_05_swing_47.end(), std::back_inserter(trajectory_05)); 
  std::copy(trajectory_traj_segment_traj_07_swing_6.begin(), trajectory_traj_segment_traj_07_swing_6.end(), std::back_inserter(trajectory_07)); 
  std::copy(trajectory_traj_segment_traj_09_swing_1.begin(), trajectory_traj_segment_traj_09_swing_1.end(), std::back_inserter(trajectory_09)); 
  std::copy(trajectory_traj_segment_traj_11_swing_70.begin(), trajectory_traj_segment_traj_11_swing_70.end(), std::back_inserter(trajectory_11)); 
  std::copy(trajectory_traj_segment_traj_13_swing_27.begin(), trajectory_traj_segment_traj_13_swing_27.end(), std::back_inserter(trajectory_13)); 
  std::copy(trajectory_traj_segment_traj_15_swing_15.begin(), trajectory_traj_segment_traj_15_swing_15.end(), std::back_inserter(trajectory_15)); 

  trajectory = &zero_trajectory;

  p_controller_.set_ffwd_control(true);

  TeensyTimerTool::attachErrFunc(timer_errors);
  analogReadAveraging(1);

  Serial.println("Hell yeah!");

  if (!controller_.init_components()) {
    Serial.println("Motor controller component failed to init");
    exit(0);
  }

  Serial.println("Aligning");
  // controller_.set_calibration_scan_range(0.2617994); // 15 degrees
  // controller_.set_calibration_scan_speed(0.1);
  
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

  // begin timers, rate in us
  current_control_timer_.begin(current_control_loop, 100);
  command_update_timer_.begin(command_update_loop, 10000);
  position_control_timer_.begin(position_control_loop, 1000);
  imu_timer_.begin(imu_loop, 1000);
  print_timer_.begin(print_loop, 10000);
  
}

void loop() {}
