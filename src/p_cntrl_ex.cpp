#include <Arduino.h>
#include <TeensyTimerTool.h>
#include <vector>
#include <math.h>
#include "nu_control.hpp"
#include "pos_controller.hpp"
#include "sinusoids.hpp"
#include "steps.hpp"

TeensyTimerTool::PeriodicTimer timer_(TeensyTimerTool::TCK);
TeensyTimerTool::PeriodicTimer position_control_timer_(TeensyTimerTool::TCK);
TeensyTimerTool::PeriodicTimer print_timer_(TeensyTimerTool::TCK);

constexpr float CURR_GAIN = 3.3333333f; // Amps / Volt
constexpr int ADC_RES = 10;

InlineCurrentSensor Current_Phase_B{A3, CURR_GAIN, ADC_RES};
InlineCurrentSensor Current_Phase_C{A8, CURR_GAIN, ADC_RES};
InlineCurrentSensorPackage Current_Sensors{{&Current_Phase_C, &Current_Phase_B}};
constexpr float PWM_FREQ = 20000.f;
constexpr int PWM_RES = 12;
constexpr float DRIVER_VOLTAGE = 16.f;

BrushlessDriver GateDriver{{6, 5, 4}, 3, PWM_FREQ, PWM_RES, DRIVER_VOLTAGE};

const uint16_t EncoderReadCmd = (0b11 << 14) | 0xFFFF;
SPIEncoder Encoder{EncoderReadCmd, SPI, 10};

BrushlessController controller_{GL40, GateDriver, Current_Sensors, Encoder};

PositionController p_controller_{25.0, 1.0, 0.1}; // sinusoid 1hz
// PositionController p_controller_{17.0, 0.0, 0.0}; // step function

auto count = 0;

double target;
double target_angle = 0.0;
double system_angle = 0.0;
std::vector<float> trajectory;

bool splitter = false;



char msg[50];

void trajectory_control_loop()
{
  controller_.update_sensors();

  if (count >= int(trajectory.size()))
  {
    count = 0;
  }

  // pump position controller
  target_angle = trajectory.at(count);
  system_angle =  controller_.get_shaft_angle();
  target = p_controller_.pump_controller(target_angle, system_angle, controller_.get_shaft_velocity());
  controller_.set_target(target);

  controller_.update_control();

  // if (splitter){
  //   count++;
  //   splitter = false;
  // }
  // else
  // {
  //   splitter = true;
  // }

  count++;
}

void point_control_loop()
{

  controller_.update_sensors();

  system_angle =  controller_.get_shaft_angle();

  target = p_controller_.pump_controller(target_angle, system_angle, controller_.get_shaft_velocity());
  controller_.set_target(target);

  controller_.update_control();

  count++;
}

void print_loop()
{
  sprintf(msg,">setpoint: %f\n>system: %f\n>error:%f\n",target_angle,system_angle, float(target_angle-system_angle));
  Serial.println(msg);

}

void setup()
{
  // while (!Serial) {}

  TeensyTimerTool::attachErrFunc(timer_errors);
  analogReadAveraging(1);

  Serial.println("Hell yeah!");

  auto ret_init = controller_.init_components();
  if (!ret_init) {
    Serial.println("Motor controller component failed to init");
    exit(0);
  }
  Serial.println("Aligning");

  auto ret_align = controller_.align_sensors(0, false);
  if (!ret_align) {
    Serial.println("Motor controller component failed to align");
    exit(0);
  }

  Serial.println("Preparing to run");
  delay(1000);
  // controller_.set_feedforward_state(true);
  // controller_.set_feedback_state(false);
  // controller_.set_back_emf_comp_state(false);

  controller_.set_target(0.0f);

  controller_.start_control(100, false);

  std::copy(sinusoid_1Hz.begin(), sinusoid_1Hz.end(), std::back_inserter(trajectory));

  position_control_timer_.begin(
    [](){
      trajectory_control_loop();
    }, 100);

  print_timer_.begin(
    [](){
      print_loop();
    }, 500);



}

void loop() {}
