#include <Arduino.h>
#include <TeensyTimerTool.h>
#include <vector>
#include <math.h>
#include "nu_control.hpp"

TeensyTimerTool::PeriodicTimer timer_(TeensyTimerTool::TCK);

constexpr float CURR_GAIN = 5.f; // Amps / Volt
constexpr int ADC_RES = 10;

InlineCurrentSensor Current_Phase_B{A2, CURR_GAIN, ADC_RES};
InlineCurrentSensor Current_Phase_C{A3, CURR_GAIN, ADC_RES};
// InlineCurrentSensor Current_Phase_B{A9, CURR_GAIN, ADC_RES};
// InlineCurrentSensor Current_Phase_C{A8, CURR_GAIN, ADC_RES};
InlineCurrentSensorPackage Current_Sensors{{&Current_Phase_C, &Current_Phase_B}};

constexpr float PWM_FREQ = 20000.f;
constexpr int PWM_RES = 12;
constexpr float DRIVER_VOLTAGE = 24.f;

const uint16_t EncoderReadCmd = (0b11 << 14) | 0x3FFF;
SPIEncoder Encoder{EncoderReadCmd, SPI2, 36};
// SPIEncoder Encoder{EncoderReadCmd, SPI1, 0};

BrushlessDriver GateDriver{{33, 29, 39}, 38, PWM_FREQ, PWM_RES, DRIVER_VOLTAGE};
// BrushlessDriver GateDriver{{3, 4, 5}, 2, PWM_FREQ, PWM_RES, DRIVER_VOLTAGE};

MotorParameters U2523{7, 0.72487f, 510.f * 1e-6f, 3.f, 8.f, 0.025f, 0.001f};

BrushlessController controller_{U2523, GateDriver, Current_Sensors, Encoder};
// BrushlessController controller_{EC45_Flat, GateDriver, Current_Sensors, Encoder};

// CoggingMapper<1000> mapper_(controller_);

constexpr std::array<float, 1000> cogging_map = {
  #include "anticogging_map.csv"
};

AnticoggingCompensator<1000> anticog_{cogging_map};

float torque_cog(float rads){
  return anticog_.get_cogging_torque(rads);
}

void update(){
  controller_.update_sensors();
  controller_.update_control();
}

void setup()
{
  while (!Serial) {}


  TeensyTimerTool::attachErrFunc(timer_errors);
  analogReadAveraging(1);

  Serial.println("Hell yeah!");

  if (!controller_.init_components()) {
    Serial.println("Motor controller component failed to init");
    exit(0);
  }

  Serial.println("Aligning");

  if (!controller_.align_sensors()) {
    Serial.println("Motor controller component failed to align");
    exit(0);
  }

  Serial.println("Preparing to run");
  controller_.print_calibration();
  delay(1000);

  controller_.set_control_mode(ControllerMode::TORQUE);
  controller_.set_velocity_filter(vel_filter_200_);
  controller_.set_target(0.0f);
  controller_.enable_anticog(std::function<float(float)>(torque_cog));

  // mapper_.map_cogging(2);

  // controller_.start_control(100,false);
  timer_.begin(update, 100);

}

void loop() {}
