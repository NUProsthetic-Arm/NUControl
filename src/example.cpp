#include <Arduino.h>
#include <TeensyTimerTool.h>
#include <vector>
#include <math.h>
#include "nu_control.hpp"

TeensyTimerTool::PeriodicTimer timer_(TeensyTimerTool::TCK);

constexpr float CURR_GAIN = 5.f; // Amps / Volt
constexpr int ADC_RES = 10;

// InlineCurrentSensor Current_Phase_B{A2, CURR_GAIN, ADC_RES};
// InlineCurrentSensor Current_Phase_C{A3, CURR_GAIN, ADC_RES};
// InlineCurrentSensor Current_Phase_B{A4, CURR_GAIN, ADC_RES};
// InlineCurrentSensor Current_Phase_C{A5, CURR_GAIN, ADC_RES};
InlineCurrentSensor Current_Phase_B{A6, CURR_GAIN, ADC_RES};
InlineCurrentSensor Current_Phase_C{A7, CURR_GAIN, ADC_RES};
// InlineCurrentSensor Current_Phase_B{A9, CURR_GAIN, ADC_RES};
// InlineCurrentSensor Current_Phase_C{A8, CURR_GAIN, ADC_RES};
InlineCurrentSensorPackage Current_Sensors{{&Current_Phase_C, &Current_Phase_B}};

constexpr float PWM_FREQ = 20000.f;
constexpr int PWM_RES = 12;
constexpr float DRIVER_VOLTAGE = 24.f;

const uint16_t EncoderReadCmd = (0b11 << 14) | 0x3FFF;
SPIEncoder Encoder{EncoderReadCmd, SPI, 10};
// SPIEncoder Encoder{EncoderReadCmd, SPI1, 0};
// SPIEncoder Encoder{EncoderReadCmd, SPI2, 36};

// BrushlessDriver GateDriver{{33, 29, 39}, 38, PWM_FREQ, PWM_RES, DRIVER_VOLTAGE};
BrushlessDriver GateDriver{{2, 3, 4}, 5, PWM_FREQ, PWM_RES, DRIVER_VOLTAGE};
// BrushlessDriver GateDriver{{9, 8, 7}, 6, PWM_FREQ, PWM_RES, DRIVER_VOLTAGE};
// 
MotorParameters U2523{7, 0.72487f, 510.f * 1e-6f, 3.f, 8.f, 0.025f, 0.001f};

BrushlessController controller_{U2535, GateDriver, Current_Sensors, Encoder};
// BrushlessController controller_{EC45_Flat, GateDriver, Current_Sensors, Encoder};

CoggingMapper<100> mapper_(controller_);

// constexpr std::array<float, 100> cogging_map = {
//   #include "anticogging_map.csv"
// };

constexpr std::array<float, 100> va= {
  #include "anticogging_map_va.csv"
};

constexpr std::array<float, 100> vb = {
  #include "anticogging_map_vb.csv"
};

constexpr std::array<float, 100> vc = {
  #include "anticogging_map_vc.csv"
};

const PhaseValues<std::array<float, 100>>volt_map{va, vb, vc};


AnticoggingCompensator<100> anticog_{volt_map};

PhaseValues<float> torque_cog(float rads){
  return anticog_.get_cogging_voltage(rads);
}


size_t idx = 0;
size_t cntr = 0;
std::array<float, 10> targets_{0.f, 0.6f, 1.2f, 1.8f, 2.4f, 3.0f, 3.6f, 4.2f, 4.8f, 5.4f};
float kp = 0.2f;

void update(){
  controller_.update_sensors();
  controller_.update_control();
  // auto torque = kp * normalize_angle(targets_[idx] - controller_.get_shaft_angle());
  // torque = std::clamp(torque, -0.3f, 0.3f);

  // controller_.set_target(torque);
  // controller_.update_control();
  // if(cntr >= 10000){
  //   idx++;
  //   cntr = 0;
  //   if(idx == 10) {
  //     idx = 0;
  //   }
  // }
  // cntr++;

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
  controller_.set_position_filter({{0.25f, 0.25f, 0.25f, 0.25f}, {}});
  controller_.set_velocity_filter(vel_filter_200_);
  controller_.set_target(0.0f);
  controller_.set_feedback_state(false);
  // controller_.enable_anticog(std::function<PhaseValues<float>(float)>(torque_cog));

  // mapper_.map_cogging(2);

  controller_.start_control(100,false);
  timer_.begin(update, 100);

}

void loop() {
  // Serial.println(Encoder.read(), 6);
  // delay(10);
}
