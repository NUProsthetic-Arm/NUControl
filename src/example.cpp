// #include <Arduino.h>
// #include <TeensyTimerTool.h>
// #include <vector>
// #include <math.h>
// #include "nu_control.hpp"

// TeensyTimerTool::PeriodicTimer timer_(TeensyTimerTool::TCK);

// constexpr float CURR_GAIN = 3.3333333f; // Amps / Volt
// constexpr int ADC_RES = 10;

// InlineCurrentSensor Current_Phase_B{A3, CURR_GAIN, ADC_RES};
// InlineCurrentSensor Current_Phase_C{A8, CURR_GAIN, ADC_RES};

// InlineCurrentSensorPackage Current_Sensors{{&Current_Phase_C, &Current_Phase_B}};

// constexpr float PWM_FREQ = 20000.f;
// constexpr int PWM_RES = 12;
// constexpr float DRIVER_VOLTAGE = 16.f;

// const uint16_t EncoderReadCmd = (0b11 << 14) | 0xFFFF;
// SPIEncoder Encoder{EncoderReadCmd, SPI, 10};

// BrushlessDriver GateDriver{{6, 5, 4}, 3, PWM_FREQ, PWM_RES, DRIVER_VOLTAGE};

// BrushlessController controller_{GL40, GateDriver, Current_Sensors, Encoder};

// void update(){
//   controller_.update_sensors();
//   controller_.update_control();
// }

// void setup()
// {
//   while (!Serial) {}

//   TeensyTimerTool::attachErrFunc(timer_errors);
//   analogReadAveraging(1);

//   Serial.println("Hell yeah!");

//   if (!controller_.init_components()) {
//     Serial.println("Motor controller component failed to init");
//     exit(0);
//   }

//   Serial.println("Aligning");
//   controller_.set_calibration_scan_range(0.3490659); // 45 degrees
  
//   if (!controller_.align_sensors()) {
//     Serial.println("Motor controller component failed to align");
//     exit(0);
//   }

//   Serial.println("Preparing to run");
//   controller_.print_calibration();
//   delay(1000);

//   controller_.set_control_mode(ControllerMode::TORQUE);
//   controller_.set_position_filter({{0.25f, 0.25f, 0.25f, 0.25f}, {}});
//   controller_.set_velocity_filter(vel_filter_200_);
//   controller_.set_feedforward_state(true);
//   controller_.set_feedback_state(false);
//   controller_.set_back_emf_comp_state(false);

//   controller_.set_target(0.1f);

//   controller_.start_control(100,false);

//   timer_.begin(update, 100);

// }

// void loop() {}
