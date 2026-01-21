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

// BrushlessDriver GateDriver{{6, 5, 4}, 3, PWM_FREQ, PWM_RES, DRIVER_VOLTAGE};

// const uint16_t EncoderReadCmd = (0b11 << 14) | 0xFFFF;
// SPIEncoder Encoder{EncoderReadCmd, SPI, 10};

// BrushlessController controller_{GL40, GateDriver, Current_Sensors, Encoder};

// void setup()
// {
//   while (!Serial) {}

//   TeensyTimerTool::attachErrFunc(timer_errors);
//   analogReadAveraging(1);

//   Serial.println("Hell yeah!");

//   auto ret_init = controller_.init_components();
//   if (!ret_init) {
//     Serial.println("Motor controller component failed to init");
//     exit(0);
//   }
//   Serial.println("Aligning");

//   auto ret_align = controller_.align_sensors(true, true);
//   if (!ret_align) {
//     Serial.println("Motor controller component failed to align");
//     exit(0);
//   }

//   // controller_.set_e_angle_offset(0.97f);  // WHY IS THIS LINE HERE?

//   Serial.println("Preparing to run");
//   delay(1000);

//   controller_.set_control_mode(ControllerMode::TORQUE);

//   controller_.set_target(0.1f);

//   controller_.start_control(100);

// }

// void loop() {}
