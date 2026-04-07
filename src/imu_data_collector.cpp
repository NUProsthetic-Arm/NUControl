// #include "lsm6dsv.hpp"
// #include "step_detection.hpp"
// #include <TeensyTimerTool.h>

// TeensyTimerTool::PeriodicTimer imu_timer_(TeensyTimerTool::TCK);
// TeensyTimerTool::PeriodicTimer print_timer_(TeensyTimerTool::TCK);

// LSM6DSV_IMU imu;

// StepDetector step_detector(15, 1.336, 0.2, 1000);

// static auto imu_data = accelerations{0.0, 0.0, 0.0};
// static auto step = false;
// static auto cadence = 0.0;

// void imu_loop()
// {
//   imu_data = imu.get_acc();
//   step_detector.filter_update(imu_data);
//   cadence = step_detector.get_cadence();
// }

// void print_loop()
// {
//   Serial.print(">ax:");
//   Serial.println(imu_data.x, 3);

//   Serial.print(">ay:");
//   Serial.println(imu_data.y, 3);

//   Serial.print(">az:");
//   Serial.println(imu_data.z, 3);

//   Serial.print(">cadence:");
//   Serial.println(cadence);
// }
// void setup() 
// {
//   // init serial
//   Serial.begin(115200);

//   // init imu
//   imu.init();

//   // start imu loop
//   imu_timer_.begin(
//     [](){
//       imu_loop();
//     }, 1ms);

//   // start print loop
//   print_timer_.begin(
//     [](){
//       print_loop();
//     }, 10ms);
// }

// void loop() 
// {
// }