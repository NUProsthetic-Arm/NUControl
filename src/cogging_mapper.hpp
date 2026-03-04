#include "brushless_controller.hpp"
#include <array>


template<std::size_t steps_>
class CoggingMapper
{
    public:
        CoggingMapper() = default;
        ~CoggingMapper() = default;

        CoggingMapper(BrushlessController & controller)
        : controller_(controller),
          timer_(TeensyTimerTool::TCK)
        {
            controller.disable_anticog();
        };

        void map_cogging(int loops)
        {
            max_loop = loops;
            looper = 0;
            idx = 0;
            target_selector(dir);
            auto motor = controller_.get_motor();
            max_torque = 0.5f * motor.kT * motor.SAFE_CURRENT;
            controller_.start_control(100, false);
            timer_.begin([this] {cogging_mapper();}, 100);
        }

    private:

        BrushlessController & controller_;
        TeensyTimerTool::PeriodicTimer timer_;

        std::array<float, steps_> positions_{};
        std::array<float, steps_> torques_{};
        std::array<PhaseValues<float>,steps_> phase_volts_{};

        float pos_target = 0.f;
        size_t idx = 0;

        // Controller parameters

        float kP = 1.f;
        float kD = 0.1f;
        float kI = 0.5f;
        float intl = 0.f;

        float max_intl = 0.1f;
        float max_torque = 0.5f;

        int dir =  1;
        int max_loop = 10;
        int looper = 0;

        const float dt_ = 100.f * 1e-6f;

        size_t clk_start = 0;
        float pos_err_tol = 0.01f;
        bool locked = false;
        constexpr static size_t torque_steps_ = 1000; // Control Frequency * torque_steps_ = settling time = 0.1s by default
        constexpr static float step_inv = 1.f / static_cast<float>(torque_steps_);
        float torque_sum_ = 0.f;
        PhaseValues<float> volt_sum_{0.f, 0.f, 0.f};

        void target_selector(int direction = 1)
        {
            float gain = static_cast<float>(direction) * _2_PI_;
            float offset = 0.f;
            float shift = 0.f;

            if (direction == -1) {
                offset = _2_PI_;
                shift = 1.f;
            }

            for (size_t i = 0; i < steps_; ++i) {
                positions_.at(i) = offset + gain * (i + shift) / static_cast<float>(steps_);
                Serial.println(positions_.at(i));
            }

        }


        void cogging_mapper()
        {
            controller_.update_sensors();

            float error = normalize_angle(positions_.at(idx) - controller_.get_shaft_angle());
            intl += (error * dt_);
            intl = std::clamp(intl, -max_intl, max_intl);
            float torque = kP * error - kD * controller_.get_shaft_velocity() + kI * intl;
            torque = std::clamp(torque, -max_torque, max_torque);
            controller_.set_target(torque);
            controller_.update_control();

        if (fabs(error) < pos_err_tol && !locked) {
            locked = true;
        }
        if (locked) {
            if (fabs(error) > pos_err_tol) {
                locked = false;
                volt_sum_ = 0.f;
                torque_sum_ = 0.f;
                clk_start = 0;
            }
            torque_sum_ += torque * step_inv;
            volt_sum_ += controller_.get_last_phasevolts() * step_inv;
            clk_start++;
            if (clk_start >= torque_steps_) {
                torques_.at(idx) = torque_sum_;
                phase_volts_.at(idx) = volt_sum_;
                volt_sum_ = 0.f;
                torque_sum_ = 0.f;
                locked = false;
                clk_start = 0;
                intl = 0.f;
                idx++;
                Serial.print("Target #");
                Serial.println(idx);
            }
            if (idx >= steps_) {
                report_out();
            }
        }
        }

        void report_out()
        {
        controller_.stop_control();
        timer_.stop();
        Serial.println("=====");

        for (size_t j = 0; j < steps_; ++j) {
            Serial.print(positions_.at(j), 6);
            Serial.print("\t");
            Serial.print(torques_.at(j), 6);
            Serial.print("\t");
            Serial.print(phase_volts_.at(j).a, 6);
            Serial.print("\t");
            Serial.print(phase_volts_.at(j).b, 6);
            Serial.print("\t");
            Serial.println(phase_volts_.at(j).c, 6);
            Serial.flush();
            delay(1);
        }
        delay(1);
        Serial.println("=====");
        Serial.flush();
        delay(10);
        Serial.print("Loop #");
        Serial.print(looper);
        Serial.println(" finished!");

        looper++;

        if (looper < max_loop) {
            idx = 0;
            controller_.start_control(100, false);
            timer_.begin([this] {cogging_mapper();}, 100);
            return;
        } 

        if (dir == 1)
        {
            dir = -1;
            looper = 0;
            target_selector(dir);
            idx = 0;
            controller_.start_control(100, false);
            timer_.begin([this] {cogging_mapper();}, 100);
            return;
        }

        Serial.println("Finished! Exiting");
        Serial.flush();
        delay(10);
        exit(1);

        }




};