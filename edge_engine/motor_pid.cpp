// edge_engine/motor_pid.cpp
#include <pybind11/pybind11.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <chrono>

namespace py = pybind11;

class L298NPIDController {
private:
    int in1_pin, in2_pin, in3_pin, in4_pin, ena_pin, enb_pin;
    double kp, ki, kd;
    double prev_error_left, integral_left;
    double prev_error_right, integral_right;
    bool sim_mode;

    void write_sysfs(int pin, int value) {
        if (sim_mode) return;
        std::ofstream fs("/sys/class/gpio/gpio" + std::to_string(pin) + "/value");
        if (fs.is_open()) {
            fs << value;
            fs.close();
        }
    }

public:
    L298NPIDController(int in1, int in2, int in3, int in4, int ena, int enb,
                       double p, double i, double d, bool simulation = false)
        : in1_pin(in1), in2_pin(in2), in3_pin(in3), in4_pin(in4),
          ena_pin(ena), enb_pin(enb), kp(p), ki(i), kd(d),
          prev_error_left(0.0), integral_left(0.0),
          prev_error_right(0.0), integral_right(0.0),
          sim_mode(simulation) {}

    double compute_pid(double setpoint, double measured, double dt, bool is_left) {
        if (dt <= 0.0) dt = 0.05;
        double error = setpoint - measured;
        if (is_left) {
            integral_left = std::clamp(integral_left + error * dt, -100.0, 100.0);
            double derivative = (error - prev_error_left) / dt;
            prev_error_left = error;
            return std::clamp((kp * error) + (ki * integral_left) + (kd * derivative), -255.0, 255.0);
        } else {
            integral_right = std::clamp(integral_right + error * dt, -100.0, 100.0);
            double derivative = (error - prev_error_right) / dt;
            prev_error_right = error;
            return std::clamp((kp * error) + (ki * integral_right) + (kd * derivative), -255.0, 255.0);
        }
    }

    py::dict apply_kinematics(double linear_vel, double angular_vel, double wheel_base,
                              double measured_l, double measured_r, double dt) {
        // Differential Drive Inverse Kinematics
        double target_l = linear_vel - (angular_vel * wheel_base / 2.0);
        double target_r = linear_vel + (angular_vel * wheel_base / 2.0);

        double pwm_l = compute_pid(target_l, measured_l, dt, true);
        double pwm_r = compute_pid(target_r, measured_r, dt, false);

        // Set Left Motor Bank Direction (IN1 / IN2)
        write_sysfs(in1_pin, pwm_l >= 0 ? 1 : 0);
        write_sysfs(in2_pin, pwm_l >= 0 ? 0 : 1);

        // Set Right Motor Bank Direction (IN3 / IN4)
        write_sysfs(in3_pin, pwm_r >= 0 ? 1 : 0);
        write_sysfs(in4_pin, pwm_r >= 0 ? 0 : 1);

        py::dict telemetry;
        telemetry["target_l"] = target_l;
        telemetry["target_r"] = target_r;
        telemetry["pwm_l"] = std::abs(pwm_l);
        telemetry["pwm_r"] = std::abs(pwm_r);
        telemetry["dir_l"] = pwm_l >= 0 ? "FWD" : "REV";
        telemetry["dir_r"] = pwm_r >= 0 ? "FWD" : "REV";
        return telemetry;
    }

    void emergency_stop() {
        write_sysfs(in1_pin, 0);
        write_sysfs(in2_pin, 0);
        write_sysfs(in3_pin, 0);
        write_sysfs(in4_pin, 0);
        integral_left = 0.0;
        integral_right = 0.0;
    }
};

PYBIND11_MODULE(motor_pid, m) {
    m.doc() = "C++17 L298N Hardware GPIO & PID Controller for 4WD Rover";
    py::class_<L298NPIDController>(m, "L298NPIDController")
        .def(py::init<int, int, int, int, int, int, double, double, double, bool>())
        .def("compute_pid", &L298NPIDController::compute_pid)
        .def("apply_kinematics", &L298NPIDController::apply_kinematics)
        .def("emergency_stop", &L298NPIDController::emergency_stop);
}