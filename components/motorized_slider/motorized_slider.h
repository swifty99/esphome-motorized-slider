#pragma once

#include "esphome/core/component.h"
#include "esphome/components/output/float_output.h"
#include "esphome/core/hal.h"
#include <vector>
#include "esphome/core/helpers.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/log.h"
#include <cmath>
#include <esp_adc/adc_oneshot.h>
#include <hal/adc_types.h>
#include "esp_timer.h"



static void s_timer_intr();

namespace esphome {
namespace motorized_slider {

// Klasse für den MotorController
class MotorController : public esphome::Component {
public:
    void setup() override;
    void loop() override;
    void set_pwm_forward(output::FloatOutput  *pwm_forward) { pwm_forward_ = pwm_forward; }
    void set_pwm_backward(output::FloatOutput  *pwm_backward) { pwm_backward_ = pwm_backward; }
  

    void set_target_position(float target_position) ;
    void add_rastpunkt(float position, float stiffness);
    void set_slider_position_sensor(sensor::Sensor *slider_position_sensor) { slider_position_sensor_ = slider_position_sensor; }
    void set_adc_channel(adc_unit_t unit, adc_channel_t channel) {
        this->adc_unit_ = unit;
        this->adc_channel_ = channel;
    }
    void on_timer(); // Timer callback function

    void set_dither_amplitude(float amplitude) { dither_amplitude_ = amplitude; }
    void set_dither_frequency(int frequency) { dither_frequency_ = frequency; }
    void set_dither_idle_ratio(float ratio) {
        if (ratio < 0.0f) ratio = 0.0f;
        if (ratio > 0.49f) ratio = 0.49f;
        dither_idle_ratio_ = ratio;
    }
    void set_static_friction_forward(float value) { friction_motor_forward_ = value; }
    void set_static_friction_backward(float value) { friction_motor_backward_ = value; }
    void set_friction_velocity_threshold(float value) { friction_velocity_threshold_ = value; }
    void set_friction_dynamic_gain(float value) { friction_dynamic_gain_ = value; }
    void set_friction_command_threshold(float value) { friction_command_threshold_ = value; }
    void set_friction_deadband(float value) {
        if (value < 0.0f) value = 0.0f;
        friction_deadband_ = value;
    }
    void set_override_position_threshold(float value) { override_position_threshold_ = value; }
    void set_override_velocity_threshold(float value) { override_velocity_threshold_ = value; }
    void set_override_debounce_time(uint32_t value) { override_debounce_time_ = value; }
    void set_override_detection(bool enabled) { override_detection_enabled_ = enabled; }
    void set_diagnostic_logging(bool enabled) { diagnostic_logging_enabled_ = enabled; }
    void set_process_noise(float value) {
        if (value < 0.0f) value = 0.0f;
        process_noise_ = value;
    }
    void set_pid_kp(float value) { pid_kp_ = value; }
    void set_pid_ki(float value) { pid_ki_ = value; }
    void set_pid_kd(float value) { pid_kd_ = value; }
    void set_rastpunkte_enabled(bool enabled) { rastpunkte_enabled_ = enabled; }

protected:
    output::FloatOutput  *pwm_forward_ = nullptr;
    output::FloatOutput  *pwm_backward_ = nullptr;
    sensor::Sensor *slider_position_sensor_ = nullptr;
    adc_unit_t adc_unit_{ADC_UNIT_1};
    adc_channel_t adc_channel_{ADC_CHANNEL_0};
    adc_atten_t adc_atten_{ADC_ATTEN_DB_12};
    adc_oneshot_unit_handle_t adc_handle_{nullptr};
    float target_position_ = 0.0f;
    float current_position_ = 0.0f;
    float current_velocity_ = 0.0f;
    float last_filtered_velocity = 0.0f;
    float kalman_estimate_ = 0.0f;
    float kalman_gain_ = 0.0f;
    bool kalman_initialized_ = false;
    
    float max_slider_speed_ = 50.0f; //value 50 go left to right in 2 seconds
    float max_slider_acceleration_ = 600.0f; //value 300 go left to right in 2 seconds
    
    float last_compensated_torque = 0.0f;
    bool motor_idle_ = true;

    // use inverse of the time constant of the velocity filter
    float velocity_pt1_time_const_ = 0.15 ;
    
    // Data structure for storing position and stiffness
    struct Rastpunkt {
        float position;
        float stiffness;
    };

    // Holds multiple Rastpunkt entries
    std::vector<Rastpunkt> rastpunkte_;

    // Torque and position/velocity methods
    void set_torque(float torque);
    float get_position(bool filtered = true, bool raw = false);
    float get_velocity();

private:
    // Core feedback variables
    float estimate_error_ = 1.0f;
    float next_position_ = 0.0f;
    int position100_raw_value_ = 100;
    int position0_raw_value_  = 8000;
    float max_torque_ = 0.4;
    const int timer_period_ = 1000;          // 1ms
    const float timer_period_sec_ = 1000000.0f / (float)timer_period_; // 1ms
    bool target_reached_ = false;

    // Kalman filter parameters
    float process_noise_ = 0.01f;
    float measurement_error_ = 0.02f;
    float slope_speed_ = 0.0f;
    uint32_t last_kalman_log_ = 0;

    // Friction data
    float friction_motor_forward_ = 0.424f;
    float friction_motor_backward_ = 0.4024f;
    float friction_velocity_threshold_ = 12.0f;
    float friction_dynamic_gain_ = 0.0005f;
    float friction_command_threshold_ = 0.02f;
    float friction_deadband_ = 2.0f;
    bool is_initialized_ = false;
    bool friction_determined_ = true;

    // Dither parameters
    float dither_amplitude_ = 0.0f;
    int dither_frequency_ = 50;
    float dither_idle_ratio_ = 0.0f;
    float calculate_dither();

    // Calculation methods
    void calculate_measurement_error();
    void set_force_feedback_torque();
    float force_feedback_strength_ = 0.50f;

    // Override detection
    float expected_position_ = 0.0f;
    bool manual_override_ = false;
    uint32_t last_override_time_ = 0;
    uint32_t override_debounce_time_ = 200;
    float override_position_threshold_ = 5.0f;
    float override_velocity_threshold_ = 70.0f;
    bool override_detection_enabled_ = true;
    bool diagnostic_logging_enabled_ = false;
    bool rastpunkte_enabled_ = true;

    float pid_kp_ = 0.05f;
    float pid_ki_ = 0.005f;
    float pid_kd_ = 0.0f;
    void check_manual_override();
    bool monitor_manual_movements();

    // Trajectory planning
    struct Trajectory {
        float t_total;
        float t1;
        float t2;
        float start_position;
        float target_position;
        float max_velocity;
        float max_acceleration;
    } trajectory_;

    uint32_t trajectory_start_time_ = 0;
    uint32_t iSR_runtime_ = 0;
    void plan_trajectory(float start_position, float target_position, 
                         float max_velocity, float max_acceleration);
    void initialize_motor();
    bool follow_trajectory();
    bool determine_friction();
    float calculate_rastpunkt_force(bool printlog = false);
    void move_to_position(float target_position);
    uint32_t duration_getsensor_ = 0;

    // System timer handle
    esp_timer_handle_t periodic_timer_;

};

}  // namespace motorized_slider
}  // namespace esphome

