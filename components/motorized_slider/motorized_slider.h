#pragma once

#include "esphome/core/component.h"
#include "esphome/components/output/float_output.h"
#include "esphome/core/hal.h"
#include "esphome/components/voltage_sampler/voltage_sampler.h"
#include <vector>
#include "esphome/core/helpers.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/log.h"
#include <cmath>
#include "driver/adc.h"
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
  

    void set_feedback_sensor_pin(int feedback_sensor_pin) { feedback_sensor_pin_ = feedback_sensor_pin; }
    void set_target_position(float target_position) ;
    void add_rastpunkt(float position, float stiffness);
    void set_slider_position_sensor(sensor::Sensor *slider_position_sensor) { slider_position_sensor_ = slider_position_sensor; }
    void set_adcchannel(adc1_channel_t channel) {
        this->adc_channel_ = channel;
      }
    void on_timer(); // Timer callback function

protected:
    output::FloatOutput  *pwm_forward_ = nullptr;
    output::FloatOutput  *pwm_backward_ = nullptr;
    sensor::Sensor *slider_position_sensor_ = nullptr;


    adc1_channel_t adc_channel_{ADC1_CHANNEL_MAX};
    int feedback_sensor_pin_ = 30;
    float target_position_ = 0.0f;
    float current_position_ = 0.0f;
    float current_velocity_ = 0.0f;
    float last_filtered_velocity = 0.0f;
    
    float max_slider_speed_ = 50.0f; //value 50 go left to right in 2 seconds
    float max_slider_acceleration_ = 600.0f; //value 300 go left to right in 2 seconds
    
    float last_compensated_torque = 0.0f;
    bool motor_idle_ = true;

    // friction management
    float friction_speed_threshold_ = 12.0f;
    float friction_reduction_ramp_ = 20.0f; // force per second

    float friction_initial_ramp_up_ = 60.0f; // force per second

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

    // Friction data
    float friction_motor_forward_ = 0.424f;
    float friction_motor_backward_ = 0.4024f;
    bool is_initialized_ = false;
    bool friction_determined_ = false;

    // Calculation methods
    void calculate_measurement_error();
    float calculate_friction();
    void set_force_feedback_torque();
    float force_feedback_strength_ = 0.50f;

    // Override detection
    float expected_position_ = 0.0f;
    bool manual_override_ = false;
    uint32_t last_override_time_ = 0;
    const uint32_t override_debounce_time_ = 200;
    const float override_position_threshold_ = 5.0f;
    const float override_velocity_threshold_ = 70.0f;
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

