#include "motorized_slider.h"
#include "esphome/core/log.h"

#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/hal.h"

#include "esp_timer.h"

namespace esphome {
namespace motorized_slider {

static const char *TAG = "motorized_slider";

static void periodic_timer_callback(void* arg) {
    MotorController* motor_controller = static_cast<MotorController*>(arg);
    if (motor_controller != nullptr) {
        motor_controller->on_timer();
    }
}


void MotorController::setup() {
    ESP_LOGCONFIG(TAG, "Setting up MotorController...");

    adc_oneshot_unit_init_cfg_t unit_config = {};
    unit_config.unit_id = this->adc_unit_;
    unit_config.ulp_mode = ADC_ULP_MODE_DISABLE;
#if USE_ESP32_VARIANT_ESP32C3 || USE_ESP32_VARIANT_ESP32C5 || USE_ESP32_VARIANT_ESP32C6 || \
    USE_ESP32_VARIANT_ESP32C61 || USE_ESP32_VARIANT_ESP32H2
    unit_config.clk_src = ADC_DIGI_CLK_SRC_DEFAULT;
#endif

    esp_err_t err = adc_oneshot_new_unit(&unit_config, &this->adc_handle_);
    if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "ADC unit %d already in use. Remove other ADC components or share the handle.", this->adc_unit_);
        this->status_set_error();
        return;
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADC unit %d: %d", this->adc_unit_, err);
        this->status_set_error();
        return;
    }

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = this->adc_atten_,
        .bitwidth = ADC_BITWIDTH_13,
    };

    err = adc_oneshot_config_channel(this->adc_handle_, this->adc_channel_, &channel_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC channel %d: %d", this->adc_channel_, err);
        this->status_set_error();
        return;
    }


    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &periodic_timer_callback,
        .arg = this, // Pass the MotorController instance
        .name = "periodic"
    };

    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &this->periodic_timer_));
    ESP_ERROR_CHECK(esp_timer_start_periodic(this->periodic_timer_, this->timer_period_));      
}


void MotorController::on_timer() {

    // upt to 250µs
    int64_t time_since_boot = esp_timer_get_time();

    // Get the current position and velocity
    current_position_ = this->get_position();
    float velocity = this->get_velocity();

    check_manual_override(); // Check for manual override


    // Define the state enum
    enum class State {
        INITIALIZE,
        IDLE,
        MOVE2POS,
        FORCEFEEDBACK
    };

    // Store the current state
    static State current_state = State::INITIALIZE;

    switch (current_state) {
        case State::INITIALIZE:
            // Initialize the motor
            if (is_initialized_) {
                ESP_LOGD(TAG, "INITIALIZE -> IDLE (Motor initialized)");
                current_state = State::IDLE;
            } 
            break;

        case State::IDLE:
            // In IDLE state, apply force feedback if the slider is unsteady
            if (this->monitor_manual_movements()) { 
                ESP_LOGD(TAG, "IDLE -> FORCEFEEDBACK (Slider unsteady)");
                current_state = State::FORCEFEEDBACK;
            } else if (!target_reached_) {
                ESP_LOGD(TAG, "IDLE -> MOVE2POS (New trajectory)");
                current_state = State::MOVE2POS;
            }
            break;

        case State::MOVE2POS:
            // In MOVE2POS state, follow the trajectory
            if (manual_override_) {
                ESP_LOGD(TAG, "MOVE2POS -> FORCEFEEDBACK (Manual override)");
                current_state = State::FORCEFEEDBACK;
            } else if (target_reached_) {
                ESP_LOGD(TAG, "MOVE2POS -> IDLE (Target reached)");
                current_state = State::IDLE;
            } else {

                // Follow the trajectory, if finished, switch to IDLE, but delay transition by 0.5 seconds
                static uint32_t target_reached_time = 0;
                if (this->follow_trajectory()) {
                    if (target_reached_time == 0) {
                        target_reached_time = millis();
                    }
                    if (millis() - target_reached_time > 500) {
                        ESP_LOGD(TAG, "MOVE2POS -> IDLE (Trajectory finished)");
                        current_state = State::IDLE;
                        target_reached_time = 0; // Reset the timer
                    }
                } else {
                    // Reset the timer if the trajectory is not finished
                    target_reached_time = 0;
                }
            }
            break;

        case State::FORCEFEEDBACK:
            

            
            if (!this->monitor_manual_movements()) {
                ESP_LOGD(TAG, "FORCEFEEDBACK -> IDLE (Slider steady)");
                current_state = State::IDLE;
            } else if (!target_reached_) {
                ESP_LOGD(TAG, "FORCEFEEDBACK -> MOVE2POS (New trajectory)");
                current_state = State::MOVE2POS;
            }
            break;
    }

    // Perform actions based on the current state
    switch (current_state) {
        case State::INITIALIZE:
            this->initialize_motor();
            break;
        case State::IDLE:

            {
                pwm_forward_->set_level(0.0f);
                pwm_backward_->set_level(0.0f);
            }
            break;

        case State::MOVE2POS:
            //this->follow_trajectory();
            //already done in the state machine
            break;

        case State::FORCEFEEDBACK:
            this->set_force_feedback_torque();
            break;
    }


    iSR_runtime_ = esp_timer_get_time() - time_since_boot;

}



float MotorController::calculate_dither() {
    if (this->dither_amplitude_ <= 0.001f) {
        return 0.0f;
    }
    
    if (this->dither_frequency_ < 1) this->dither_frequency_ = 1;

    uint32_t half_period = 500000 / this->dither_frequency_;
    if (half_period == 0) {
        return 0.0f;
    }

    float clamped_idle = this->dither_idle_ratio_;
    if (clamped_idle < 0.0f) clamped_idle = 0.0f;
    if (clamped_idle > 0.49f) clamped_idle = 0.49f;

    uint32_t active_duration = static_cast<uint32_t>(half_period * (1.0f - clamped_idle));
    if (active_duration == 0) {
        return 0.0f;
    }
    
    uint32_t cycle = micros() % (2 * half_period);
    bool positive_half = cycle < half_period;
    uint32_t phase_time = positive_half ? cycle : (cycle - half_period);

    if (phase_time < active_duration) {
        return positive_half ? this->dither_amplitude_ : -this->dither_amplitude_;
    }

    return 0.0f;
}


void MotorController::set_force_feedback_torque() {
    float torque = calculate_rastpunkt_force(true);
    this->set_torque(torque);
}

// set torque to motor
void MotorController::set_torque(float torque) {
    float commanded = clamp(torque, -max_torque_, max_torque_);
    float friction_offset = 0.0f;
    float velocity = this->current_velocity_;
    float abs_velocity = fabsf(velocity);

    if (abs_velocity < this->friction_velocity_threshold_) {
        if (commanded > this->friction_command_threshold_) {
            friction_offset = this->friction_motor_forward_;
        } else if (commanded < -this->friction_command_threshold_) {
            friction_offset = -this->friction_motor_backward_;
        }

        if (friction_offset != 0.0f && this->friction_deadband_ > 0.0f) {
            float position_error = 0.0f;
            if (!this->target_reached_) {
                position_error = this->expected_position_ - this->current_position_;
            }
            float abs_error = fabsf(position_error);
            float scale = 1.0f;
            if (this->target_reached_) {
                scale = 0.0f;
            } else if (abs_error < this->friction_deadband_) {
                scale = abs_error / this->friction_deadband_;
            }
            friction_offset *= scale;
        }
    } else {
        friction_offset = this->friction_dynamic_gain_ * velocity;
    }

    float dither = this->calculate_dither();
    float output = commanded + friction_offset + dither;
    output = clamp(output, -1.0f, 1.0f);

    if (output > 0.0f) {
        pwm_forward_->set_level(output);
        pwm_backward_->set_level(0.0f);
    } else if (output < 0.0f) {
        pwm_forward_->set_level(0.0f);
        pwm_backward_->set_level(-output);
    } else {
        pwm_forward_->set_level(0.0f);
        pwm_backward_->set_level(0.0f);
    }

    last_compensated_torque = output;

    static uint32_t last_log_time = 0;
    uint32_t current_time = millis();
    if (diagnostic_logging_enabled_ && current_time - last_log_time >= 500) {
        last_log_time = current_time;
        ESP_LOGI(TAG, "cmd=%.3f fric=%.3f dith=%.3f out=%.3f vel=%.2f pos=%.2f", commanded, friction_offset, dither, output, velocity, current_position_);
    }
}

//PID controller to move motor to target position
void MotorController::move_to_position(float next_position) {
    // PID controller variables
    float kp = pid_kp_;
    float ki = pid_ki_;
    float kd = pid_kd_;
    static float integral = 0.0f;
    static float last_error = 0.0f;
    float integral_limit = 10.0f; // Anti-windup limit, based on error not pi
    float last_torque = 0.0f;

    // Calculate error‚
    float error = next_position - this->current_position_;

    // the allowed error is small until we reach the target position
    // if next position has not been changed for 100ms allowed error is 0.8
    // otherwise 0
    static uint32_t last_position_change_time = 0;
    static float last_next_position_ = 0.0f;

    if (next_position != last_next_position_) {
        last_position_change_time = millis();
        last_next_position_ = next_position;
        integral = 0.0f;
        last_error = error;
    }

    uint32_t time_since_change = millis() - last_position_change_time;
    float allowed_error = 2.0f;

    if (time_since_change > 400 && fabsf(error) <= allowed_error) {
        this->set_torque(0.0f);
        return;
    }


    if (fabsf(error) < allowed_error) {   // stop the motor if the error is less than 0.1

        //this->set_torque(0.0f);
        //return;
        error = 0.0f;
    }


    // Calculate integral with anti-windup
    integral += error;
    if (integral > integral_limit) {
        integral = integral_limit;
    } else if (integral < -integral_limit) {
        integral = -integral_limit;
    }

    // Calculate derivative
    float derivative = error - last_error;
    last_error = error;

    // Calculate torque
    float torque = kp * error + ki * integral + kd * derivative;

    // Reset integral if direction changes
    if ((torque > 0 && last_torque < 0) || (torque < 0 && last_torque > 0)) {
        integral = 0.0f;
    }
    last_torque = torque;

    // output internal controller values every 2 seconds
    static uint32_t last_log_time = 0;
    uint32_t current_time = millis();
    if (current_time - last_log_time >= 2000) {
        last_log_time = current_time;
        ESP_LOGD(TAG, "Error: %.2f, Integral: %.2f, Torque: %.2f", error, integral, torque);
    }


    // Set torque to motor
    this->set_torque(torque);
}

// detects manual interaction while idle or force feedback
bool MotorController::monitor_manual_movements() {
    float last_position = current_position_;
    static bool last_move_state = false;

    const int window_size = 16;
    static float position_history[window_size];
    static int history_index = 0;
    static float last_steady_position = 0.0f;
    float position_average = current_position_;

    // Update position history
    position_history[history_index] = current_position_;
    history_index = (history_index + 1) % window_size;

    // Calculate new average
    float sum = 0;
    for (int i = 0; i < window_size; i++) {
        sum += position_history[i];
    }
    position_average = sum / window_size;

    float position_change = abs(position_average - last_position);

    //log every second the position change
    static uint32_t last_log_time = 0;  
    uint32_t current_time = millis();
    // if (current_time - last_log_time >= 1000) {
    //     last_log_time = current_time;
    //     ESP_LOGD(TAG, "Position change detected: %.2f", position_change);
    // }

    static uint32_t last_interaction_check_time = 0;
    // Check if the position has changed significantly
    // in the already unsteady state be very generours with the threshold
    if (((labs(last_steady_position - position_average) > 6) && last_move_state == false) ||
        (position_change > 3.0f && last_move_state == true)) {
        last_interaction_check_time = current_time;
        last_move_state = true;
        return true;
    }
    
    // only return false if no manual interaction was detected for 0.5 seconds
    if (current_time - last_interaction_check_time >= 1500) {
        last_interaction_check_time = current_time;
        last_move_state = false;
        last_steady_position = position_average;
        return false;
    }
    return last_move_state;
}



// Rastpunktkraft berechnen
float MotorController::calculate_rastpunkt_force(bool printlog) {
    // Calculates a restoring force to the nearest detent points (rastpunkte).
    // The force is limited by the estimated friction to prevent excessive torque.

    if (!rastpunkte_enabled_ || rastpunkte_.empty()) {
        return 0.0f;
    }

    float absolute_tolerance = 1.5f; // Absolute tolerance to consider a rastpunkt "reached"
    float current_position = this->get_position(false); // Get unfiltered position


    float nearest_distance = std::numeric_limits<float>::max();
    const Rastpunkt* nearest_rastpunkt = nullptr;


    // 1. Find the nearest Rastpunkt
    for (const Rastpunkt &rastpunkt : rastpunkte_) {
        float distance = fabs(current_position - rastpunkt.position);
        if (distance < nearest_distance) {
            nearest_distance = distance;
            nearest_rastpunkt = &rastpunkt;
        }
    }

    float force = 0.0f; // Initialize force to zero

    // 2. Calculate force to the nearest Rastpunkt based on distance and stiffness
    if (nearest_rastpunkt != nullptr) {
        float distance_to_rastpunkt = current_position - nearest_rastpunkt->position;

        // 3. Check if the slider is already moving towards the nearest rastpunkt
        // If the velocity has the same sign as the direction to the rastpunkt, disable force feedback
        if ((current_velocity_ > 60 && distance_to_rastpunkt > 0) ||
            (current_velocity_ < -30 && distance_to_rastpunkt < 0)) {
            return 0.0f;
        }

        // 4. Check if the distance to the rastpunkt is within the tolerances (reached)
        if (fabs(distance_to_rastpunkt) > absolute_tolerance) {
            // 5. Calculate blend factor (0.0 at tolerance, 1.0 at 2x tolerance)
            float blend_factor = fmin(1.0f, (fabs(distance_to_rastpunkt) - absolute_tolerance) / absolute_tolerance);

            // 6. Apply the stiffness as the force, direction depends on the distance
            float stiffness_force = (distance_to_rastpunkt > 0) ? -nearest_rastpunkt->stiffness : nearest_rastpunkt->stiffness;

            // 7. Blend in the stiffness force and global factor
            force = blend_factor * stiffness_force * force_feedback_strength_;

            // // 8. Limit the force to the friction to prevent excessive torque
            // float friction = this->calculate_friction();    // Calculate friction
            // if (fabs(force) > friction) {
            //     force = (force > 0) ? friction : -friction; // Limit force to friction
            // }
        } else {
            // 9. If within tolerance, apply no force (already at rastpunkt)
            force = 0.0f;
        }
    }

    // reduce the force linearly according to the velocity with damping factor to prevent high slider speeds
    float damping_factor = 0.015f;

    // 10. Apply damping to the force based on the current velocity
    // Damping is blended in between 20 and 40 %/s
    float damping_min_speed_ = 10.0f;
    float damping_max_speed_ = 20.0f;

    if (fabs(current_velocity_) > damping_min_speed_) {
        float damping_blend = 0.0f;
        if (fabs(current_velocity_) < damping_max_speed_) {
            damping_blend = (fabs(current_velocity_) - damping_min_speed_) / (damping_max_speed_ - damping_min_speed_);
        } else {
            damping_blend = 1.0f;
        }
        force -= damping_blend * damping_factor * current_velocity_;
    }




    // 7. Log the force and the calculated values to get there every second
    if (printlog) {
        static uint32_t last_log_time = 0;
        uint32_t current_time = millis();
        if (current_time - last_log_time >= 1000) {
            last_log_time = current_time;
            ESP_LOGD(TAG, "Force: %.2f, Current position: %.2f, Nearest rastpunkt: %.2f, Stiffness: %.2f",
                     force, current_position,
                     (nearest_rastpunkt != nullptr) ? nearest_rastpunkt->position : NAN,
                     (nearest_rastpunkt != nullptr) ? nearest_rastpunkt->stiffness : NAN);
        }
    }

    return force;
}

// Automatically determin friction of the motor
bool MotorController::determine_friction() { // returns true if friction was determined or aborted
    // This function is called after the initialization of the motor
    // the friction detection has several states:
    // 1. Move motor to middle position and stop
    // 2. Wait for a certain period
    // 3. set forwward friction to zero and start increasing torque until the motor moves
    // 4. stop motor
    // 5. Wait for a certain period
    // 6. set backward friction to zero and start increasing torque until the motor moves
    // 7. stop motor    
    enum InitState {
        MOVE_TO_MIDDLE,
        WAIT_1,
        INCREASE_TORQUE_FORWARD,
        WAIT_2,
        INCREASE_TORQUE_BACKWARD,
        STOP

    };



    static InitState state = MOVE_TO_MIDDLE;  // slider already at 50
    static uint32_t state_start_time = 0;

    switch (state) {
        case MOVE_TO_MIDDLE:
            // Move motor to middle position
            state_start_time = millis();

            static float initial_position_forward = this->get_position();
            state = WAIT_1;
            break;

        case WAIT_1:
            // Wait for a certain period
            if (millis() - state_start_time > 1000) { // Wait for 2 seconds
                state = INCREASE_TORQUE_FORWARD;

                state_start_time = millis();
                initial_position_forward = this->get_position();
            }
            break;

        case INCREASE_TORQUE_FORWARD:
            // Reset friction to 0 and store initial position
            static float friction_forward = 0.0f;

            // Increase friction by 0.01
            friction_forward += 0.01f;
            this->set_torque(friction_forward);

            // Check if position change is greater than 1
            if (fabs(this->get_position(false) - initial_position_forward) > 3.0f) {
                friction_motor_forward_ = friction_forward;
                state = WAIT_2;
                state_start_time = millis();
                this->set_torque(0);
            } else if (millis() - state_start_time > 5000) { // Timeout after 5 seconds
                ESP_LOGE(TAG, "Timeout reached while increasing forward torque");
                state = WAIT_2;
                state_start_time = millis();
                this->set_torque(0);
            }
            break;

        case WAIT_2:
            // Wait for a certain period
            if (millis() - state_start_time > 1500) { // Wait for 2 seconds
                state = INCREASE_TORQUE_BACKWARD;
            }
            break;

        case INCREASE_TORQUE_BACKWARD:
            // Reset friction to 0 and store initial position
            static float initial_position_backward = this->get_position(false);
            static float friction_backward = 0.0f;
            this->set_torque(-friction_backward); // Apply current friction

            // Increase friction by 0.01
            friction_backward += 0.01f;
            this->set_torque(-friction_backward);

            // Check if position change is greater than 1
            if (fabs(this->get_position(false) - initial_position_backward) > 3.0f) {
                friction_motor_backward_ = friction_backward;
                friction_determined_ = true;
                this->set_torque(0);
                // Log info friction values
                ESP_LOGI(TAG, "Forward friction: %.2f, Backward friction: %.2f", friction_motor_forward_, friction_motor_backward_);
                return true;
            } else if (millis() - state_start_time > 5000) { // Timeout after 5 seconds
                ESP_LOGE(TAG, "Timeout reached while increasing backward torque");
                state = STOP;
                this->set_torque(0);
            }
            break;

        case STOP:
            // Stop the motor
            return true;
            break;

    }

    // on each state change log the current state
    static InitState last_state = state;
    if (last_state != state) {
        const char* state_names[] = {
            "MOVE_TO_MIDDLE",
            "WAIT_1",
            "INCREASE_TORQUE_FORWARD",
            "WAIT_2",
            "INCREASE_TORQUE_BACKWARD",
            "STOP"
        };
        ESP_LOGI(TAG, "State changed to: %s", state_names[state]);
        last_state = state;
    }

    return false;
}   

// autotune kalman filter
void MotorController::calculate_measurement_error() {
    float sum = 0.0f;
    float sum_squared = 0.0f;
    int num_measurements = 20;

    for (int i = 0; i < num_measurements; ++i) {
        float measurement = this->get_position(false); // unfiltered
        sum += measurement;
        sum_squared += measurement * measurement;
    }

    float mean = sum / num_measurements;
    float variance = (sum_squared / num_measurements) - (mean * mean);
    if (!std::isfinite(variance) || variance < 0.0f) {
        variance = 0.0f;
    }
    this->measurement_error_ = sqrt(variance);

    if (!std::isfinite(this->measurement_error_) || this->measurement_error_ < 1e-4f) {
        this->measurement_error_ = 1e-4f;
    }

    this->estimate_error_ = this->measurement_error_;
    this->kalman_initialized_ = false;

    ESP_LOGD(TAG, "Calculated measurement error: %.2f", this->measurement_error_);


}

void MotorController::initialize_motor() {
    // Motor initialisieren
    // Kalman filter initialisieren
    float position = this->get_position(false);

        enum State {
            WAIT_WIFI,
            MOVE_TO_MIDDLE,
            AUTOTUNE,
            DETECT_FRICTION,
            MOVE_TO_100,
            MOVE_TO_0,
            END,
        };;

        static State state = WAIT_WIFI;
        static uint32_t state_start_time =  0;
        static int raw_min_value = std::numeric_limits<int>::max();
        static int raw_max_value = std::numeric_limits<int>::min();
        static uint32_t last_extreme_update_time = 0;

        switch (state) {
            case WAIT_WIFI:
                // Wait for WiFi to connect
                if (millis() - state_start_time > 8000) { // Wait for 8 seconds
                    state = MOVE_TO_MIDDLE;

                    state_start_time = millis();
                    raw_min_value = std::numeric_limits<int>::max();
                    raw_max_value = std::numeric_limits<int>::min();
                    last_extreme_update_time = millis();
                }
                break;


            case MOVE_TO_MIDDLE:
                // Move motor to middle position
                move_to_position(50);
                if (position >= 48.0f && position <= 52.0f && millis() - state_start_time >500) {
                    this->set_torque(0.0f);
                    state = AUTOTUNE;
                    state_start_time = millis();
                } else if (millis() - state_start_time > 3000) { // Timeout after 3 seconds

                    this->set_torque(0.0f);
                    ESP_LOGW(TAG, "Timeout reached while moving to middle position. Current position: %.2f", position);
                    state = AUTOTUNE;
                    state_start_time = millis();
                }
                break;

            case AUTOTUNE:
                // Perform autotune
                this->calculate_measurement_error();
                state = MOVE_TO_100; // Restart the state machine or set to another state as needed
                // log the measurement error

                state_start_time = millis();
                ESP_LOGI(TAG, "Measurement error: %.2f", this->measurement_error_);
                break;

            case DETECT_FRICTION:
                // Perform friction detection
                if (this->determine_friction()) {   // returns true if friction was determined or aborted
                    state = MOVE_TO_100; // Restart the state machine or set to another state as needed
                    // log the friction values
                    ESP_LOGI(TAG, "Forward friction: %.2f, Backward friction: %.2f", friction_motor_forward_, friction_motor_backward_);

                    state_start_time = millis();
                }
                break;

            case MOVE_TO_100:
                // Set torque to move to position for 1 second
                this->set_torque(1.0f);
                {
                    int raw_sample = static_cast<int>(get_position(true, true));
                    if (raw_sample < raw_min_value) {
                        raw_min_value = raw_sample;
                        last_extreme_update_time = millis();
                    }
                }

                if ((millis() - state_start_time > 4000) ||
                    ((millis() - last_extreme_update_time) > 500 && fabsf(current_velocity_) < 5.0f)) {
                    this->set_torque(0.0f);
                    if (raw_min_value != std::numeric_limits<int>::max()) {
                        position100_raw_value_ = raw_min_value;
                    } else {
                        position100_raw_value_ = static_cast<int>(get_position(true, true));
                    }

                    state = MOVE_TO_0;
                    state_start_time = millis();
                    last_extreme_update_time = millis();
                }
                break;

            case MOVE_TO_0:
                // Set torque to move to position for 1 second
                this->set_torque(-1.0f);
                {
                    int raw_sample = static_cast<int>(get_position(true, true));
                    if (raw_sample > raw_max_value) {
                        raw_max_value = raw_sample;
                        last_extreme_update_time = millis();
                    }
                }

                if ((millis() - state_start_time > 4000) ||
                    ((millis() - last_extreme_update_time) > 500 && fabsf(current_velocity_) < 5.0f)) {
                    this->set_torque(0.0f);
                    if (raw_max_value != std::numeric_limits<int>::min()) {
                        position0_raw_value_ = raw_max_value;
                    } else {
                        position0_raw_value_ = static_cast<int>(get_position(true, true));
                    }

                    state = END;

                    // log the raw values
                    ESP_LOGI(TAG, "Position 0 raw value: %d, Position 100 raw value: %d", position0_raw_value_, position100_raw_value_);
                }
                break;


            case END:
                // Stop the motor
                 this->is_initialized_ = true;
                break;
        }

    // on each state change log the current state
    static State last_state = state;
    if (last_state != state) {
        const char* state_names[] = {
            "WAIT_WIFI",
            "MOVE_TO_MIDDLE",
            "AUTOTUNE",
            "DETECT_FRICTION",
            "MOVE_TO_100",
            "MOVE_TO_0",
            "END"
        };
        ESP_LOGI(TAG, "State init changed to: %s", state_names[state]);
        last_state = state;
    }
}

// aktuelle Position auslesen
float MotorController::get_position(bool filtered , bool raw) {
    if (this->adc_handle_ == nullptr) {
        ESP_LOGE(TAG, "ADC handle not initialized");
        return raw ? 0.0f : current_position_;
    }

    int analog_value = 0;
    int successful_samples = 0;
    for (int i = 0; i < 4; i++) {
        int sample = 0;
        esp_err_t err = adc_oneshot_read(this->adc_handle_, this->adc_channel_, &sample);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "ADC read failed: %d", err);
            continue;
        }
        analog_value += sample;
        successful_samples++;
    }

    if (successful_samples == 0) {
        ESP_LOGE(TAG, "No valid ADC samples available");
        return raw ? 0.0f : current_position_;
    }

    analog_value /= successful_samples;

    if (raw) {
        return analog_value;
    }

    // Convert analog value to position
    // scale from position0_raw_value_ to position100_raw_value_
    int raw_span = position100_raw_value_ - position0_raw_value_;
    if (raw_span == 0) {
        raw_span = 1;
    }
    float scaled_position = (float)(analog_value - position0_raw_value_) / (float)raw_span * 100.0f;
    if (scaled_position < 0.0f) {
        scaled_position = 0.0f;
    } else if (scaled_position > 100.0f) {
        scaled_position = 100.0f;
    }

    float raw_position = scaled_position;

    if (!filtered) {
        return raw_position;
    }

    if (!kalman_initialized_) {
        kalman_estimate_ = raw_position;
        kalman_initialized_ = true;
        if (!std::isfinite(measurement_error_) || measurement_error_ < 1e-4f) {
            measurement_error_ = 1e-4f;
        }
        estimate_error_ = measurement_error_;
        kalman_gain_ = 1.0f;
        return kalman_estimate_;
    }

    estimate_error_ += process_noise_;
    float meas_err = measurement_error_;
    if (!std::isfinite(meas_err) || meas_err < 1e-6f) {
        meas_err = 1e-4f;
    }
    float denom = estimate_error_ + meas_err;
    if (denom < 1e-6f) {
        denom = 1e-6f;
    }
    kalman_gain_ = estimate_error_ / denom;
    kalman_estimate_ = kalman_estimate_ + kalman_gain_ * (raw_position - kalman_estimate_);
    estimate_error_ = (1.0f - kalman_gain_) * estimate_error_;

    if (!std::isfinite(kalman_estimate_)) {
        kalman_estimate_ = raw_position;
        estimate_error_ = meas_err;
    }

    if (diagnostic_logging_enabled_) {
        uint32_t now = millis();
        if (now - last_kalman_log_ >= 500) {
            last_kalman_log_ = now;
            ESP_LOGI(TAG, "kalman raw=%.2f filt=%.2f K=%.3f Q=%.5f R=%.3f err=%.4f", raw_position, kalman_estimate_, kalman_gain_, process_noise_, measurement_error_, estimate_error_);
        }
    }

    return kalman_estimate_;







}
 

float MotorController::get_velocity() {
    // Calculate velocity based on the difference to the previous position
    // Store last run time
    static uint32_t last_run_time = 0;
    uint32_t current_time = micros();
    static float last_position = 0.0f;

    // Calculate the time since the last run
    uint32_t time_diff = current_time - last_run_time;
    if (time_diff == 0) {
        return current_velocity_; // Avoid division by zero
    }

    // Calculate the difference in position since the last run
    float position_diff = current_position_ - last_position;

    // Calculate the velocity in percentage per second
    float actual_velocity = position_diff / ((float)time_diff / 1000000.0f);

    last_run_time = current_time;
    last_position = current_position_;

    // Apply PT1 filter to smooth the velocity

    // richrige PT1 filter
    static float last_filtered_velocity = 0.0f;
    float T = time_diff / 1000000.0f; // Convert time_diff to seconds
    float tau = velocity_pt1_time_const_; // Time constant of the filter in seconds
    float alpha = T / (tau + T);

    current_velocity_ = alpha * actual_velocity + (1 - alpha) * last_filtered_velocity;


    // every 1000 cycles log the velocity and all internal variables
    static uint32_t last_log_time = 0;
    uint32_t current_time_millis = millis();
    // if (current_time_millis - last_log_time >= 1000) {
    //     last_log_time = current_time_millis;
    //     ESP_LOGD(TAG, "Velocity: %.2f, Actual velocity: %.2f, last velocity: %.2f, Position diff: %.2f, Time diff: %d", current_velocity_, actual_velocity,last_filtered_velocity, position_diff, time_diff);
    // }
    last_filtered_velocity = current_velocity_;

    // set velocities smaller 10 to 0

    if (fabs(current_velocity_) < 9.0f) {
        current_velocity_ = 0.0f;
    }



    return current_velocity_;
}

// Plan the trajectory for a motion profile.

void MotorController::plan_trajectory(float start_position, float target_position, 
                     float max_velocity, float max_acceleration) {
    // This function calculates the timing and parameters for a trapezoidal velocity profile
    // or an S-curve profile, depending on the configuration.
    // Inputs:
    // - start_position: The initial position of the system.
    // - target_position: The desired target position to reach.
    // - max_velocity: The maximum allowed velocity.
    // - max_acceleration: The maximum allowed acceleration.
    // Outputs:
    // - Updates internal variables to store the timing and key points of the trajectory.
    // - These include acceleration time, constant velocity time, and deceleration time.
    // Assumptions:
    // - The profile assumes the system starts at rest (velocity = 0).
    // - The profile assumes the system will stop at the target (velocity = 0).

    // Step 1: Calculate the total distance to travel
    float distance = abs(target_position - start_position);

    // Step 2: Determine if the motion is symmetric (acceleration = deceleration)
    // If the distance is too short to reach max_velocity, limit the velocity.
    float t_acc = max_velocity / max_acceleration;
    float d_acc = 0.5f * max_acceleration * t_acc * t_acc;

    if (distance < 2 * d_acc) {
        // Distance is too short to reach max_velocity
        t_acc = sqrt(distance / max_acceleration);
        d_acc = 0.5f * max_acceleration * t_acc * t_acc;
    }

    // Step 3: Calculate the acceleration and deceleration times (t_acc and t_dec).
    // Use the formula: t_acc = max_velocity / max_acceleration
    // Adjust t_acc if the distance is too short to allow full acceleration.
    float t_dec = t_acc;
    float d_dec = d_acc;

    if (distance < 2 * d_acc) {
        t_dec = t_acc;
        d_dec = d_acc;
    } else {
        float remaining_distance = distance - 2 * d_acc;
        float t_const = remaining_distance / max_velocity;
    }

    // Step 4: Calculate the time at constant velocity (t_const).
    // Use the formula: t_const = (distance - 2 * d_acc) / max_velocity
    // If t_const < 0, reduce t_acc and t_dec proportionally.
    float t_const = (distance - 2 * d_acc) / max_velocity;
    if (t_const < 0) {
        t_const = 0;
        t_acc = sqrt(distance / (2 * max_acceleration));
        t_dec = t_acc;
    }

    // Step 5: Store the computed trajectory parameters:
    // - Total trajectory time (t_total = t_acc + t_const + t_dec).
    // - Timing of key points (t1 = end of acceleration, t2 = start of deceleration).
    float t_total = t_acc + t_const + t_dec;
    float t1 = t_acc;
    float t2 = t_acc + t_const;

    // Store the computed parameters in the class or a struct for later use
    this->trajectory_.t_total = t_total;
    this->trajectory_.t1 = t1;
    this->trajectory_.t2 = t2;
    this->trajectory_.start_position = start_position;
    this->trajectory_.target_position = target_position;
    this->trajectory_.max_velocity = max_velocity;
    this->trajectory_.max_acceleration = max_acceleration;


    // Initialize the start time of the trajectory
    this->trajectory_start_time_ = millis();

    // Step 6: Validate the trajectory.
    // Ensure that the profile respects all constraints (e.g., max velocity and acceleration).
    if (t_total <= 0 || t1 <= 0 || t2 < t1) {
        ESP_LOGE(TAG, "Invalid trajectory parameters: t_total=%.2f, t1=%.2f, t2=%.2f", t_total, t1, t2);
        return;
    }

    // Step 7: Log or output the calculated trajectory parameters for debugging.
    ESP_LOGI(TAG, "Trajectory planned: t_total=%.2f, t1=%.2f, t2=%.2f, start_position=%.2f, target_position=%.2f, max_velocity=%.2f, max_acceleration=%.2f",
             t_total, t1, t2, start_position, target_position, max_velocity, max_acceleration);

}

// Move the motor to the target position using the planned trajectory. returns true if the target position is reached
bool MotorController::follow_trajectory() {
    // This function calculates the desired position at each time step to follow the planned trajectory.

    // It uses the calculated trajectory parameters to determine the desired position at each time step.
    // Outputs:
    // - Updates the motor controller with the desired position at each time step.

    // Step 1: Get the current time since the start of the trajectory.
    // Use the millis() function or a similar time source.
    uint32_t current_time = millis();
    float elapsed_time = (current_time - this->trajectory_start_time_) / 1000.0f; // Convert to seconds

    // Step 2: Calculate the desired position based on the elapsed time.
    // Use the planned trajectory parameters to determine the desired position at the current time.
    float desired_position = 0.0f;
    float direction = (this->trajectory_.target_position > this->trajectory_.start_position) ? 1.0f : -1.0f;
    const char *phase = "ACC";
    float commanded_velocity = 0.0f;

    // initially we have to pretension the belt

    if (elapsed_time < this->trajectory_.t1) {
        // Acceleration phase
        desired_position = this->trajectory_.start_position + direction * 0.5f * this->trajectory_.max_acceleration * elapsed_time * elapsed_time;
        commanded_velocity = direction * this->trajectory_.max_acceleration * elapsed_time;
    } else if (elapsed_time < this->trajectory_.t2) {
        // Constant velocity phase
        float t = elapsed_time - this->trajectory_.t1;
        desired_position = this->trajectory_.start_position + direction * (this->trajectory_.max_velocity * t + 0.5f * this->trajectory_.max_acceleration * this->trajectory_.t1 * this->trajectory_.t1);
        phase = "CRU";
        commanded_velocity = direction * this->trajectory_.max_velocity;
    } else if (elapsed_time < this->trajectory_.t_total) {
        // Deceleration phase
        float t = elapsed_time - this->trajectory_.t_total;
        desired_position = this->trajectory_.target_position - direction * 0.5f * this->trajectory_.max_acceleration * t * t;
        phase = "DEC";
        float time_to_stop = this->trajectory_.t_total - elapsed_time;
        commanded_velocity = direction * this->trajectory_.max_acceleration * time_to_stop;
    } else {
        // Trajectory complete
        desired_position = this->trajectory_.target_position;
        this->move_to_position(desired_position);
        
        // target_reached_ = true,
        this->target_reached_ = true;

        return true;
    }

    // Update expected position
    expected_position_ = desired_position;

    // Move the motor to the desired position (move_to_position)
    this->move_to_position(desired_position);

    if (diagnostic_logging_enabled_) {
        static uint32_t last_traj_log = 0;
        if (current_time - last_traj_log >= 33) {
            last_traj_log = current_time;
            ESP_LOGI(TAG, "traj phase=%s t=%.3fs des=%.2f cur=%.2f vel_cmd=%.2f vel_act=%.2f err=%.2f", phase, elapsed_time, desired_position, current_position_, commanded_velocity, current_velocity_, desired_position - current_position_);
        }
    }

    // Log the desired position and elapsed time for debugging
    //ESP_LOGD(TAG, "Desired position: %.2f, Elapsed time: %.2f", desired_position, elapsed_time);
    return false;
}



void MotorController::set_target_position(float position) {
    this->target_position_ = position ;

    // clamp target position to 0-1
    // log W if outside of range
    if (this->target_position_ < 0.0f) {
        this->target_position_ = 0.0f;
        ESP_LOGW(TAG, "Target position below 0.0, clamped to 0.0");
    } else if (this->target_position_ > 1.0f) {
        this->target_position_ = 1.0f;
        ESP_LOGW(TAG, "Target position above 100.0, clamped to 100.0");
    }

    // scale to 0-100, the way we handle the position internally
    this->target_position_ *= 100.0f;
    // log new target position
    ESP_LOGI(TAG, "New target position: %.2f", this->target_position_);

    // Plan the trajectory to the target position
    this->plan_trajectory(this->current_position_, this->target_position_, 
            this->max_slider_speed_, this->max_slider_acceleration_);

    // Reset the manual override flag
    this->manual_override_ = false;
    this->target_reached_ = false;
}


void MotorController::loop() {
    // Main loop code
    



    // Alle 2 Sekunden per log die Geschwindigkeit ud Position ausgeben
    static uint32_t last_log = 0;
    static uint32_t last_slider_publish_ = 0;

    uint32_t now = millis();
    
    //ESP_LOGD(TAG, "Current time: %u, Last log time: %u", now, last_log);
    if (now - last_log > 2000) {
        last_log = now;

        //  log the current position and velocity
        //ESP_LOGI(TAG, "Position: %.2f , Velocity: %.2f %/s, ISTime: %d µs", current_position_, current_velocity_, iSR_runtime_ );
        //

        //ESP_LOGI(TAG, "P: %.2f,", current_position_);
        // log runtime of addsensor
        //ESP_LOGI(TAG, "Duration of get_position: %u", duration_getsensor_);

        //calculate_rastpunkt_force(true);    // display the force to the nearest rastpunkt
    }


    if (slider_position_sensor_ != nullptr) {
        if (now - last_slider_publish_ > 150) {
            last_slider_publish_ = now;
            slider_position_sensor_->publish_state(current_position_);
        }
    }

}

void MotorController::add_rastpunkt(float position, float stiffness) {
    rastpunkte_.push_back({position, stiffness});
}

// detects ovveride while move to position
void MotorController::check_manual_override() {
    static bool override_detected = false;
    static uint32_t override_start_time = 0;

    if (!override_detection_enabled_) {
        manual_override_ = false;
        override_detected = false;
        return;
    }

    float position_difference = fabs(current_position_ - expected_position_);
    float velocity_difference = fabs(current_velocity_); // Compare with 0 if motor should be still

    // Skip override detection if not initialized or target reached
    if (!is_initialized_ || target_reached_) 
    {
        override_detected = false;
        return;
        
    }

    if (position_difference > override_position_threshold_ ||
        velocity_difference > override_velocity_threshold_) {
        if (!override_detected) {
            override_detected = true;
            override_start_time = millis();
        } else if (millis() - override_start_time > override_debounce_time_) {
            // Manual override detected
            manual_override_ = true;
            last_override_time_ = millis();
            // no need to detect again here, we are already in manual override
            target_reached_ = true;
            ESP_LOGW(TAG, "Manual override detected! Position diff: %.2f, Velocity: %.2f", position_difference, current_velocity_);
        }
    } else {
        override_detected = false;
        manual_override_ = false;
    }
}

}  // namespace motorized_slider
}  // namespace esphome