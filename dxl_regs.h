/*
 * Control tables, one per series: dxl_write(&bus, id, DXL_XM430.goal_position, 2048).
 * Missing registers are {0, 0} (DXL_ERR_ARG). Mixed models: dxl_regs(model).
 *
 * Except on the Y series, every register below torque_enable is EEPROM: it
 * survives power-off and can only be written with torque off (else "servo:
 * access error"). That's why changing operating_mode needs torque off first.
 *
 * Addresses from ROBOTIS Dynamixel2Arduino (Apache-2.0), X/MX/P Bus Watchdog
 * fixed to 1 byte (Y's is 2) and H54-100-S500-R(A) fixed to model 53769.
 * Units and ranges differ per model: see its e-Manual page.
 */
#pragma once
#include <stdint.h>

typedef struct {
    uint16_t addr;
    uint8_t len; /* 1, 2 or 4 */
} dxl_reg_t;

typedef struct {
    dxl_reg_t model_number, model_information, firmware_version, id;
    dxl_reg_t baud_rate, return_delay_time, drive_mode, operating_mode;
    dxl_reg_t secondary_id, protocol_version, homing_offset, moving_threshold;
    dxl_reg_t temperature_limit, max_voltage_limit, min_voltage_limit, pwm_limit;
    dxl_reg_t current_limit, acceleration_limit, velocity_limit, max_position_limit;
    dxl_reg_t min_position_limit, external_port_mode_1, external_port_mode_2, external_port_mode_3;
    dxl_reg_t external_port_mode_4, shutdown, torque_enable, led;
    dxl_reg_t led_red, led_green, led_blue, status_return_level;
    dxl_reg_t registered_instruction, hardware_error_status, velocity_i_gain, velocity_p_gain;
    dxl_reg_t position_d_gain, position_i_gain, position_p_gain, feedforward_2nd_gain;
    dxl_reg_t feedforward_1st_gain, bus_watchdog, goal_pwm, goal_current;
    dxl_reg_t goal_velocity, profile_acceleration, profile_velocity, goal_position;
    dxl_reg_t realtime_tick, moving, moving_status, present_pwm;
    dxl_reg_t present_load, present_current, present_velocity, present_position;
    dxl_reg_t velocity_trajectory, position_trajectory, present_input_voltage, present_temperature;
    dxl_reg_t external_port_data_1, external_port_data_2, external_port_data_3, external_port_data_4;
    /* PRO (R) */
    dxl_reg_t torque_limit, goal_torque, goal_acceleration;
    /* XL-320 */
    dxl_reg_t cw_angle_limit, ccw_angle_limit, control_mode, max_torque;
    dxl_reg_t d_gain, i_gain, p_gain, moving_speed;
    dxl_reg_t present_speed, punch;
    /* Y series */
    dxl_reg_t startup_configuration, position_limit_threshold, in_position_threshold, following_error_threshold;
    dxl_reg_t inverter_temperature_limit, motor_temperature_limit, electronic_gear_ratio_numerator, electronic_gear_ratio_denominator;
    dxl_reg_t safe_stop_time, brake_delay, goal_update_delay, overexcitation_voltage;
    dxl_reg_t normal_excitation_voltage, overexcitation_time, present_velocity_lpf_frequency, goal_current_lpf_frequency;
    dxl_reg_t position_ff_lpf_time, velocity_ff_lpf_time, controller_state, error_code;
    dxl_reg_t error_code_history1, hybrid_save, profile_acceleration_time, profile_time;
    dxl_reg_t pwm_offset, current_offset, velocity_offset, present_motor_temperature;
} dxl_regs_t;

extern const dxl_regs_t
    DXL_XL430,  /* XL430, XC430, 2XL430, 2XC430 (present_load, no current) */
    DXL_XM430,  /* XM/XH/XD430, XL/XC330, XM335, XW430/540 (current) */
    DXL_XM540,  /* XM/XH/XD540 (current + external ports) */
    DXL_MX28_2, /* MX-28(2.0) */
    DXL_MX64_2, /* MX-64(2.0), MX-106(2.0) */
    DXL_XL320,
    DXL_PRO,    /* PRO (R) */
    DXL_P,      /* PRO (RA), P series */
    DXL_Y;      /* YM070, YM080 */

/* ---- register values ---- */

/* baud_rate (X, MX(2.0)): 0 = 9600, 1 = 57600, 2 = 115200, 3 = 1M, 4 = 2M, 5 = 3M, 6 = 4M, 7 = 4.5M */

/*
 * hardware_error_status bits (what went wrong) and shutdown bits (which
 * faults cut torque). X, MX(2.0), P, PRO:
 */
enum {
    DXL_HW_INPUT_VOLTAGE = 1 << 0,
    DXL_HW_HALL_SENSOR = 1 << 1, /* P and PRO only */
    DXL_HW_OVERHEATING = 1 << 2,
    DXL_HW_ENCODER = 1 << 3,
    DXL_HW_ELECTRICAL_SHOCK = 1 << 4,
    DXL_HW_OVERLOAD = 1 << 5,
};
/* XL-320 uses different bits: */
enum {
    DXL_XL320_HW_OVERLOAD = 1 << 0,
    DXL_XL320_HW_OVERHEATING = 1 << 1,
    DXL_XL320_HW_INPUT_VOLTAGE = 1 << 2,
};
/* Y series error_code (a number, not bits). "clear": DXL_CLEAR_ERRORS resets it. */
enum {
    DXL_Y_ERR_NONE = 0,
    DXL_Y_ERR_OVER_VOLTAGE = 1,       /* clear */
    DXL_Y_ERR_LOW_VOLTAGE = 2,        /* clear */
    DXL_Y_ERR_INVERTER_OVERHEAT = 3,  /* clear */
    DXL_Y_ERR_MOTOR_OVERHEAT = 4,     /* clear */
    DXL_Y_ERR_OVERLOAD = 5,           /* clear */
    DXL_Y_ERR_INVERTER = 7,
    DXL_Y_ERR_BATTERY_WARNING = 9,
    DXL_Y_ERR_BATTERY = 10,
    DXL_Y_ERR_MAGNET = 11,
    DXL_Y_ERR_MULTI_TURN = 12,
    DXL_Y_ERR_ENCODER = 13,
    DXL_Y_ERR_HALL_SENSOR = 14,
    DXL_Y_ERR_CALIBRATION = 15,
    DXL_Y_ERR_FOLLOWING = 17,         /* clear */
    DXL_Y_ERR_BUS_WATCHDOG = 18,      /* clear */
    DXL_Y_ERR_OVER_SPEED = 19,        /* clear */
    DXL_Y_ERR_POSITION_LIMIT = 20,    /* clear */
};

enum { /* operating_mode values (X series) */
    DXL_MODE_CURRENT = 0,
    DXL_MODE_VELOCITY = 1,
    DXL_MODE_POSITION = 3,
    DXL_MODE_EXTENDED_POSITION = 4,
    DXL_MODE_CURRENT_POSITION = 5,
    DXL_MODE_PWM = 16,
};

/* Register set for a model number from dxl_ping, or NULL if unknown. */
const dxl_regs_t *dxl_regs(uint16_t model);
