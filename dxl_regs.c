/*
 * Protocol 2.0 control tables. Addresses ported from ROBOTIS Dynamixel2Arduino
 * (src/actuator.cpp, Apache-2.0).
 */
#include <stddef.h>
#include "dxl_regs.h"

/* Shared by every X and MX(2.0) table; each adds its model's extras. */
#define X_COMMON \
    .model_number = {0, 2}, .model_information = {2, 4}, \
    .firmware_version = {6, 1}, .id = {7, 1}, .baud_rate = {8, 1}, \
    .return_delay_time = {9, 1}, .drive_mode = {10, 1}, \
    .operating_mode = {11, 1}, .secondary_id = {12, 1}, \
    .protocol_version = {13, 1}, .homing_offset = {20, 4}, \
    .moving_threshold = {24, 4}, .temperature_limit = {31, 1}, \
    .max_voltage_limit = {32, 2}, .min_voltage_limit = {34, 2}, \
    .pwm_limit = {36, 2}, .velocity_limit = {44, 4}, \
    .max_position_limit = {48, 4}, .min_position_limit = {52, 4}, \
    .shutdown = {63, 1}, .torque_enable = {64, 1}, .led = {65, 1}, \
    .status_return_level = {68, 1}, .registered_instruction = {69, 1}, \
    .hardware_error_status = {70, 1}, .velocity_i_gain = {76, 2}, \
    .velocity_p_gain = {78, 2}, .position_d_gain = {80, 2}, \
    .position_i_gain = {82, 2}, .position_p_gain = {84, 2}, \
    .feedforward_2nd_gain = {88, 2}, .feedforward_1st_gain = {90, 2}, \
    .bus_watchdog = {98, 1}, .goal_pwm = {100, 2}, \
    .goal_velocity = {104, 4}, .profile_acceleration = {108, 4}, \
    .profile_velocity = {112, 4}, .goal_position = {116, 4}, \
    .realtime_tick = {120, 2}, .moving = {122, 1}, \
    .moving_status = {123, 1}, .present_pwm = {124, 2}, \
    .present_velocity = {128, 4}, .present_position = {132, 4}, \
    .velocity_trajectory = {136, 4}, .position_trajectory = {140, 4}, \
    .present_input_voltage = {144, 2}, .present_temperature = {146, 1}

/* Current sensing; models without it have present_load at 126 instead. */
#define X_CURRENT \
    .current_limit = {38, 2}, .goal_current = {102, 2}, \
    .present_current = {126, 2}

const dxl_regs_t DXL_XL430 = { X_COMMON, .present_load = {126, 2} }; /* XL430, XC430 */
const dxl_regs_t DXL_XM430 = { X_COMMON, X_CURRENT }; /* XM/XH/XD430, XL/XC330, XM335, XW */
const dxl_regs_t DXL_XM540 = { X_COMMON, X_CURRENT,
    .external_port_mode_1 = {56, 1}, .external_port_mode_2 = {57, 1},
    .external_port_mode_3 = {58, 1}, .external_port_data_1 = {152, 2},
    .external_port_data_2 = {154, 2}, .external_port_data_3 = {156, 2} };
const dxl_regs_t DXL_MX28_2 = { X_COMMON,
    .acceleration_limit = {40, 4}, .present_load = {126, 2} };
const dxl_regs_t DXL_MX64_2 = { X_COMMON, X_CURRENT, .acceleration_limit = {40, 4} };

const dxl_regs_t DXL_XL320 = {
    .model_number = {0, 2}, .firmware_version = {2, 1}, .id = {3, 1},
    .baud_rate = {4, 1}, .return_delay_time = {5, 1},
    .cw_angle_limit = {6, 2}, .ccw_angle_limit = {8, 2},
    .control_mode = {11, 1}, .temperature_limit = {12, 1},
    .min_voltage_limit = {13, 1}, .max_voltage_limit = {14, 1},
    .max_torque = {15, 2}, .status_return_level = {17, 1},
    .shutdown = {18, 1}, .torque_enable = {24, 1}, .led = {25, 1},
    .d_gain = {27, 1}, .i_gain = {28, 1}, .p_gain = {29, 1},
    .goal_position = {30, 2}, .moving_speed = {32, 2},
    .torque_limit = {35, 2}, .present_position = {37, 2},
    .present_speed = {39, 2}, .present_load = {41, 2},
    .present_input_voltage = {45, 1}, .present_temperature = {46, 1},
    .registered_instruction = {47, 1}, .moving = {49, 1},
    .hardware_error_status = {50, 1}, .punch = {51, 2} };

const dxl_regs_t DXL_PRO = {
    .model_number = {0, 2}, .model_information = {2, 4},
    .firmware_version = {6, 1}, .id = {7, 1}, .baud_rate = {8, 1},
    .return_delay_time = {9, 1}, .operating_mode = {11, 1},
    .homing_offset = {13, 4}, .moving_threshold = {17, 4},
    .temperature_limit = {21, 1}, .max_voltage_limit = {22, 2},
    .min_voltage_limit = {24, 2}, .acceleration_limit = {26, 4},
    .torque_limit = {30, 2}, .velocity_limit = {32, 4},
    .max_position_limit = {36, 4}, .min_position_limit = {40, 4},
    .external_port_mode_1 = {44, 1}, .external_port_mode_2 = {45, 1},
    .external_port_mode_3 = {46, 1}, .external_port_mode_4 = {47, 1},
    .shutdown = {48, 1}, .torque_enable = {562, 1},
    .led_red = {563, 1}, .led_green = {564, 1}, .led_blue = {565, 1},
    .velocity_i_gain = {586, 2}, .velocity_p_gain = {588, 2},
    .position_p_gain = {594, 2}, .goal_position = {596, 4},
    .goal_velocity = {600, 4}, .goal_torque = {604, 2},
    .goal_acceleration = {606, 4}, .moving = {610, 1},
    .present_position = {611, 4}, .present_velocity = {615, 4},
    .present_current = {621, 2}, .present_input_voltage = {623, 2},
    .present_temperature = {625, 1}, .external_port_data_1 = {626, 2},
    .external_port_data_2 = {628, 2}, .external_port_data_3 = {630, 2},
    .external_port_data_4 = {632, 2}, .registered_instruction = {890, 1},
    .status_return_level = {891, 1}, .hardware_error_status = {892, 1} };

const dxl_regs_t DXL_P = { /* PRO (RA) and PRO+ (P series) */
    .model_number = {0, 2}, .model_information = {2, 4},
    .firmware_version = {6, 1}, .id = {7, 1}, .baud_rate = {8, 1},
    .return_delay_time = {9, 1}, .drive_mode = {10, 1},
    .operating_mode = {11, 1}, .secondary_id = {12, 1},
    .homing_offset = {20, 4}, .moving_threshold = {24, 4},
    .temperature_limit = {31, 1}, .max_voltage_limit = {32, 2},
    .min_voltage_limit = {34, 2}, .pwm_limit = {36, 2},
    .current_limit = {38, 2}, .acceleration_limit = {40, 4},
    .velocity_limit = {44, 4}, .max_position_limit = {48, 4},
    .min_position_limit = {52, 4}, .external_port_mode_1 = {56, 1},
    .external_port_mode_2 = {57, 1}, .external_port_mode_3 = {58, 1},
    .external_port_mode_4 = {59, 1}, .shutdown = {63, 1},
    .torque_enable = {512, 1}, .led_red = {513, 1},
    .led_green = {514, 1}, .led_blue = {515, 1},
    .status_return_level = {516, 1}, .registered_instruction = {517, 1},
    .hardware_error_status = {518, 1}, .velocity_i_gain = {524, 2},
    .velocity_p_gain = {526, 2}, .position_d_gain = {528, 2},
    .position_i_gain = {530, 2}, .position_p_gain = {532, 2},
    .feedforward_2nd_gain = {536, 2}, .feedforward_1st_gain = {538, 2},
    .bus_watchdog = {546, 1}, .goal_pwm = {548, 2},
    .goal_current = {550, 2}, .goal_velocity = {552, 4},
    .profile_acceleration = {556, 4}, .profile_velocity = {560, 4},
    .goal_position = {564, 4}, .realtime_tick = {568, 2},
    .moving = {570, 1}, .moving_status = {571, 1},
    .present_pwm = {572, 2}, .present_current = {574, 2},
    .present_velocity = {576, 4}, .present_position = {580, 4},
    .velocity_trajectory = {584, 4}, .position_trajectory = {588, 4},
    .present_input_voltage = {592, 2}, .present_temperature = {594, 1},
    .external_port_data_1 = {600, 2}, .external_port_data_2 = {602, 2},
    .external_port_data_3 = {604, 2}, .external_port_data_4 = {606, 2} };

const dxl_regs_t DXL_Y = {
    .model_number = {0, 2}, .model_information = {2, 4},
    .firmware_version = {6, 1}, .id = {7, 1}, .bus_watchdog = {8, 2},
    .secondary_id = {10, 1}, .baud_rate = {12, 1},
    .return_delay_time = {13, 1}, .status_return_level = {15, 1},
    .registered_instruction = {16, 1}, .drive_mode = {32, 1},
    .operating_mode = {33, 1}, .startup_configuration = {34, 1},
    .position_limit_threshold = {38, 2}, .in_position_threshold = {40, 4},
    .following_error_threshold = {44, 4}, .moving_threshold = {48, 4},
    .homing_offset = {52, 4}, .inverter_temperature_limit = {56, 1},
    .motor_temperature_limit = {57, 1}, .max_voltage_limit = {60, 2},
    .min_voltage_limit = {62, 2}, .pwm_limit = {64, 2},
    .current_limit = {66, 2}, .acceleration_limit = {68, 4},
    .velocity_limit = {72, 4}, .max_position_limit = {76, 4},
    .min_position_limit = {84, 4}, .electronic_gear_ratio_numerator = {96, 4},
    .electronic_gear_ratio_denominator = {100, 4}, .safe_stop_time = {104, 2},
    .brake_delay = {106, 2}, .goal_update_delay = {108, 2},
    .overexcitation_voltage = {110, 1}, .normal_excitation_voltage = {111, 1},
    .overexcitation_time = {112, 2}, .present_velocity_lpf_frequency = {132, 2},
    .goal_current_lpf_frequency = {134, 2}, .position_ff_lpf_time = {136, 2},
    .velocity_ff_lpf_time = {138, 2}, .controller_state = {152, 1},
    .error_code = {153, 1}, .error_code_history1 = {154, 1},
    .hybrid_save = {170, 1}, .velocity_i_gain = {212, 4},
    .velocity_p_gain = {216, 4}, .feedforward_2nd_gain = {220, 4},
    .position_d_gain = {224, 4}, .position_i_gain = {228, 4},
    .position_p_gain = {232, 4}, .feedforward_1st_gain = {236, 4},
    .profile_acceleration = {240, 4}, .profile_velocity = {244, 4},
    .profile_acceleration_time = {248, 4}, .profile_time = {252, 4},
    .torque_enable = {512, 1}, .led = {513, 1}, .pwm_offset = {516, 2},
    .current_offset = {518, 2}, .velocity_offset = {520, 4},
    .goal_pwm = {524, 2}, .goal_current = {526, 2},
    .goal_velocity = {528, 4}, .goal_position = {532, 4},
    .moving_status = {541, 1}, .realtime_tick = {542, 2},
    .present_pwm = {544, 2}, .present_current = {546, 2},
    .present_velocity = {548, 4}, .present_position = {552, 4},
    .position_trajectory = {560, 4}, .velocity_trajectory = {564, 4},
    .present_input_voltage = {568, 2}, .present_temperature = {570, 1},
    .present_motor_temperature = {571, 1} };

const dxl_regs_t *dxl_regs(uint16_t model) {
    switch (model) {
    case 1060: case 1070: case 1080: case 1090: case 1160: /* XL430, XC430, 2XL/2XC430 */
        return &DXL_XL430;
    case 1190: case 1200: case 1210: case 1220: case 1230: case 1240: /* XL330, XC330 */
    case 1710:                                                         /* XM335 */
    case 1020: case 1030: case 1040: case 1050: case 1000: case 1010: /* XM430, XH430 */
    case 1001: case 1011:                                              /* XD430 */
    case 1170: case 1180: case 1270: case 1280:                        /* XW540, XW430 */
        return &DXL_XM430;
    case 1120: case 1130: case 1100: case 1110: case 1140: case 1150: /* XM540, XH540 */
    case 1101: case 1111:                                              /* XD540 */
        return &DXL_XM540;
    case 30: return &DXL_MX28_2;
    case 311: case 321: return &DXL_MX64_2;
    case 350: return &DXL_XL320;
    case 43288: case 46096: case 46352: case 51200: case 53768: case 54024: /* PRO (R) */
        return &DXL_PRO;
    case 43289: case 46097: case 46353: case 51201: case 53769: case 54025: /* PRO (RA) */
    case 2000: case 2010: case 2020: case 2100: case 2110: case 2120:       /* PRO+ (P) */
        return &DXL_P;
    case 4000: case 4010: case 4020: case 4030: case 4040: case 4050:       /* YM070 */
    case 4120: case 4130: case 4140: case 4150: case 4160: case 4170:       /* YM080 */
        return &DXL_Y;
    default: return NULL;
    }
}
