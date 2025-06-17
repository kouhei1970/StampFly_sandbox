#ifndef STAMPFLY_HPP
#define STAMPFLY_HPP

#include <stdint.h>
#include "pid.hpp"

//Mode
#define INIT_MODE       0
#define AVERAGE_MODE    1
#define FLIGHT_MODE     2
#define PARKING_MODE    3

//Battery
#define BATTERY_VOLTAGE (3.7)
#define POWER_LIMIT 3.34
#define UNDER_VOLTAGE_COUNT 100

//etc
#define AVERAGENUM      800

typedef struct{
    float x;
    float y;
    float z;
    float x_offset;
    float y_offset;
    float z_offset;
}accel_t;

typedef struct{
    float x;
    float y;
    float z;
    float x_offset;
    float y_offset;
    float z_offset;
}gyro_t;

typedef struct{
    float x;
    float y;
    float z;
    float x_offset;
    float y_offset;
    float z_offset;
    float x_scale;
    float y_scale;
    float z_scale;
}mag_t;

typedef struct{
    int16_t delta_x;
    int16_t delta_y;
}opt_t;

typedef struct{
    float range;
    float range_offset;
    float range_scale;
}tof_t;

typedef struct{
    float voltage;
}battery_t;

typedef struct{
    float voltage;
}pressuer_t;


typedef struct{
    float pressure;
    float voltage;
    int16_t opt_x;
    int16_t opt_y;
    uint16_t bottom_tof_range;
    uint16_t front_tof_range;
}sensor_value_t;

typedef struct{
    float accx;
    float accy;
    float accz;
    float roll_rate;
    float pitch_rate;
    float yaw_rate;
    float euler_x; // Roll angle
    float euler_y; // Pitch angle
    float euler_z; // Yaw angle
    float quat_x; // Quaternion x
    float quat_y; // Quaternion y
    float quat_z; // Quaternion z
    float quat_w; // Quaternion w
    float body_vel_x;
    float body_vel_y;
    float body_vel_z;
    float inertial_vel_x;
    float inertial_vel_y;
    float inertial_vel_z;
    float x;
    float y;
    float z;
    float altutude;
}state_t;

typedef struct{
    uint8_t mode=0;
    uint8_t oldmode=0;
    uint8_t loop=0;
}flag_t;

typedef struct{
    uint16_t loop=0;
    uint16_t offset=0;
    uint32_t counter=0;
}counter_t;

typedef struct{
    float elapsed_time;
    float old_elapsed_time;
    float interval_time;
    uint32_t start_time;
}times_t;

typedef struct{
    PID roll_rate;
    PID pitch_rate;
    PID yaw_rate;
    PID roll_angle;
    PID pitch_angle;
    PID yaw_angle;
}pidstruct_t;

typedef struct{
    state_t state;
    sensor_value_t sensor;
    flag_t flag;
    counter_t counter;
    pidstruct_t pid;
    times_t times;
}stampfly_t;

extern stampfly_t StampFly;

#endif