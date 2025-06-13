#ifndef STAMPFLY_HPP
#define STAMPFLY_HPP

#include <stdint.h>

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
    float accx;
    float accy;
    float accz;
    float roll_rate;
    float pitch_rate;
    float yaw_rate;
    float roll_angel;
    float pitch_angle;
    float yaw_angle;
    float voltage;
    uint16_t bottom_tof_range;
}sensor_value_t;

typedef struct{
    volatile uint8_t mode=0;
    volatile uint8_t oldmode=0;
    volatile uint8_t loop=0;
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
    sensor_value_t sensor;
    flag_t flag;
    counter_t counter;
    times_t times;
}stampfly_t;

extern stampfly_t StampFly;

#endif