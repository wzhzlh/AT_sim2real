#ifndef __DATAPACK_H__
#define __DATAPACK_H__

#include <cstdint>

#pragma pack(1)

typedef struct {
    float rad;
    float omega;
    float torque;
    float kp;
    float kd;
} MotorTarget_t;

typedef struct {
    float omega;
    float kd;
} WheelTarget_t;

typedef struct {
    MotorTarget_t joint[3];
    WheelTarget_t wheel;
} LegTarget_t;


typedef struct {
    struct {
        float x, y, z;
    }angular_vel;
    struct {
        float x, y, z;
    }acc;
    struct {
        float  roll,pitch,yaw;
    }angle;
} JY61_Typedef;

typedef struct{
    float wx;
    float wy;
    float wz;

    float w;
    float x;
    float y;
    float z;
}IMU_t;

typedef struct {
    float rad;
    float omega;
    float torque;
} MotorState_t;

typedef struct {
    float rad;
    float omega;
    float torque;
} WheelState_t;

typedef struct {
    MotorState_t joint[3];
    WheelState_t wheel;
} LegState_t;

typedef struct {
    float vx;
    float vy;
    float omega;
    float wheel_v;
} Remotepack_t;

// 包类型为0——全功能包
typedef struct {
    int pack_type;
    LegState_t leg[4];
    IMU_t imu;
    uint16_t motor_state;
    uint32_t time;
} DogStatePack0_t;


// 包类型为3——无IMU数据
typedef struct {
    int pack_type;
    LegState_t leg[4];
    WheelState_t wheel[4];
    uint16_t watch_dog;
    uint32_t time;
    uint16_t motor_error;
} DogStatePack3_t;

typedef struct {
    int pack_type;
    LegTarget_t leg[4];
    uint32_t time;
} DogTargetPack_t;



#pragma pack()

static_assert(sizeof(DogTargetPack_t) == 280, "DogTargetPack_t must match lower-level protocol (280 bytes)");
static_assert(sizeof(DogStatePack3_t) == 252, "DogStatePack3_t must match lower-level protocol (252 bytes)");

#endif