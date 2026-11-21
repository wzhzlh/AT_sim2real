#include "leg_driver/leg_driver.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <iostream>


// constexpr uint16_t kLegDriverVendorId = 0x0483;
// constexpr uint16_t kLegDriverProductId = 0x5740;


LegDriver::LegDriver(uint16_t vid, uint16_t pid) {
    cdc_trans = std::make_unique<CDCTrans>();
    cdc_trans->regeiser_recv_cb([this](const uint8_t* data, int size) {
        if (data == nullptr || size < static_cast<int>(sizeof(int))) {
            return;
        }

        std::lock_guard<std::mutex> lock(state_mutex_);

        int pack_type = -1;
        std::memcpy(&pack_type, data, sizeof(pack_type));

        const bool is_pack3 = pack_type == 3 && size == static_cast<int>(sizeof(DogStatePack3_t));
        const bool is_pack0 = pack_type == 0 && size == static_cast<int>(sizeof(DogStatePack0_t));

        if ((!is_pack3) && (!is_pack0))
            return;

        LegState_t* legs_state;

        if (is_pack3) {
            legs_state = state_pack.pack3.leg;
            std::memcpy(&state_pack.pack3, data, size);
            this->watch_dog_  = state_pack.pack3.watch_dog;
            this->motor_error_ = state_pack.pack3.motor_error;
            this->last_time   = state_pack.pack3.time;
        }
        else if (is_pack0) {
            legs_state = state_pack.pack0.leg;
            std::memcpy(&state_pack.pack0, data, size);
            // TODO:复制IMU信息
            imu         = state_pack.pack0.imu;
            imu_updated = true;
            this->watch_dog_   = 0;
            this->motor_error_ = state_pack.pack0.motor_state;

            this->last_time = state_pack.pack0.time;
        }

        const bool need_reset_filter = first_update;

        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 3; ++j) {
                position[i][j]       = legs_state[i].joint[j].rad;
                filtered_omega[i][j] = legs_state[i].joint[j].omega;

                if (need_reset_filter) {
                    torque_filters[i][j].reset(legs_state[i].joint[j].torque);
                    filtered_torque[i][j] = legs_state[i].joint[j].torque;
                } else {
                    filtered_torque[i][j] = torque_filters[i][j].update(legs_state[i].joint[j].torque);
                }
            }

            const WheelState_t& wheel_state = is_pack3 ? state_pack.pack3.wheel[i] : legs_state[i].wheel;
            wheel_rad[i]            = wheel_state.rad;
            filtered_wheel_omega[i] = wheel_state.omega;
            if (need_reset_filter) {
                wheel_torque_filters[i].reset(wheel_state.torque);
                filtered_wheel_torque[i] = wheel_state.torque;
            } else {
                filtered_wheel_torque[i] = wheel_torque_filters[i].update(wheel_state.torque);
            }
        }

        motor_has_error = (this->watch_dog_ != 0) || (this->motor_error_ != 0);
        uint16_t motor_error_code = static_cast<uint16_t>(this->watch_dog_ | this->motor_error_);

        if (motor_has_error) {
            std::cout << "电机异常: watch_dog=" << this->watch_dog_
                      << ", motor_error=" << this->motor_error_ << std::endl;
        }

        auto motor_error_callback = motor_error_callback_;



        first_update = false;

        if (motor_has_error && motor_error_callback) {
            motor_error_callback(motor_error_code);
        }

        // std::cout<<"joint1_pos="<<position[0][0]<<std::endl;
    });

    if (!cdc_trans->open(vid, pid)) {
        exit_thread = true;
        std::cerr << "[LegDriver] Failed to open USB CDC device " << std::hex << vid << ":" << pid << std::dec << std::endl;
        return;
    }

    usb_event_handle_thread = std::make_unique<std::thread>([this]() {
        while (!exit_thread) {
            cdc_trans->process_once();
        }
    });
}

bool LegDriver::get_imu_state(std::array<float,4> &q,std::array<float,3> &w) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!imu_updated)
        return false;
    q[0]=imu.w;
    q[1]=imu.x;
    q[2]=imu.y;
    q[3]=imu.z;

    w[0]=imu.wx;
    w[1]=imu.wy;
    w[2]=imu.wz;
    return true;
}

void LegDriver::set_motor_error_callback(std::function<void(uint16_t)> callback) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    motor_error_callback_ = std::move(callback);
}

uint16_t LegDriver::get_watch_dog() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return watch_dog_;
}

uint16_t LegDriver::get_motor_error() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return motor_error_;
}

LegDriver::~LegDriver() {
    exit_thread = true;

    if (usb_event_handle_thread && usb_event_handle_thread->joinable()) {
        usb_event_handle_thread->join();
    }

    if (cdc_trans) {
        cdc_trans->close();
    }
}

bool LegDriver::get_leg_state(std::array<LegState_t, 4>& legs_state, uint32_t& last_time) {
    const bool has_state = get_leg_state(legs_state);
    last_time = this->last_time;
    return has_state;
}

bool LegDriver::set_leg_target(const std::array<LegTarget_t, 4>& legs_target, uint32_t time) {
    if (!cdc_trans) {
        return false;
    }

    DogTargetPack_t target_pack{};
    target_pack.pack_type = 4;
    target_pack.time      = time;

    if (enable_control_) { // 正常模式
        for (int i = 0; i < 4; i++)
            target_pack.leg[i] = legs_target[i];
    } else {               // 安全阻尼模式
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 3; j++) {
                target_pack.leg[i].joint[j].rad    = 0.0f;
                target_pack.leg[i].joint[j].omega  = 0.0f;
                target_pack.leg[i].joint[j].torque = 0.0f;
                target_pack.leg[i].joint[j].kp     = 0.0f;
                target_pack.leg[i].joint[j].kd     = joint_default_kd;
            }
            target_pack.leg[i].wheel.omega = 0.0f;
            target_pack.leg[i].wheel.kd    = wheel_default_kd;
        }
    }
    return cdc_trans->send_struct(target_pack);
}

bool LegDriver::get_leg_state(std::array<LegState_t, 4>& legs_state) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (first_update)      // 完成首次更新后再去给电机提供数值
        return false;

    if (motor_has_error)
        return false;

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 3; ++j) {
            legs_state[i].joint[j].rad    = position[i][j];
            legs_state[i].joint[j].omega  = filtered_omega[i][j];
            legs_state[i].joint[j].torque = filtered_torque[i][j];
        }
        legs_state[i].wheel.rad    = wheel_rad[i];
        legs_state[i].wheel.omega  = filtered_wheel_omega[i];
        legs_state[i].wheel.torque = filtered_wheel_torque[i];
    }
    return true;
}

bool LegDriver::enable_control(bool cmd) {
    enable_control_ = cmd;
    first_update    = true;
    return enable_control_;
}
