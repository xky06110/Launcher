#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args:
  - task_stack_depth: 2048
  - cmd: '@&cmd'
  - pid_param_trig_angle:
      k: 1.0
      p: 35.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 10.0
      cycle: false
  - pid_param_trig_speed:
      k: 1.0
      p: 0.65
      i: 0.8
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - pid_param_fric_0:
      k: 1.0
      p: 0.003
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - pid_param_fric_1:
      k: 1.0
      p: 0.003
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - motor_fric_0: '@&motor_fric_0'
  - motor_fric_1: '@&motor_fric_1'
  - motor_trig: '@&motor_trig'
  - launcher_param:
      fric_rpm: 5000
      trig_gear_ratio: 36.0
      num_trig_tooth: 10
      expect_trig_freq_: 10.0
template_args:
required_hardware:
  - dr16
  - can
depends:
  - qdu-future/CMD
  - qdu-future/RMMotor
=== END MANIFEST === */
// clang-format on

#include <cmath>
#include <cstdint>
#include <initializer_list>
#include "Motor.hpp"
#include "CMD.hpp"
#include "RMMotor.hpp"
#include "app_framework.hpp"
#include "cycle_value.hpp"
#include "event.hpp"
#include "libxr_cb.hpp"
#include "message.hpp"
#include "mutex.hpp"
#include "pid.hpp"
#include "timebase.hpp"
#include "timer.hpp"

namespace launcher::param {
constexpr float TRIGSTEP = static_cast<float>(M_2PI) / 10;
constexpr float MAX_FRIC_CUR = 2.0f;
constexpr float MIN_FRIC_RPM = 200.0f;
}  // namespace launcher::param

class Launcher {
 public:
  enum class TRIGMODE : uint8_t { RELAX, SAFE, SINGLE, CONTINUE, JAM };

  enum class LauncherEvent : uint8_t {
    SET_FRICMODE_RELAX,
    SET_FRICMODE_SAFE,
    SET_FRICMODE_READY,
  };

  struct LauncherParam {
    float fric_rpm_;
    float trig_gear_ratio;
    uint8_t num_trig_tooth;
    float trig_freq_;
  };
  /**
   * @brief Launcher 构造函数
   *
   * @param hw 硬件容器
   * @param app 应用管理器
   * @param task_stack_depth 任务堆栈深度
   * @param pid_param_trig 拨弹盘PID参数
   * @param pid_param_fric 摩擦轮PID参数
   * @param  fric_radius_ 摩擦轮半径
   * @param trig_gear_ratio_ 拨弹电机减速比
   * @param  num_trig_tooth_ 拨弹盘中一圈能存储几颗弹丸
   */
  Launcher(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app,
           uint32_t task_stack_depth, CMD *cmd,
           LibXR::PID<float>::Param pid_param_trig_angle,
           LibXR::PID<float>::Param pid_param_trig_speed,
           LibXR::PID<float>::Param pid_param_fric_0,
           LibXR::PID<float>::Param pid_param_fric_1,
           RMMotor *motor_fric_front_left, RMMotor *motor_fric_front_right,
           RMMotor *motor_trig, LauncherParam launch_param)
      : param_(launch_param),
        motor_fric_0_(motor_fric_front_left),
        motor_fric_1_(motor_fric_front_right),
        motor_trig_(motor_trig),
        pid_fric_0_(pid_param_fric_0),
        pid_fric_1_(pid_param_fric_1),
        pid_trig_sp_(pid_param_trig_speed),
        pid_trig_angle_(pid_param_trig_angle),
        cmd_(cmd) {
    UNUSED(hw);
    UNUSED(app);

    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Launcher *launcher, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          launcher->SetMode(static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_RELAX));
          launcher->trig_mod_ = TRIGMODE::RELAX;
        },
        this);

    cmd_->GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);
    auto callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Launcher *launcher, uint32_t event_id) {
          UNUSED(in_isr);
          launcher->SetMode(event_id);
        },
        this);

    launcher_event_.Register(static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_RELAX), callback);

    launcher_event_.Register(static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_SAFE), callback);

    launcher_event_.Register(static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_READY), callback);

    auto launcher_cmd_callback = LibXR::Callback<LibXR::RawData &>::Create(
        [](bool in_isr, Launcher *Launcher, LibXR::RawData &raw_data) {
          UNUSED(in_isr);
          CMD::LauncherCMD cmd_lau =
              *reinterpret_cast<CMD::LauncherCMD *>(raw_data.addr_);
          Launcher->launcher_cmd_.isfire = cmd_lau.isfire;
        },
        this);

    auto tp_cmd_launcher =
        LibXR::Topic(LibXR::Topic::Find("launcher_cmd", nullptr));

    tp_cmd_launcher.RegisterCallback(launcher_cmd_callback);
    thread_.Create(this, ThreadFunction, "LauncherThread", task_stack_depth,
                   LibXR::Thread::Priority::MEDIUM);
  }

  static void ThreadFunction(Launcher *launcher) {
    auto now = LibXR::Timebase::GetMilliseconds();
    launcher->dt_ = (now - launcher->last_online_time_).ToSecondf();
    launcher->last_online_time_ = now;

    while (1) {
      launcher->Update();
      launcher->FricControl();
      launcher->SetTrig();

      LibXR::Thread::Sleep(2);
    }
  }
  /**
   * @brief 更新函数
   *
   */
  void Update() {
    motor_fric_0_->Update();
    motor_fric_1_->Update();
    motor_trig_->Update();

    param_frirc_0_ = motor_fric_0_->GetFeedback();
    param_frirc_1_ = motor_fric_1_->GetFeedback();
    param_trig_ = motor_trig_->GetFeedback();

    static float last_motor_angle = 0.0f;
    static bool initialized = false;

    float current_motor_angle = param_trig_.position;

    if (!initialized) {
      last_motor_angle = current_motor_angle;
      initialized = true;
      return;
    }

    float delta_trig_angle = LibXR::CycleValue<float>(current_motor_angle) -
                             LibXR::CycleValue<float>(last_motor_angle);

    trig_angle_ += delta_trig_angle / param_.trig_gear_ratio;
    last_motor_angle = current_motor_angle;

  }
  void FricControl() {
    switch (fric_event_) {
      case (LauncherEvent::SET_FRICMODE_RELAX): {
        motor_fric_0_->Relax();
        motor_fric_1_->Relax();
      } break;
      case LauncherEvent::SET_FRICMODE_SAFE: {
        float out_rpm_0 = SoftTransition(0, param_frirc_0_.velocity);
        float out_rpm_1 = SoftTransition(0, param_frirc_1_.velocity);
        if (param_frirc_0_.velocity < 200 || param_frirc_1_.velocity < 200) {
          out_rpm_0 = 0;
          out_rpm_1 = 0;
        }
        fric_out_left_ =
            pid_fric_0_.Calculate(out_rpm_0, param_frirc_0_.velocity, dt_);
        fric_out_right_ =
            pid_fric_1_.Calculate(out_rpm_1, param_frirc_1_.velocity, dt_);
        motor_fric_0_->Control(cmd_fric_0_);
        motor_fric_1_->Control(cmd_fric_1_);
      } break;
      case LauncherEvent::SET_FRICMODE_READY: {
        cmd_fric_0_.velocity = param_.fric_rpm_;
        cmd_fric_1_.velocity = param_.fric_rpm_;

        fric_out_left_ =
            pid_fric_0_.Calculate(out_rpm_0_, param_frirc_0_.velocity, dt_);
        fric_out_right_ =
            pid_fric_1_.Calculate(out_rpm_1_, param_frirc_1_.velocity, dt_);
        motor_fric_0_->Control(cmd_fric_0_);
        motor_fric_1_->Control(cmd_fric_1_);
      }
      default:
        break;
    }
  }

  void SetTrig() {
    auto now = LibXR::Timebase::GetMilliseconds();

    switch (trig_mod_) {
      case TRIGMODE::RELAX:
        motor_trig_->Relax();
        break;
      case TRIGMODE::SAFE:
        cmd_trig_.position = trig_angle_;

        break;
      case TRIGMODE::SINGLE: {
        if (last_trig_mod_ == TRIGMODE::SAFE) {
          cmd_trig_.position += launcher::param::TRIGSTEP;
        }
        last_trig_angle_ = cmd_trig_.position;
      } break;

      case TRIGMODE::CONTINUE: {
        float since_last = (now - last_trig_time_).ToSecondf();
        if (param_.trig_freq_ > 0.0f) {
          float trig_speed = 1.0f / param_.trig_freq_;
          if (since_last >= trig_speed) {
            cmd_trig_.position = launcher::param::TRIGSTEP;
            last_trig_time_ = now;
          }
        }

        last_trig_angle_ = cmd_trig_.position;
      } break;

      default:
        break;
    }
    float plate_omega_ref = pid_trig_angle_.Calculate(
        cmd_trig_.position, trig_angle_,
        param_trig_.omega / param_.trig_gear_ratio, dt_);
    float motor_omega_ref = std::clamp(
        plate_omega_ref,
        static_cast<float>(-1.5 * M_2PI * param_.trig_freq_ / param_.num_trig_tooth),
        static_cast<float>(1.5 * M_2PI *param_. trig_freq_ / param_.num_trig_tooth));
    cmd_trig_.velocity = pid_trig_sp_.Calculate(
        motor_omega_ref, param_trig_.omega / param_.trig_gear_ratio, dt_);

    motor_trig_->Control(cmd_trig_);

    last_trig_mod_ = trig_mod_;
  }
  void SetMode(uint32_t mode) {
    mutex_.Lock();
    pid_fric_0_.Reset();
    pid_fric_1_.Reset();
    pid_trig_angle_.Reset();
    pid_trig_sp_.Reset();
    mutex_.Unlock();

    switch (mode) {
      case 0:
        fric_event_= LauncherEvent::SET_FRICMODE_RELAX;
        trig_mod_ = TRIGMODE::SAFE;
        break;
      case 1:
      fric_event_ = LauncherEvent::SET_FRICMODE_RELAX;
        trig_mod_ = TRIGMODE::SINGLE;
        break;
      case 2:
      fric_event_ = LauncherEvent::SET_FRICMODE_READY;
        trig_mod_ = TRIGMODE::CONTINUE;
        break;
      default:
        break;

    }
  }
  float SoftTransition(float target, float cur) {
    constexpr float TAU = 0.15f;
    float alpha = dt_ / (TAU + dt_);
    return cur + alpha * (target - cur);
  }

  LibXR::Event &GetEvent() { return launcher_event_; }

 private:
  LauncherParam param_;
  TRIGMODE last_trig_mod_ = TRIGMODE::RELAX;
  TRIGMODE trig_mod_ = TRIGMODE::RELAX;

  float trig_angle_ = 0.0f;
  float last_trig_angle_ = 0.0f;
  LibXR::MillisecondTimestamp last_trig_time_ = 0;

  RMMotor *motor_fric_0_;
  RMMotor *motor_fric_1_;
  RMMotor *motor_trig_;

  LibXR::PID<float> pid_fric_0_;
  LibXR::PID<float> pid_fric_1_;
  LibXR::PID<float> pid_trig_sp_;
  LibXR::PID<float> pid_trig_angle_;
  CMD::LauncherCMD launcher_cmd_;

  CMD *cmd_;
  LibXR::MillisecondTimestamp last_online_time_ = 0.0f;
  float dt_ = 0;
  LibXR::MillisecondTimestamp last_jam_time_ = 0.0f;
  LibXR::MillisecondTimestamp jam_keep_time_ = 0.0f;
  LibXR::MillisecondTimestamp fire_press_time_ = 0;
  LibXR::MillisecondTimestamp last_heat_time_ = 0.0f;
  LibXR::MillisecondTimestamp last_check_time_ = 0.0f;
  LibXR::Thread thread_;
  LibXR::Semaphore semaphore_;
  LibXR::Mutex mutex_;
  LibXR::Event launcher_event_;

  float fric_out_left_ = 0.0f;
  float fric_out_right_ = 0.0f;
  float out_rpm_0_ = 0;
  float out_rpm_1_ = 0;
  Motor::Feedback param_frirc_0_;
  Motor::Feedback param_frirc_1_;
  Motor::Feedback param_trig_;
  Motor::MotorCmd cmd_fric_0_ =
      Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                      .reduction_ratio = 19.0f,
                      .velocity = 0};
  Motor::MotorCmd cmd_fric_1_ =
      Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                      .reduction_ratio = 19.0f,
                      .velocity = 0};
  Motor::MotorCmd cmd_trig_ =
      Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                      .reduction_ratio = 36.0f,
                      .velocity = 0};
  LauncherEvent fric_event_ = LauncherEvent::SET_FRICMODE_RELAX;

  /*指数缓变*/
};
