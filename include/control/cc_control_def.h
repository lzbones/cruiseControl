#pragma once

namespace cc {
namespace control {

/**
 * @brief SPPVT 控制配置参数
 */
struct SppvtConfig {
    double Kp;           // 比例控制系数 (Nm * s / m)
    double rho;          // 惩罚系数 (0 < rho < 0.5)
    double delta;        // 变目标时刻加速度阈值 (m/s^2)
    double maxOffset;    // 虚拟目标允许的最大偏置量 (m/s)，即 V_max_offset
    double cooldownTime; // 变目标后的冷却锁存时长 (s)
    double maxTorque;    // 最大扭矩限幅 (Nm)
    double minTorque;    // 最小扭矩限幅 (Nm)，一般为 0.0，定速巡航不主动刹车
};

/**
 * @brief SPPVT 周期输入数据
 */
struct SppvtInput {
    double currentSpeed;        // 当前车辆实际速度 (m/s)
    double targetSpeed;         // 驾驶员设定的 CC 最终目标车速 V_t (m/s)
    double currentAcceleration; // 当前车辆加速度 v_dot (m/s^2)
    double dt;                  // 控制周期时长 (s)
    bool pauseControl;          // 是否暂停控制 (当前为待命或发生干扰时为 true)
    bool isNewControlSession;   // 是否新进入控制会话 (用于执行 i <- 1, V1 <- V_t 的重置逻辑)
};

/**
 * @brief SPPVT 持久化状态变量
 */
struct SppvtState {
    double virtualTargetSpeed;  // 内部虚拟目标速度 V_i (m/s)
    int stepIndex;              // 阶式阶数 i
    double cooldownTimer;       // 变目标冷却计时器 (s)
    bool controlActive;         // 控制器激活标志
};

/**
 * @brief SPPVT 周期输出数据
 */
struct SppvtOutput {
    double torqueCommand;        // 输出控制扭矩 T_i(t) (Nm)
    double activeVirtualTarget;  // 当前正在起作用的虚拟目标车速 V_i (m/s)
};

} // namespace control
} // namespace cc

