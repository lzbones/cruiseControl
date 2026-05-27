#include <iostream>
#include <iomanip>
#include <vector>
#include <cassert>
#include <fstream>
#include <string>

#include "math/math_utils.h"
#include "decision/cc_decision_def.h"
#include "decision/cc_process_command.h"
#include "control/cc_control_def.h"
#include "control/cc_sppvt_step.h"

// 车辆物理模型常量
const double VEHICLE_MASS = 150.0;     // 质量 (kg)
const double WHEEL_RADIUS = 0.3;      // 车轮半径 (m)
const double G_CONSTANT = 9.8;        // 重力加速度 (m/s^2)
const double CR_ROLL = 0.015;         // 滚动阻力系数
const double CD_AERO = 0.15;          // 空气阻力系数 (0.5 * Cd * A * rho)

/**
 * @brief 仿真被控车辆动力学
 * @param speed 当前实际车速 (m/s)
 * @param torqueWheel 作用在车轮上的驱动扭矩 (Nm)
 * @param slopeAngle 道路坡度角度 (度)
 * @param dt 仿真时间步长 (s)
 * @return 下一时刻的实际车速 (m/s)
 */
double simulateVehiclePhysics(double speed, double torqueWheel, double slopeAngle, double dt) {
    // 1. 驱动力 (F_drive)
    double fDrive = torqueWheel / WHEEL_RADIUS;

    // 2. 滚动阻力 (F_roll)
    double fRoll = VEHICLE_MASS * G_CONSTANT * CR_ROLL;

    // 3. 空气阻力 (F_aero)
    double fAero = CD_AERO * speed * speed;

    // 4. 坡度阻力 (F_slope = m * g * sin(theta))
    // 将坡度角度换算为弧度
    double rad = slopeAngle * 3.14159265 / 180.0;
    double fSlope = VEHICLE_MASS * G_CONSTANT * sin(rad);

    // 5. 合力与加速度
    double fTotal = fDrive - (fRoll + fAero + fSlope);
    double acceleration = fTotal / VEHICLE_MASS;

    // 6. 欧拉法更新车速 (限幅不小于 0)
    double nextSpeed = speed + acceleration * dt;
    if (nextSpeed < 0.0) {
        nextSpeed = 0.0;
    }
    return nextSpeed;
}

int main() {
    std::cout << "========= 开始定速巡航（CC）系统联合仿真测试 =========" << std::endl;

    // 1. 初始化决策与控制器配置
    cc::decision::DecisionConfig decConfig;
    decConfig.minSpeed = 5.0;            // 18 km/h
    decConfig.maxSpeed = 25.0;           // 90 km/h
    decConfig.lowSpeedTimeout = 3.0;     // 3秒低速超时保护
    decConfig.deltaSpeedStep = 1.0;      // 增减速步长 1.0m/s

    cc::control::SppvtConfig ctrlConfig;
    ctrlConfig.Kp = 45.0;                // 比例系数
    ctrlConfig.rho = 0.35;               // 惩罚系数 (收敛速度)
    ctrlConfig.delta = 0.03;             // 判定为平稳的加速度上限 (m/s^2)
    ctrlConfig.maxOffset = 4.0;          // 虚拟目标车速最大偏置 (m/s)
    ctrlConfig.cooldownTime = 2.0;       // 触发更新后冷却 2s
    ctrlConfig.minTorque = 0.0;
    ctrlConfig.maxTorque = 150.0;        // 电机最大额定扭矩

    // 2. 初始化持久化运行状态
    cc::decision::DecisionState decState;
    decState.currentState = cc::decision::SYS_STATE_S3; // 初始为适速无油无史待命
    decState.targetSpeed = 15.0;        // 预设设定目标速度
    decState.lastTargetSpeed = 15.0;    // 预设历史目标速度，以便激活时可以通过 Resume 恢复至 15.0 m/s
    decState.lowSpeedTimer = 0.0;

    cc::control::SppvtState ctrlState;
    ctrlState.virtualTargetSpeed = 15.0;
    ctrlState.stepIndex = 1;
    ctrlState.cooldownTimer = 0.0;
    ctrlState.controlActive = false;

    // 3. 运行环境状态
    double currentSpeed = 12.0;         // 初始车速 (m/s)
    double lastSpeed = 12.0;
    double currentAccel = 0.0;
    double currentTorque = 0.0;
    double slopeAngle = 0.0;            // 初始平路
    bool controlActive = false;
    bool isNewSession = false;

    // 仿真参数
    double t = 0.0;
    double dt = 0.05;                   // 仿真步长 (50ms，对应 20Hz 环)
    int steps = 600;                    // 总仿真步数：30 秒

    // 打开 CSV 文件准备写入高频仿真数据
    std::ofstream csvFile("output/simulation_results.csv");
    if (csvFile.is_open()) {
        csvFile << "Time,Speed,TargetSpeed,VirtualTargetSpeed,StepIndex,Torque,Slope,State\n";
    }

    std::cout << std::left 
              << std::setw(8) << "时间(s)" 
              << std::setw(10) << "车速(m/s)" 
              << std::setw(12) << "设定目标(m/s)" 
              << std::setw(12) << "虚拟目标(m/s)" 
              << std::setw(10) << "阶数(i)" 
              << std::setw(10) << "输出扭矩(Nm)" 
              << std::setw(10) << "坡度(度)" 
              << "系统状态" << std::endl;
    std::cout << "---------------------------------------------------------------------------------------" << std::endl;

    for (int i = 0; i <= steps; ++i) {
        t = i * dt;

        // 模拟外部驾驶员按键输入行为
        cc::decision::DriverCommand cmd = cc::decision::COMMAND_NONE;
        
        // 在 t = 1.0s 时，驾驶员按下增速键，相当于触发巡航控制 Set (R5)
        if (i == 20) { // 1.0s
            cmd = cc::decision::COMMAND_INC;
        }

        // 4. 执行决策业务库处理
        cc::decision::DecisionInput decInput;
        decInput.currentSpeed = currentSpeed;
        decInput.isBrakePressed = false;
        decInput.isThrottlePressed = false;
        decInput.hasHistoryTarget = (decState.lastTargetSpeed > 0.0);
        decInput.controlActive = controlActive;
        decInput.driverCommand = cmd;
        decInput.dt = dt;

        cc::decision::DecisionOutput decOutput = cc::decision::processCommand(decInput, decConfig, decState);

        // 如果决策输出要求进入/改变控制状态，进行同步
        if (decOutput.decision == cc::decision::RESULT_R4 || decOutput.decision == cc::decision::RESULT_R5) {
            controlActive = true;
            isNewSession = true; // 标志为新控制会话，重置 SPPVT 累加器
        } else if (decOutput.decision == cc::decision::RESULT_R6 || decOutput.decision == cc::decision::RESULT_R7) {
            controlActive = false;
        }

        // 5. 计算加速度（共性数学库）
        currentAccel = cc::math::calcAcceleration(currentSpeed, lastSpeed, dt);
        lastSpeed = currentSpeed;

        // 6. 执行控制业务库处理
        cc::control::SppvtInput ctrlInput;
        ctrlInput.currentSpeed = currentSpeed;
        ctrlInput.targetSpeed = decState.targetSpeed;
        ctrlInput.currentAcceleration = currentAccel;
        ctrlInput.dt = dt;
        ctrlInput.pauseControl = !controlActive;
        ctrlInput.isNewControlSession = isNewSession;

        // 一旦输入了一次重置信号，消费后立刻清零
        isNewSession = false;

        cc::control::SppvtOutput ctrlOutput = cc::control::sppvtStep(ctrlConfig, ctrlInput, ctrlState);
        currentTorque = ctrlOutput.torqueCommand;

        // 7. 车辆物理仿真更新
        // 在 t = 12.0s 时引入突发的 5 度上坡阻力
        if (t >= 12.0) {
            slopeAngle = 5.0;
        }

        currentSpeed = simulateVehiclePhysics(currentSpeed, currentTorque, slopeAngle, dt);

        // 将高频数据点写入 CSV
        if (csvFile.is_open()) {
            csvFile << t << ","
                    << currentSpeed << ","
                    << decState.targetSpeed << ","
                    << ctrlState.virtualTargetSpeed << ","
                    << ctrlState.stepIndex << ","
                    << currentTorque << ","
                    << slopeAngle << ","
                    << decState.currentState << "\n";
        }

        // 8. 每隔 1 秒打印一次数据日志
        if (i % 20 == 0) {
            std::string stateStr;
            switch (decState.currentState) {
                case cc::decision::SYS_STATE_S0: stateStr = "S0 适速在控"; break;
                case cc::decision::SYS_STATE_S1: stateStr = "S1 高速在控"; break;
                case cc::decision::SYS_STATE_S2: stateStr = "S2 适速有史待命"; break;
                case cc::decision::SYS_STATE_S3: stateStr = "S3 适速无史待命"; break;
                case cc::decision::SYS_STATE_S4: stateStr = "S4 有油待命"; break;
                case cc::decision::SYS_STATE_S5: stateStr = "S5 非适速待命"; break;
                case cc::decision::SYS_STATE_S6: stateStr = "S6 低速在控"; break;
                default: stateStr = "未知状态"; break;
            }

            std::cout << std::left << std::fixed << std::setprecision(2)
                      << std::setw(8) << t
                      << std::setw(10) << currentSpeed
                      << std::setw(12) << decState.targetSpeed
                      << std::setw(12) << ctrlState.virtualTargetSpeed
                      << std::setw(10) << ctrlState.stepIndex
                      << std::setw(10) << currentTorque
                      << std::setw(10) << slopeAngle
                      << stateStr << std::endl;
        }
    }

    if (csvFile.is_open()) {
        csvFile.close();
    }

    std::cout << "---------------------------------------------------------------------------------------" << std::endl;
    
    // 验证平路稳态车速应能逼近 15.0 m/s，且上坡阶段结束后通过 SPPVT 变目标，车速最终仍能接近 15.0 m/s
    // 经过 30 秒的离散仿真，实际速度应收敛在 15.0 +/- 0.5 范围内
    double diff = cc::math::absVal(currentSpeed - 15.0);
    std::cout << "仿真结束车速: " << currentSpeed << " m/s, 与目标偏差为: " << diff << " m/s" << std::endl;
    assert(diff < 0.5);

    std::cout << "[PASS] 联合仿真测试通过！SPPVT成功克服了上坡阻力消除静差！" << std::endl;
    return 0;
}
