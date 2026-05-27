#ifndef CC_DECISION_DEF_H_
#define CC_DECISION_DEF_H_

namespace cc {
namespace decision {

/**
 * @brief 驾驶员按键/踏板指令 Im (表 1)
 */
enum DriverCommand {
    COMMAND_NONE = 0,   // 无指令
    COMMAND_INC = 1,    // I0: 增速键
    COMMAND_DEC = 2,    // I1: 降速键
    COMMAND_CANCEL = 3, // I2: 取消键
    COMMAND_BRAKE = 4,  // I3: 刹车踏板踩下
    COMMAND_EXIT = 5    // I4: 退出键
};

/**
 * @brief 车速子态 S_vi (表 2)
 */
enum SpeedSubstate {
    SPEED_SUBSTATE_SUITABLE = 0, // Sv0: 适速
    SPEED_SUBSTATE_LOW = 1,      // Sv1: 低速
    SPEED_SUBSTATE_HIGH = 2      // Sv2: 高速
};

/**
 * @brief 系统子态集 S_sj (表 3)
 * 通过 j = 4 * b2 + 2 * b1 + b0 编码得到，j 范围 0 ~ 7
 */
enum SystemSubstate {
    SYS_SUBSTATE_S0 = 0, // Ss0: 待命且无油无史 (000)
    SYS_SUBSTATE_S1 = 1, // Ss1: 待命且无油有史 (001)
    SYS_SUBSTATE_S2 = 2, // Ss2: 待命且有油无史 (010)
    SYS_SUBSTATE_S3 = 3, // Ss3: 待命且有油有史 (011)
    SYS_SUBSTATE_S4 = 4, // Ss4: 在控且无油无史 (100)
    SYS_SUBSTATE_S5 = 5, // Ss5: 在控且无油有史 (101)
    SYS_SUBSTATE_S6 = 6, // Ss6: 在控且有油无史 (110)
    SYS_SUBSTATE_S7 = 7  // Ss7: 在控且有油有史 (111)
};

/**
 * @brief 系统状态 Sn (表 3)
 */
enum SystemState {
    SYS_STATE_S0 = 0, // S0: 适速在控
    SYS_STATE_S1 = 1, // S1: 高速在控
    SYS_STATE_S2 = 2, // S2: 适速无油有史待命
    SYS_STATE_S3 = 3, // S3: 适速无油无史待命
    SYS_STATE_S4 = 4, // S4: 有油待命
    SYS_STATE_S5 = 5, // S5: 非适速待命
    SYS_STATE_S6 = 6  // S6: 低速在控
};

/**
 * @brief 系统决策输出结果 Rk (表 4)
 */
enum DecisionResult {
    RESULT_NONE = 0, // 无动作
    RESULT_R1 = 1,   // R1: 目标增量 (定量增目标速)
    RESULT_R2 = 2,   // R2: 目标减量 (定量减目标速)
    RESULT_R3 = 3,   // R3: 保持控制
    RESULT_R4 = 4,   // R4: 继承控制 (Resume)
    RESULT_R5 = 5,   // R5: 无继控制 (Set)
    RESULT_R6 = 6,   // R6: 系统待命
    RESULT_R7 = 7,   // R7: 报错待命 (车速异常，切断动力)
    RESULT_R8 = 8    // R8: 系统退出 (关闭巡航)
};

/**
 * @brief 决策配置参数
 */
struct DecisionConfig {
    double minSpeed;         // 最低适控车速 (m/s)
    double maxSpeed;         // 最高适控车速 (m/s)
    double lowSpeedTimeout;  // S6 状态下最低车速持续时间上限 (s)，超时将主动退出
    double deltaSpeedStep;   // 增减速单次修改的步长 (m/s)
};

/**
 * @brief 决策周期输入信号
 */
struct DecisionInput {
    double currentSpeed;       // 当前车辆实际速度 (m/s)
    bool isBrakePressed;       // 刹车是否踩下
    bool isThrottlePressed;    // 油门是否踩下
    bool hasHistoryTarget;     // 是否有历史目标速度 (决定待命状态下能否 Resume)
    bool controlActive;        // 当前控制环是否处于激活（在控）状态
    DriverCommand driverCommand; // 当前周期的按键指令
    double dt;                 // 周期步长 (s)
};

/**
 * @brief 决策状态变量 (持久化状态)
 */
struct DecisionState {
    SystemState currentState; // 当前系统运行状态
    double targetSpeed;       // 当前目标设定车速 (m/s)
    double lastTargetSpeed;   // 上一次有效的历史目标车速 (m/s)
    double lowSpeedTimer;     // 进入 S6（低速在控）后的持续计时器 (s)
};

/**
 * @brief 决策单步输出结果
 */
struct DecisionOutput {
    DecisionResult decision;     // 决策执行结果 Rk
    SystemState nextState;       // 系统流转后的新状态
    double updatedTargetSpeed;   // 更新后的设定目标车速
};

} // namespace decision
} // namespace cc

#endif // CC_DECISION_DEF_H_
