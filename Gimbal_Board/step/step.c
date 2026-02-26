#include "step.h"    
#include <math.h>     
#include <stdlib.h>   


// --- 硬件参数
#define STEPS_PER_REVOLUTION 200    // 电机物理参数: 电机转一圈(360度)需要的步数。1.8度步进角 = 360/1.8 = 200步。
#define MICROSTEPS 16               // 驱动器参数: 驱动器设置的细分数。细分越高，转动越平滑，但对单片机性能要求也越高。
#define PWM_CLOCK_FREQ 625000    

// --- 系统参数 ---
#define ISR_INTERVAL_S 0.010f       // 中断调用周期(秒)。您设置为10ms，所以是0.010。
                                    // 如果遇到此问题，可尝试提高中断频率到 2ms (500Hz) 或 1ms (1000Hz)，
                                    // 并相应修改此值为 0.002f 或 0.001f。

// --- 精度控制参数 (这三个参数是调优速度和精度的核心!) ---
#define FAST_SPEED_RPM 100           // 远距离巡航时的转速 (RPM - 每分钟转数)。在确保不丢步的情况下可适当调高。
#define SLOW_SPEED_RPM 6            // 近距离逼近时的转速 (RPM)。这是精度的保证！此值应设置得足够低，以消除惯性过冲。
#define DECELERATION_DEGREES 25.0f  // 提前多少度开始减速。如果电机惯性大，负载重，应适当增大此值，给足减速缓冲。

#define INITIAL_SPEED_RPM 20.0f      // 电机启动时的初始速度
#define ACCELERATION_RPM_PER_SEC 400.0f // 每秒钟增加/减少的转速 (RPM/s)，即加速度
// 全局常量：预先计算出“每转一度需要多少个脉冲”，方便后续直接使用。
// 公式: (总步数/圈 * 细分数) / 360度/圈
const float PULSES_PER_DEGREE = (STEPS_PER_REVOLUTION * MICROSTEPS) / 360.0f;
extern float yaw;
static float s_ma_current_target_angle = 0.0f;  // 电机运动的绝对目标角度
static float s_ma_current_assumed_angle = 0.0f; // 电机当前假定的绝对角度 (关键的“里程计”)
static long  s_ma_pulses_remaining = 0;         // 当前运动剩余的脉冲数
static int   s_ma_direction = 0;                // 运动方向: 1=正, -1=反, 0=停
static float s_ma_pulse_accumulator = 0.0f;     // 脉冲累加器，用于处理小数脉冲
static float s_mb_current_target_angle = 0.0f;  // 电机运动的绝对目标角度
static float s_mb_current_assumed_angle = 0.0f; // 电机当前假定的绝对角度 (关键的“里程计”)
static long  s_mb_pulses_remaining = 0;         // 当前运动剩余的脉冲数
static int   s_mb_direction = 0;                // 运动方向: 1=正, -1=反, 0=停
static float s_mb_pulse_accumulator = 0.0f;     // 脉冲累加器，用于处理小数脉冲
static float s_mb_current_speed_rpm = 0.0f; // << 新增: 用于追踪当前实时速度



int Calculate_target(int Target) 
{
	
	if (Target==0) return 0;//如果目标值为0，直接返回0失能电机
    // 方向由Target的符号决定，数值部分表示转速（RPM）
    int direction = (Target >= 0) ? 1 : -1;
    float speed = (float)abs(Target); // 转速绝对值（RPM）
    
    // 计算每转需要的总步数（考虑细分）
    float stepsPerRevolution = STEPS_PER_REVOLUTION * MICROSTEPS;
    
    // 计算每秒需要的步数（SPM: Steps Per Minute -> SPS: Steps Per Second）
    float stepsPerSecond = (speed * stepsPerRevolution) / 60.0f;
    
    // 计算PWM频率（Hz）：每秒需要的脉冲数
    float pwmFrequency = stepsPerSecond;
    
    // 计算对应的PWM周期值（Period_PWM_Count）
    // 周期值 = PWM时钟频率 / PWM目标频率 - 1
    uint16_t periodValue = (uint16_t)((PWM_CLOCK_FREQ / pwmFrequency) - 1);
    
    // 确保计算结果在有效范围内（0-65535）
    if (periodValue > 65535) {
        periodValue = 65535; // 限制最大值
    }
    // 返回带方向的周期值
    return direction * periodValue;
}



/**
 * @brief 将 RPM (每分钟转速) 转换为PWM定时器需要的周期计数值(LoadValue)
 * @param rpm 目标转速
 * @return int 返回给PWM定时器设置的周期值。该值越大，PWM频率越低，电机转速越慢。
 */
static int rpm_to_pwm_period(int rpm)
{
    // 如果速度为0，不产生脉冲，直接返回0
    if (rpm <= 0) return 0;

    // 1. 将 RPM (转/分钟) 转换为 RPS (转/秒)
    float rps = (float)rpm / 60.0f;
    // 2. 将 RPS (转/秒) 转换为 Steps-per-second (脉冲数/秒)
    float steps_per_second = rps * STEPS_PER_REVOLUTION * MICROSTEPS;
    // 3. 脉冲数/秒 就是PWM的频率(Hz)
    // 4. 根据PWM频率和时钟频率，计算定时器周期值。公式: Period = ClockFreq / PwmFreq - 1
    return (int)((PWM_CLOCK_FREQ / steps_per_second) - 1);
}

/**
 * @brief 计算电机A(左)为到达目标角度所需要的PWM控制值
 * @param target_angle 电机A的目标绝对角度 (单位: 度)
 * @return int 带符号的PWM周期值。
 *         - 正数: 表示正转，数值为PWM周期值。
 *         - 负数: 表示反转，数值的绝对值为PWM周期值。
 *         - 0:    表示停止。
 */
// int Calculate_MA_Angle_PWM(float target_angle)
// {
//     // 电机A的私有状态变量。使用'static'关键字，使得它们的值在函数多次调用之间得以保持。
//     static float s_ma_current_target_angle = 0.0f;   // 记录上一次的目标，用于判断指令是否更新
//     static float s_ma_current_assumed_angle = 0.0f;  // 内部追踪的电机当前假定位置（伪闭环）
//     static long  s_ma_pulses_remaining = 0;          // 本次运动还剩余多少脉冲需要发送
//     static int   s_ma_direction = 0;                 // 运动方向: 1=正, -1=反, 0=停
//     static float s_ma_pulse_accumulator = 0.0f;      // 脉冲累加器，用于补偿浮点运算的精度损失

//     // 步骤 1: 检查目标角度是否发生变化。如果变了，就规划一次新的运动。
//     // 这个if块只在目标角度更新的那一次中断中执行。
//     if (target_angle != s_ma_current_target_angle)
//     {
//         s_ma_current_target_angle = target_angle; // 锁定新目标
//         // 从上一次运动的终点，计算到新目标需要移动的角度
//         float angle_to_move = target_angle - s_ma_current_assumed_angle;
//         // 计算总脉冲数
//         s_ma_pulses_remaining = (long)(fabs(angle_to_move) * PULSES_PER_DEGREE);
//         // 重置脉冲累加器
//         s_ma_pulse_accumulator = 0.0f;
//         // 根据移动角度的正负，确定方向
//         s_ma_direction = (angle_to_move > 0) ? 1 : ((angle_to_move < 0) ? -1 : 0);
//     }

//     // 步骤 2: 如果还有脉冲需要发送，就执行运动控制逻辑。
//     // 这个if块在电机运动期间的每一次中断中都会执行。
//     if (s_ma_pulses_remaining > 0)
//     {
//         int current_speed_rpm;
//         // 根据减速角度，计算出需要开始减速的脉冲数阈值
//         long slowdown_threshold_pulses = (long)(DECELERATION_DEGREES * PULSES_PER_DEGREE);

//         // 核心精度控制逻辑：两段速控制
//         if (s_ma_pulses_remaining < slowdown_threshold_pulses) {
//             // 如果剩余脉冲数已进入减速区，切换到慢速
//             current_speed_rpm = SLOW_SPEED_RPM;
//         } else {
//             // 否则，保持高速巡航
//             current_speed_rpm = FAST_SPEED_RPM;
//         }
        
//         // 根据当前速度，计算在这个中断周期内(10ms)，理论上应发送多少个脉冲（可能是小数）
//         float steps_per_second = (current_speed_rpm * (STEPS_PER_REVOLUTION * MICROSTEPS)) / 60.0f;
//         float pulses_to_send_float = steps_per_second * ISR_INTERVAL_S;
        
//         // 使用累加器，将每次计算出的小数脉冲累加起来
//         s_ma_pulse_accumulator += pulses_to_send_float;
//         // 从累加器中取出整数部分作为本次中断实际要发送的脉冲数
//         long pulses_this_interval = (long)s_ma_pulse_accumulator;
        
//         if (pulses_this_interval > 0) {
//             s_ma_pulse_accumulator -= pulses_this_interval; // 累加器减去已发送的部分，保留小数
//             s_ma_pulses_remaining -= pulses_this_interval;  // 总剩余脉冲数也减去相应部分
//         }

//         // 步骤 3: 检查运动是否完成
//         if (s_ma_pulses_remaining <= 0)
//         {
//             s_ma_pulses_remaining = 0; // 清零，防止负数
//             s_ma_direction = 0;        // 停止
//             // 到达目标！更新电机的假定位置为当前目标位置。这是伪闭环的关键。
//             s_ma_current_assumed_angle = s_ma_current_target_angle; 
//             return 0; // 返回0，命令Set_PWM函数停止电机
//         }
        
//         // 如果运动未完成，返回带方向的速度指令
//         return s_ma_direction * rpm_to_pwm_period(current_speed_rpm);
//     }
    
//     // 如果没有运动（已到达或无新指令），返回0
//     return 0;
// }

/**
 * @brief 计算电机B(右)为到达目标角度所需要的PWM控制值
 * @note  此函数逻辑与Calculate_MA_Angle_PWM完全相同，但使用独立的static变量，
 *        从而实现对电机B的独立状态追踪和控制。
 */
// int Calculate_MB_Angle_PWM(float target_angle)
// {
//     // 电机B的私有状态变量 (与电机A的完全独立)
//     static float s_mb_current_target_angle = 0.0f;
//     static float s_mb_current_assumed_angle = 0.0f;
//     static long s_mb_pulses_remaining = 0;
//     static int s_mb_direction = 0;
//     static float s_mb_pulse_accumulator = 0.0f;

//     // --- 算法逻辑与电机A完全一致，此处不再重复注释 ---
    
//     if (target_angle != s_mb_current_target_angle)
//     {
//         s_mb_current_target_angle = target_angle;
//         float angle_to_move = target_angle - s_mb_current_assumed_angle;
//         s_mb_pulses_remaining = (long)(fabs(angle_to_move) * PULSES_PER_DEGREE);
//         s_mb_pulse_accumulator = 0.0f;
//         s_mb_direction = (angle_to_move > 0) ? 1 : ((angle_to_move < 0) ? -1 : 0);
//     }

//     if (s_mb_pulses_remaining > 0)
//     {
//         int current_speed_rpm;
//         long slowdown_threshold_pulses = (long)(DECELERATION_DEGREES * PULSES_PER_DEGREE);

//         if (s_mb_pulses_remaining < slowdown_threshold_pulses) {
//             current_speed_rpm = SLOW_SPEED_RPM;
//         } else {
//             current_speed_rpm = FAST_SPEED_RPM;
//         }
        
//         float steps_per_second = (current_speed_rpm * (STEPS_PER_REVOLUTION * MICROSTEPS)) / 60.0f;
//         float pulses_to_send_float = steps_per_second * ISR_INTERVAL_S;
        
//         s_mb_pulse_accumulator += pulses_to_send_float;
//         long pulses_this_interval = (long)s_mb_pulse_accumulator;
        
//         if (pulses_this_interval > 0) {
//             s_mb_pulse_accumulator -= pulses_this_interval;
//             s_mb_pulses_remaining -= pulses_this_interval;
//         }

//         if (s_mb_pulses_remaining <= 0)
//         {
//             s_mb_pulses_remaining = 0;
//             s_mb_direction = 0;
//             s_mb_current_assumed_angle = s_mb_current_target_angle;
//             return 0;
//         }
        
//         return s_mb_direction * rpm_to_pwm_period(current_speed_rpm);
//     }
    
//     return 0;
// }

// ==============================================================================
//  5. 底层硬件驱动函数
// ==============================================================================

/**
 * @brief 步进电机底层驱动函数
 * @param L_Target 左电机(A)的控制指令，由Calculate_MA_Angle_PWM()生成。
 * @param R_Target 右电机(B)的控制指令，由Calculate_MB_Angle_PWM()生成。
 * @details
 *  - 根据指令值的正负号设置电机方向。
 *  - 根据指令值的绝对值设置PWM频率（即电机速度）。
 *  - 指令值为0时，关闭电机使能，释放电机。
 */
void Step_Set_PWM(int L_Target)
{
    // --- 左电机(A)控制 ---
	if(L_Target > 0) // 指令为正数: 正转
    {
		DL_GPIO_setPins(EN_PORT, EN_MA_PIN);     // 1. 打开电机A的使能
		DL_GPIO_setPins(DIR_PORT, DIR_A_PIN);    // 2. 设置电机A的方向为正转 (假设高电平为正)
		DL_TimerA_setLoadValue(PWM_0_INST, abs(L_Target)); // 3. 设置PWM周期，决定转速
		DL_Timer_setCaptureCompareValue(PWM_0_INST, abs(L_Target/2), GPIO_PWM_0_C0_IDX); // 4. 设置比较值，产生50%占空比方波
    }
    else if(L_Target < 0) // 指令为负数: 反转
    {
		DL_GPIO_setPins(EN_PORT, EN_MA_PIN);     // 1. 打开电机A的使能
		DL_GPIO_clearPins(DIR_PORT, DIR_A_PIN);  // 2. 设置电机A的方向为反转 (假设低电平为反)
		DL_TimerA_setLoadValue(PWM_0_INST, abs(L_Target));
		DL_Timer_setCaptureCompareValue(PWM_0_INST, abs(L_Target/2), GPIO_PWM_0_C0_IDX);
    }
	else // 指令为0: 停止
    {
        DL_Timer_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C0_IDX);
        //DL_GPIO_clearPins(EN_PORT, EN_MA_PIN); // 关闭电机A的使能，电机将处于自由态，不产生力矩
    }
    // --- 右电机(B)控制 ---
	// if(R_Target > 0) // 指令为正数: 正转
    // {
	// 	DL_GPIO_setPins(EN_PORT, EN_MB_PIN);
	// 	DL_GPIO_setPins(DIR_PORT, DIR_B_PIN);
	// 	DL_TimerG_setLoadValue(PWM_1_INST, abs(R_Target));
    //     DL_Timer_setCaptureCompareValue(PWM_1_INST, abs(R_Target/2), GPIO_PWM_1_C0_IDX);
    // }
    // else if(R_Target < 0) // 指令为负数: 反转
    // {
	// 	DL_GPIO_setPins(EN_PORT, EN_MB_PIN);
	// 	DL_GPIO_clearPins(DIR_PORT, DIR_B_PIN);
	// 	DL_TimerG_setLoadValue(PWM_1_INST, abs(R_Target));
	// 	DL_Timer_setCaptureCompareValue(PWM_1_INST, abs(R_Target/2), GPIO_PWM_1_C0_IDX);
    // }
	// else // 指令为0: 停止
    // {
    //     DL_Timer_setCaptureCompareValue(PWM_1_INST, 0, GPIO_PWM_1_C0_IDX);
    //     //DL_GPIO_clearPins(EN_PORT, EN_MB_PIN);
    // }
}

// int Calculate_A_Angle_to_turn(float target_angle)
// {
//     static float s_ma_current_target_angle = 0.0f;   // 当前的目标角度
//     static long  s_ma_pulses_remaining = 0;           // 剩余的脉冲数
//     static int   s_ma_direction = 0;                  // 旋转方向: 1=顺时针, -1=逆时针, 0=停止
//     static float s_ma_pulse_accumulator = 0.0f;       // 脉冲累加器

//     // 更新目标角度时，计算旋转角度的变化
//     if (target_angle != s_ma_current_target_angle)
//     {
//         s_ma_current_target_angle = target_angle;  // 更新目标角度
//         float angle_to_move = target_angle - s_ma_current_target_angle; // 计算角度变化
//         s_ma_pulses_remaining = (long)(fabs(angle_to_move) * PULSES_PER_DEGREE); // 计算剩余脉冲数
//         s_ma_pulse_accumulator = 0.0f;
//         s_ma_direction = (angle_to_move > 0) ? 1 : ((angle_to_move < 0) ? -1 : 0); // 更新方向
//     }

//     if (s_ma_pulses_remaining > 0)
//     {
//         int current_speed_rpm;
//         long slowdown_threshold_pulses = (long)(DECELERATION_DEGREES * PULSES_PER_DEGREE);

//         if (s_ma_pulses_remaining < slowdown_threshold_pulses) {
//             current_speed_rpm = SLOW_SPEED_RPM;  // 减速阶段
//         } else {
//             current_speed_rpm = FAST_SPEED_RPM;  // 加速阶段
//         }
        
//         float steps_per_second = (current_speed_rpm * (STEPS_PER_REVOLUTION * MICROSTEPS)) / 60.0f;
//         float pulses_to_send_float = steps_per_second * ISR_INTERVAL_S;
        
//         s_ma_pulse_accumulator += pulses_to_send_float;  // 累加脉冲
//         long pulses_this_interval = (long)s_ma_pulse_accumulator;
        
//         if (pulses_this_interval > 0) {
//             s_ma_pulse_accumulator -= pulses_this_interval;
//             s_ma_pulses_remaining -= pulses_this_interval;
//         }

//         if (s_ma_pulses_remaining <= 0)
//         {
//             s_ma_pulses_remaining = 0;
//             s_ma_direction = 0;
//             return 0;  // 停止旋转
//         }
        
//         return s_ma_direction * rpm_to_pwm_period(current_speed_rpm);
//     }
    
//     return 0;  // 没有旋转
// }

void Calculate_A_Angle_to_turn(float angle_to_add)
{
  // 如果电机正在运动 (还有剩余脉冲)，则忽略新的指令以防冲突
    if (s_ma_pulses_remaining > 0) {
        return;
    }

    // 如果转动角度为0，则什么也不做
    if (angle_to_add == 0.0f) {
        return;
    }

    // 核心逻辑: 新的目标角度 = 当前假定角度 + 需要增加的角度
    s_ma_current_target_angle = s_ma_current_assumed_angle + angle_to_add;

    // 基于要转动的相对角度，计算总脉冲数和方向
    s_ma_pulses_remaining = (long)(fabs(angle_to_add) * PULSES_PER_DEGREE);
    s_ma_pulse_accumulator = 0.0f;
    s_ma_direction = (angle_to_add > 0) ? 1 : -1;
}

/**
 * @brief 处理函数: 处理电机B的当前运动状态并返回PWM值
 * @return int 返回给Set_PWM函数的控制值 (正/负代表方向和速度，0代表停止)
 * @note   此函数应该被定时器中断周期性调用 (例如，每10ms)。
 */
int Process_MotorA_Movement_PWM()
{
    // 只有在有剩余脉冲时，才执行运动控制
    if (s_ma_pulses_remaining > 0)
    {
        // 这部分运动控制逻辑与你的原函数完全相同
        int current_speed_rpm;
        long slowdown_threshold_pulses = (long)(DECELERATION_DEGREES * PULSES_PER_DEGREE);

        if (s_ma_pulses_remaining < slowdown_threshold_pulses) {
            current_speed_rpm = SLOW_SPEED_RPM;
        } else {
            current_speed_rpm = FAST_SPEED_RPM;
        }

        float steps_per_second = (current_speed_rpm * (STEPS_PER_REVOLUTION * MICROSTEPS)) / 60.0f;
        float pulses_to_send_float = steps_per_second * ISR_INTERVAL_S;

        s_ma_pulse_accumulator += pulses_to_send_float;
        long pulses_this_interval = (long)s_ma_pulse_accumulator;

        if (pulses_this_interval > 0) {
            s_ma_pulse_accumulator -= pulses_this_interval;
            s_ma_pulses_remaining -= pulses_this_interval;
        }

        // 检查运动是否刚好完成
        if (s_ma_pulses_remaining <= 0)
        {
            s_ma_pulses_remaining = 0;
            s_ma_direction = 0;
            // 关键步骤: 运动结束后，更新“里程计”，将当前位置更新为目标位置
            s_ma_current_assumed_angle = s_ma_current_target_angle;
            return 0; // 返回0，停止电机
        }

        // 运动未完成，返回带方向的速度指令
        return s_ma_direction * rpm_to_pwm_period(current_speed_rpm);
    }

    // 如果没有剩余脉冲，电机处于空闲状态，返回0
    return 0;
}

void Calculate_B_Angle_to_turn(float angle_to_add)
{
    // 如果电机正在运动 (还有剩余脉冲)，则忽略新的指令以防冲突
    if (s_mb_pulses_remaining > 0) {
        return;
    }
    if (angle_to_add == 0.0f) {
        return;
    }

    s_mb_current_target_angle = s_mb_current_assumed_angle + angle_to_add;
    s_mb_pulses_remaining = (long)(fabs(angle_to_add) * PULSES_PER_DEGREE);
    s_mb_pulse_accumulator = 0.0f;
    s_mb_direction = (angle_to_add > 0) ? 1 : -1;
    
    // >> 新增: 启动时将当前速度设置为初始速度，而不是0
    s_mb_current_speed_rpm = INITIAL_SPEED_RPM; 
}

/**
 * @brief 处理函数: 处理电机B的当前运动状态并返回PWM值
 * @return int 返回给Set_PWM函数的控制值 (正/负代表方向和速度，0代表停止)
 * @note   此函数应该被定时器中断周期性调用 (例如，每10ms)。
 */
int Process_MotorB_Movement_PWM()
{
    // 只有在有剩余脉冲时，才执行运动控制
   if (s_mb_pulses_remaining > 0)
    {
        // 计算每一中断间隔，速度应该变化多少
        float speed_increment = ACCELERATION_RPM_PER_SEC * ISR_INTERVAL_S;
        
        // 计算减速需要的脉冲数 (这是一个简化的估算，但很有效)
        // 也可以继续使用固定的 DECELERATION_DEGREES * PULSES_PER_DEGREE
        long slowdown_threshold_pulses = (long)(DECELERATION_DEGREES * PULSES_PER_DEGREE);

        // --- 加减速逻辑判断 ---
        // 1. 是否进入减速阶段？
        if (s_mb_pulses_remaining < slowdown_threshold_pulses) {
            s_mb_current_speed_rpm -= speed_increment; // 减速
            // 防止速度降得太低
            if (s_mb_current_speed_rpm < SLOW_SPEED_RPM) {
                s_mb_current_speed_rpm = SLOW_SPEED_RPM;
            }
        } 
        // 2. 否则，是否需要加速？
        else if (s_mb_current_speed_rpm < FAST_SPEED_RPM) {
            s_mb_current_speed_rpm += speed_increment; // 加速
            // 防止速度超过设定的最快速度
            if (s_mb_current_speed_rpm > FAST_SPEED_RPM) {
                s_mb_current_speed_rpm = FAST_SPEED_RPM;
            }
        }
        
        // 使用动态计算出的当前速度 s_mb_current_speed_rpm 来生成脉冲
        float steps_per_second = (s_mb_current_speed_rpm * (STEPS_PER_REVOLUTION * MICROSTEPS)) / 60.0f;
        float pulses_to_send_float = steps_per_second * ISR_INTERVAL_S;
        
        s_mb_pulse_accumulator += pulses_to_send_float;
        long pulses_this_interval = (long)s_mb_pulse_accumulator;
        
        if (pulses_this_interval > 0) {
            s_mb_pulse_accumulator -= pulses_this_interval;
            s_mb_pulses_remaining -= pulses_this_interval;
        }

        if (s_mb_pulses_remaining <= 0)
        {
            s_mb_pulses_remaining = 0;
            s_mb_direction = 0;
            s_mb_current_assumed_angle = s_mb_current_target_angle;
            s_mb_current_speed_rpm = 0.0f; // >> 新增: 运动结束，速度清零
            return 0;
        }
        
        // 使用实时速度来计算PWM周期
        return s_mb_direction * rpm_to_pwm_period((int)s_mb_current_speed_rpm);
    }
    
    return 0;
}


void laser_on()
{
    DL_GPIO_setPins(Laser_PORT,Laser_PIN_0_PIN);
}