/**
 * @file alg_power_limit.h
 * @author qyx
 * @brief 自适应功率限制算法
 * @version 1.2
 * @date
 *
 * @copyright ZLLC 2025
 *
 */

#include "alg_power_limit.h"
#include "math.h"

/* Private macros ------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


/* Private function declarations ---------------------------------------------*/
static inline bool floatEqual(float a, float b) { return fabs(a - b) < 1e-5f; }
static inline float rpm2av(float rpm) { return rpm * (float)PI / 30.0f; }
static inline float av2rpm(float av) { return av * 30.0f / (float)PI; }
static inline float my_fmax(float a, float b) { return (a > b) ? a : b; }

void Class_Power_Limit::Init()
{
#ifdef AGV
    float initParams_dir[2] = {k1_dir, k2_dir};
    rls_dir.setParamVector(Matrixf<2, 1>(initParams_dir));
    float initParams_mot[2] = {k1_mot, k2_mot};
    rls_mot.setParamVector(Matrixf<2, 1>(initParams_mot));
#else
    float initParams[2] = {k1, k2};
    rls.setParamVector(Matrixf<2, 1>(initParams));
#endif
}

/**
 * @brief 返回单个电机的计算功率
 *
 * @param omega 转子转速，单位为rpm
 * @param torque 转子扭矩大小，单位为nm
 * @param motor_index 电机索引，偶数为转向电机，奇数为动力电机，舵轮需要传此参数
 * @return float 理论功率值
 */
float Class_Power_Limit::Calculate_Theoretical_Power(float omega, float torque, uint8_t motor_index)
{
#ifdef AGV
    // 根据电机索引选择对应的参数
    bool is_direction_motor = (motor_index % 2 == 0);
    float k1_use = is_direction_motor ? k1_dir : k1_mot;
    float k2_use = is_direction_motor ? k2_dir : k2_mot;
    float k3_use = is_direction_motor ? k3_dir : k3_mot;

    float cmdPower = rpm2av(omega) * torque +
                     fabs(rpm2av(omega)) * k1_use +
                     torque * torque * k2_use +
                     k3_use;
#else
    float cmdPower = rpm2av(omega) * torque +
                     fabs(rpm2av(omega)) * k1 +
                     torque * torque * k2 +
                     k3;
#endif
    return cmdPower;
}

/**
 * @brief 计算功率系数
 *
 * @param actual_power 实际功率
 * @param motor_data 电机数据数组
 */
void Class_Power_Limit::Calculate_Power_Coefficient(float actual_power, const Struct_Power_Motor_Data *motor_data)
{
#ifdef AGV
    static Matrixf<2, 1> samples_mot, samples_dir;
    static Matrixf<2, 1> params_mot, params_dir;
    float effectivePower_mot = 0, effectivePower_dir = 0;

    samples_mot[0][0] = samples_mot[1][0] = 0;
    samples_dir[0][0] = samples_dir[1][0] = 0;

    if (actual_power > 5)
    {
        // 分别处理动力电机(奇数索引)和转向电机(偶数索引)
        for (int i = 0; i < 8; i++)
        {
            if (i % 2 == 0) // 转向电机
            {
                if (motor_data[i].feedback_torque * rpm2av(motor_data[i].feedback_omega) > 0)
                {
                    effectivePower_dir += motor_data[i].feedback_torque *
                                          rpm2av(motor_data[i].feedback_omega);
                }
                samples_dir[0][0] += fabsf(rpm2av(motor_data[i].feedback_omega));
                samples_dir[1][0] += motor_data[i].feedback_torque *
                                     motor_data[i].feedback_torque;
            }
            else // 动力电机
            {
                if (motor_data[i].feedback_torque * rpm2av(motor_data[i].feedback_omega) > 0)
                {
                    effectivePower_mot += motor_data[i].feedback_torque *
                                          rpm2av(motor_data[i].feedback_omega);
                }
                samples_mot[0][0] += fabsf(rpm2av(motor_data[i].feedback_omega));
                samples_mot[1][0] += motor_data[i].feedback_torque *
                                     motor_data[i].feedback_torque;
            }
        }

        // 更新RLS参数
        float power_ratio = 0.8f; // 动力电机功率占比
        params_mot = rls_mot.update(samples_mot,
                                    power_ratio * actual_power - effectivePower_mot - 4 * k3_mot);
        params_dir = rls_dir.update(samples_dir,
                                    (1 - power_ratio) * actual_power - effectivePower_dir - 4 * k3_dir);

        // 更新系数
        k1_mot = my_fmax(params_mot[0][0], 1e-5f);
        k2_mot = my_fmax(params_mot[1][0], 1e-5f);
        k1_dir = my_fmax(params_dir[0][0], 1e-5f);
        k2_dir = my_fmax(params_dir[1][0], 1e-5f);
    }
#else
    static Matrixf<2, 1> samples;
    static Matrixf<2, 1> params;
    float effectivePower = 0;

    samples[0][0] = samples[1][0] = 0;

    if (actual_power > 5)
    {
        for (int i = 0; i < 8; i++)
        {
            if (motor_data[i].feedback_torque * rpm2av(motor_data[i].feedback_omega) > 0)
            {
                effectivePower += motor_data[i].feedback_torque *
                                  rpm2av(motor_data[i].feedback_omega);
            }
            samples[0][0] += fabsf(rpm2av(motor_data[i].feedback_omega));
            samples[1][0] += motor_data[i].feedback_torque *
                             motor_data[i].feedback_torque;
        }

        params = rls.update(samples, actual_power - effectivePower - 8 * k3);
        k1 = my_fmax(params[0][0], 1e-5f);
        k2 = my_fmax(params[1][0], 1e-5f);
    }
#endif
}

/**
 * @brief 计算限制后的扭矩
 *
 * @param omega 转子转速，单位为rpm
 * @param power 限制功率值
 * @param torque 原始扭矩值
 * @param motor_index 电机索引，偶数为转向电机，奇数为动力电机
 * @return float 限制后的扭矩值
 */
float Class_Power_Limit::Calculate_Toque(float omega, float power, float torque, uint8_t motor_index)
{
#ifdef AGV
    bool is_direction_motor = (motor_index % 2 == 0);
    float k1_use = is_direction_motor ? k1_dir : k1_mot;
    float k2_use = is_direction_motor ? k2_dir : k2_mot;
    float k3_use = is_direction_motor ? k3_dir : k3_mot;
#endif

    omega = rpm2av(omega);
    float newTorqueCurrent = 0.0f;

#ifdef AGV
    float delta = omega * omega - 4 * (k1_use * fabs(omega) + k3_use - power) * k2_use;
#else
    float delta = omega * omega - 4 * (k1 * fabs(omega) + k3 - power) * k2;
#endif

    if (torque * omega <= 0)
    {
        newTorqueCurrent = torque;
    }
    else
    {
        if (floatEqual(delta, 0.0f))
        {
            newTorqueCurrent = -omega / (2.0f * k2_use);
        }
        else if (delta > 0.0f)
        {
#ifdef AGV
            float solution1 = (-omega + sqrtf(delta)) / (2.0f * k2_use);
            float solution2 = (-omega - sqrtf(delta)) / (2.0f * k2_use);
#else
            float solution1 = (-omega + sqrtf(delta)) / (2.0f * k2);
            float solution2 = (-omega - sqrtf(delta)) / (2.0f * k2);
#endif
            newTorqueCurrent = (torque > 0) ? solution1 : solution2;
        }
        else
        {
            newTorqueCurrent = -omega / (2.0f * k2_use);
        }
    }
    return newTorqueCurrent;
}

/**
 * @brief 功率限制主任务
 *
 * @param power_management 功率管理结构体
 */
void Class_Power_Limit::Power_Task(Struct_Power_Management &power_management)
{
#ifdef AGV
    float theoretical_sum_mot = 0, theoretical_sum_dir = 0;
    float scaled_sum_mot = 0, scaled_sum_dir = 0;

    // 分别计算动力和转向电机的理论功率
    for (uint8_t i = 0; i < 8; i++)
    {
        power_management.Motor_Data[i].theoretical_power =
            Calculate_Theoretical_Power(power_management.Motor_Data[i].feedback_omega,
                                        power_management.Motor_Data[i].torque,
                                        i);

        if (i % 2 == 0) // 转向电机
        {
            if (power_management.Motor_Data[i].theoretical_power > 0)
            {
                theoretical_sum_dir += power_management.Motor_Data[i].theoretical_power;
            }
            else
            {
                power_management.Motor_Data[i].theoretical_power = 0;
            }
        }
        else // 动力电机
        {
            if (power_management.Motor_Data[i].theoretical_power > 0)
            {
                theoretical_sum_mot += power_management.Motor_Data[i].theoretical_power;
            }
            else
            {
                power_management.Motor_Data[i].theoretical_power = 0;
            }
        }
    }

    // 新的功率分配逻辑
    float scale_mot = 1.0f, scale_dir = 1.0f;
    float dir_power_limit = power_management.Max_Power * 0.8f; // 转向电机功率上限
    float mot_power_limit;                                     // 动力电机功率上限，动态计算

    // 首先分配转向电机功率
    if (theoretical_sum_dir > dir_power_limit)
    {
        // 转向功率需求超过限制，按限制分配
        scale_dir = dir_power_limit / theoretical_sum_dir;
        mot_power_limit = power_management.Max_Power - dir_power_limit;
    }
    else
    {
        // 转向功率需求未超限制，全部分配
        scale_dir = 1.0f;
        // 剩余功率全部分配给动力电机
        mot_power_limit = power_management.Max_Power - theoretical_sum_dir;
    }

    // 然后分配动力电机功率
    if (theoretical_sum_mot > mot_power_limit)
    {
        scale_mot = mot_power_limit / theoretical_sum_mot;
    }
    else
    {
        scale_mot = 1.0f;
    }

    // 应用收缩系数并更新输出
    for (uint8_t i = 0; i < 8; i++)
    {
        float scale = (i % 2 == 0) ? scale_dir : scale_mot;
        power_management.Motor_Data[i].scaled_power =
            power_management.Motor_Data[i].theoretical_power * scale;

        if (i % 2 == 0)
        {
            scaled_sum_dir += power_management.Motor_Data[i].scaled_power;
        }
        else
        {
            scaled_sum_mot += power_management.Motor_Data[i].scaled_power;
        }

        power_management.Motor_Data[i].output =
            Calculate_Toque(power_management.Motor_Data[i].feedback_omega,
                            power_management.Motor_Data[i].scaled_power,
                            power_management.Motor_Data[i].torque,
                            i) *
            GET_TORQUE_TO_CMD_CURRENT(i);

        // 限幅处理
        power_management.Motor_Data[i].output =
            (power_management.Motor_Data[i].output > 16384) ? 16384 : (power_management.Motor_Data[i].output < -16384) ? -16384
                                                                                                                       : power_management.Motor_Data[i].output;
    }

    power_management.Theoretical_Total_Power = theoretical_sum_mot + theoretical_sum_dir;
    power_management.Scaled_Total_Power = scaled_sum_mot + scaled_sum_dir;

    Calculate_Power_Coefficient(power_management.Actual_Power, power_management.Motor_Data);
#else
    float theoretical_sum = 0;
    float scaled_sum = 0;

    // 计算理论功率
    for (uint8_t i = 0; i < 8; i++)
    {
        power_management.Motor_Data[i].theoretical_power =
            Calculate_Theoretical_Power(power_management.Motor_Data[i].feedback_omega,
                                        power_management.Motor_Data[i].torque,
                                        i);
        if (power_management.Motor_Data[i].theoretical_power > 0)
        {
            theoretical_sum += power_management.Motor_Data[i].theoretical_power;
        }
    }

    power_management.Theoretical_Total_Power = theoretical_sum;

    // 计算收缩系数
    if (power_management.Max_Power < power_management.Theoretical_Total_Power)
    {
        power_management.Scale_Conffient =
            power_management.Max_Power / power_management.Theoretical_Total_Power;
    }
    else
    {
        power_management.Scale_Conffient = 1.0f;
    }

    // 应用收缩系数并更新输出
    for (uint8_t i = 0; i < 8; i++)
    {
        power_management.Motor_Data[i].scaled_power =
            power_management.Motor_Data[i].theoretical_power *
            power_management.Scale_Conffient;

        scaled_sum += power_management.Motor_Data[i].scaled_power;

        power_management.Motor_Data[i].output =
            Calculate_Toque(power_management.Motor_Data[i].feedback_omega,
                            power_management.Motor_Data[i].scaled_power,
                            power_management.Motor_Data[i].torque,
                            i) *
            TORQUE_TO_CMD_CURRENT;

        if (abs(power_management.Motor_Data[i].output) >= 16384)
        {
            power_management.Motor_Data[i].output = 0;
        }
    }

    power_management.Scaled_Total_Power = scaled_sum;

    Calculate_Power_Coefficient(power_management.Actual_Power, power_management.Motor_Data);
#endif
}