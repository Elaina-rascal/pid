/*
 * @Author: Elaina
 * @Date: 2024-07-07 17:06:10
 * @LastEditors: chaffer-cold 1463967532@qq.com
 * @LastEditTime: 2024-07-31 20:30:17
 * @FilePath: \MDK-ARM\Hardware\pid.cpp
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#include "pid.h"
pid_base_t::pid_base_t()
{

    kp_ = 0.6;
    kd_ = 0.2;
    target_ = 0;
    last_output_ = 0;
}
pid_base_t::pid_base_t(float kp, float ki, float kd)
{
    reset();                // 初始化控制器
    pid_update(kp, ki, kd); // 更新PID参数
}
pid_base_t::pid_base_t(float kp, float ki, float kd, float out_min, float out_max)
{
    reset();                // 初始化控制器
    pid_update(kp, ki, kd); // 更新PID参数
    out_limit(out_min, out_max);
}

/**
 * @brief 更新pid参并重置pid控制器
 * @param {float} kp
 * @param {float} ki
 * @param {float} kd
 * @return {*}
 * @note 还要设置限幅参数才能用
 */
void pid_base_t::pid_update(float kp, float ki, float kd)
{
    reset();
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}
/**
 * @brief 输出限幅
 * @param {float} out_min
 * @param {float} out_max
 * @return {*}
 * @note:
 */
void pid_base_t::out_limit(float out_min, float out_max)
{
    out_min_ = out_min;
    out_max_ = out_max;
}
/**
 * @brief 控制器清零重置
 * @return {*}
 * @note:
 */
void pid_base_t::reset(void)
{
    last_output_ = 0.0f; // 上一次的控制输出值
    target_ = 0.0f;      // 控制目标值
    out_min_ = 0.0f;     // 控制输出最小值
    out_max_ = 0.0f;     // 控制输出最大值

    kp_ = 0.0;
    ki_ = 0.0;
    kd_ = 0.0;

    error = 0.0;
    error_delta = 0.0;
    error_last = 0.0;
    error_sum = 0.0;
    error_pre = 0.0;
}
/**
 * @brief 更新目标值
 * @param {float} target
 * @param {bool} clear_integral 是否清除积分
 * @return {*}
 * @note:
 */
void pid_base_t::target_update(float target, bool clear_integral)
{
    target_ = target;
    // 积分过大的时候清除积分
    if (clear_integral)
    {
        error_sum = 0;
    }
}
/**
 * @brief 更新pid控制器
 * @param {float} contrl 当前实际状态
 * @return {*} 控制输出
 * @note 设置过目标情况下才能用
 */
float pid_base_t::update(float contrl)
{
    // 计算误差
    error = target_ - contrl;
    // 积分微分
    error_sum += error;
    error_delta = error_last - error;

    // 更新误差
    error_last = error;

    // 积分限幅
    if (error_sum > error_sum_max)
    {
        error_sum = error_sum_max;
    }
    if (error_sum < -error_sum_max)
    {
        error_sum = -error_sum_max;
    }
    // 输出计算
    float output = kp_ * error + ki_ * error_sum + kd_ * error_delta;
    // 输出限幅
    if (output > out_max_)
    {
        output = out_max_;
    }
    else if (output < out_min_)
    {
        output = out_min_;
    }

    last_output_ = output;
    return output;
}
/**
 * @brief 重置积分
 * @return {*}
 * @note:
 */
void pid_base_t::reset_integral(void)
{
    error_sum = 0;
}
/**
 * @brief  pid传入目标值与当前值进行计算
 * @param {float} target    目标值
 * @param {float} contrl    档期值
 * @param {bool} clear_integral 是否清除积分
 * @return {*}
 * @note:
 */
float pid_base_t::cal(float target, float contrl, bool clear_integral)
{
    target_update(target, clear_integral);
    return update(contrl);
}
float pid_base_t::output_limit(float output)
{
    if (output > out_max_)
    {
        output = out_max_;
    }
    else if (output < out_min_)
    {
        output = out_min_;
    }
    return output;
}

/**
 * @brief 前馈式pid更新控制量
 * @param {float} contrl
 * @param {bool} clear_integral 是否清除积分
 * @return {*}
 * @note   在有target情况下才能用
 */
float pid_foward_t::update(float contrl, bool clear_integral)
{

    return output_limit(pid_base_t::update(contrl) + forwardfeed());
}
/**
 * @brief 前馈量
 * @return {*}
 * @note:
 */
float pid_foward_t::forwardfeed()
{

    return forwardfeed_k_ * target_;
}
/**
 * @brief 更新前馈式pid参数
 * @param {float} kp
 * @param {float} ki
 * @param {float} kd
 * @param {float} forward_k
 * @return {*}
 * @note 还要设置限幅参数才能用
 */
void pid_foward_t::pidk_update(float kp, float ki, float kd, float forward_k)
{
    pid_base_t::pid_update(kp, ki, kd);
    forwardfeed_k_ = forward_k;
}
/**
 * @brief 前馈式pid传入目标值与当前值进行计算
 * @param {float} target    目标值
 * @param {float} contrl    实际值
 * @param {bool} clear_integral 是否清除积分
 * @return {*}
 * @note:
 */
float pid_foward_t::cal(float target, float contrl, bool clear_integral)
{
    return output_limit(cal(target, contrl, clear_integral) + forwardfeed());
}

/**
 * @brief 增量式pid更新控制量
 * @param {float} contrl
 * @return {*}
 * @note:
 */
float pid_Increment_t::update(float contrl)
{
    error = target_ - contrl;
    float output = kp_ * (error - error_last) + ki_ * error + kd_ * (error - 2 * error_last + error_last_last);
    // 更新误差
    error_last_last = error_last;
    error_last = error;
    // 输出限幅
    if (output > out_max_)
    {
        output = out_max_;
    }
    else if (output < out_min_)
    {
        output = out_min_;
    }
    return output;
}
/**
 * @brief 增量式pid传入目标值与当前值进行计算
 * @param {float} target    目标值
 * @param {float} contrl    实际值
 * @param {bool} clear_integral 是否清除积分
 * @return {*}
 * @note:
 */
float pid_Increment_t::cal(float target, float contrl, bool clear_integral)
{
    target_update(target, clear_integral);
    return update(contrl);
}