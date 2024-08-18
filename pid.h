/*
 * @Author: Elaina
 * @Date: 2024-07-07 17:06:10
 * @LastEditors: chaffer-cold 1463967532@qq.com
 * @LastEditTime: 2024-08-17 22:16:49
 * @FilePath: \MDK-ARM\Hardware\pid.h
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#ifndef __PID_HPP
#define __PID_HPP
// 先设置pid的值，再设置限幅才能用
// 位置式pid
/*更新有两种api*/
/*一是调用target_update(可以设置是否清除微分),再用update更新*/
/*二是调用cal(可以设置是否清除微分),要传入target与control*/
class pid_base_t
{
public:
    pid_base_t();
    pid_base_t(float kp, float ki, float kd);
    pid_base_t(float kp, float ki, float kd, float out_min, float out_max);
    float target_;
    float kp_;          // 比例系数
    float ki_;          // 积分系数
    float kd_;          // 微分系数
    float last_output_; // 上一次输出值

    float update(float contrl);                                         // 更新输出，在有target情况下
    float cal(float target, float contrl, bool clear_integral = false); // 计算
    void reset(void);                                                   // 重置pid控制器
    void target_update(float target, bool clear_integral = false);
    void pid_update(float kp, float ki, float kd); // 更新pid
    void out_limit(float out_min, float out_max);  // 设置限幅
    void reset_integral(void);                     // 重置积分
    float output_limit(float output);              // 输出限幅
protected:
    float error;              // 误差
    float error_sum;          // 累计误差
    float error_sum_max = 25; // 积分上限;

    float error_delta; // 误差微分
    float error_last;  // 上一次的误差
    float error_pre;   // 前次的误差

    float out_min_; // 输出下限
    float out_max_; // 输出上限
};
// 输入的contrl是当前的速度
// error_pre与last_output没有用到

// 前馈pid
class pid_foward_t : public pid_base_t
{
public:
    void pidk_update(float kp, float ki, float kd, float forward_k);
    float update(float contrl, bool clear_integral = false); // 在有target情况下计算
    float forwardfeed();
    float cal(float target, float contrl, bool clear_integral = false); // 计算
private:
    float forwardfeed_k_ = 0;
};
// 增量式pid
/*增量式pid中的i类似于位置式中的p,p类似与位置式中的i*/
class pid_Increment_t : public pid_base_t
{
public:
    float update(float contrl);
    float cal(float target, float contrl, bool clear_integral = false);

private:
    float error_last_last;
};
#endif
