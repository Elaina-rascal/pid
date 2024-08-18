#ifndef __PID_TEMPLATE_H
#define __PID_TEMPLATE_H
template <typename T>

struct PidBaseConfig_T
{
    T kp = 0;
    T ki = 0;
    T kd = 0;
    T out_min = 0;
    T out_max = 0;
    T integral_max = 25;
};

// 先设置pid的值，再设置限幅才能用
// 位置式pid
/*更新有两种api*/
/*一是调用target_update(可以设置是否清除微分),再用update更新*/
/*二是调用cal(可以设置是否清除微分),要传入target与control*/
template <typename T>
class pid_base_template_t
{
public:
    // 构造函数
    pid_base_template_t() {}; // 默认构造函数
    pid_base_template_t(T kp, T ki, T kd);
    pid_base_template_t(T kp, T ki, T kd, T out_min, T out_max);
    pid_base_template_t(PidBaseConfig_T<T> config) : pid_base_template_t(config.kp, config.ki, config.kd, config.out_min, config.out_max) {};

    T target_;
    T kp_;          // 比例系数
    T ki_;          // 积分系数
    T kd_;          // 微分系数
    T last_output_; // 上一次输出值

    T update(T contrl);                                     // 更新输出，在有target情况下
    T cal(T target, T contrl, bool clear_integral = false); // 计算
    void reset(void);                                       // 重置pid控制器
    void target_update(T target, bool clear_integral = false);
    void pid_update(T kp, T ki, T kd);    // 更新pid
    void out_limit(T out_min, T out_max); // 设置限幅
    void reset_integral(void);            // 重置积分
    T output_limit(T output);             // 输出限幅
protected:
    T error;              // 误差
    T error_sum;          // 累计误差
    T error_sum_max = 25; // 积分上限;

    T error_delta; // 误差微分
    T error_last;  // 上一次的误差

    T error_pre; // 前次的误差

    T out_min_; // 输出下限
    T out_max_; // 输出上限
};
// 输入的contrl是当前的速度
// error_pre与last_output没有用到

// 前馈pid
template <typename T>
class pid_foward_template_t : public pid_base_template_t<T>
{
public:
    // pid_update(T kp, T ki, T kd, T forward_k = 0) : pid_base_t<T>(kp, ki, kd), forwardfeed_k_(forward_k) {};
    pid_foward_template_t(T kp, T ki, T kd, T forward_k = 0) : pid_base_template_t<T>(kp, ki, kd), forwardfeed_k_(forward_k) {};
    pid_foward_template_t(PidBaseConfig_T<T> config, T forward_k = 0) : pid_base_template_t<T>(config), forwardfeed_k_(forward_k) {};
    T update(T contrl, bool clear_integral = false); // 在有target情况下计算
    T forwardfeed();
    T cal(T target, T contrl, bool clear_integral = false); // 计算
private:
    T forwardfeed_k_ = 0;
};

// 增量式pid
/*增量式pid中的i类似于位置式中的p,p类似与位置式中的i*/
template <typename T>
class pid_Increment_template_t : public pid_base_template_t<T>
{
public:
    pid_Increment_template_t(PidBaseConfig_T<T> config) : pid_base_template_t<T>(config) {};
    T update(T contrl);
    T cal(T target, T contrl, bool clear_integral = false);

private:
    T error_last_last;
};

template <typename T>

pid_base_template_t<T>::pid_base_template_t(T kp, T ki, T kd)
{
    reset();                // 初始化控制器
    pid_update(kp, ki, kd); // 更新PID参数
}

template <typename T>
pid_base_template_t<T>::pid_base_template_t(T kp, T ki, T kd, T out_min, T out_max)
{
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
template <typename T>
void pid_base_template_t<T>::pid_update(T kp, T ki, T kd)
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
template <typename T>
void pid_base_template_t<T>::out_limit(T out_min, T out_max)
{
    out_min_ = out_min;
    out_max_ = out_max;
}
/**
 * @brief 控制器清零重置
 * @return {*}
 * @note:
 */
template <typename T>
void pid_base_template_t<T>::reset(void)
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
template <typename T>
void pid_base_template_t<T>::target_update(T target, bool clear_integral)
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
template <typename T>
T pid_base_template_t<T>::update(T contrl)
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
    T output = kp_ * error + ki_ * error_sum + kd_ * error_delta;
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
template <typename T>
void pid_base_template_t<T>::reset_integral(void)
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
template <typename T>
// float pid_base_template_t::cal(float target, float contrl, bool clear_integral)
T pid_base_template_t<T>::cal(T target, T contrl, bool clear_integral)
{
    target_update(target, clear_integral);
    return update(contrl);
}
/**
 * @brief 输出限幅
 * @param {float} output
 * @return {*}
 * @note:
 */
template <typename T>
T pid_base_template_t<T>::output_limit(T output)
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
template <typename T>
T pid_foward_template_t<T>::update(T contrl, bool clear_integral)
{

    return this->output_limit(pid_base_template_t<T>::update(contrl) + forwardfeed());
}
/**
 * @brief 前馈量
 * @return {*}
 * @note:
 */
template <typename T>
T pid_foward_template_t<T>::forwardfeed()
{

    return forwardfeed_k_ * this->target_;
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

/**
 * @brief 前馈式pid传入目标值与当前值进行计算
 * @param {float} target    目标值
 * @param {float} contrl    实际值
 * @param {bool} clear_integral 是否清除积分
 * @return {*}
 * @note:
 */
template <typename T>
T pid_foward_template_t<T>::cal(T target, T contrl, bool clear_integral)
{
    this->target_update(target, clear_integral);
    return update(contrl);
}
/**
 * @brief 增量式pid更新控制量
 * @param {float} contrl
 * @return {*}
 * @note:
 */

template <typename T>
T pid_Increment_template_t<T>::update(T contrl)
{

    // error = this->target_ - contrl;
    this->error = this->target_ - contrl;
    T output = this->kp_ * this->error + this->ki_ * this->error_last + this->kd_ * (this->error - 2 * this->error_last + this->error_last_last);

    // 更新误差
    this->error_last_last = this->error_last;
    this->error_last = this->error;
    // 输出限幅
    if (output > this->out_max_)
    {
        output = this->out_max_;
    }
    else if (output < this->out_min_)
    {
        output = this->out_min_;
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
template <typename T>
T pid_Increment_template_t<T>::cal(T target, T contrl, bool clear_integral)
{
    this->target_update(target, clear_integral);
    return update(contrl);
}
#endif