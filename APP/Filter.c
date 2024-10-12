#include "Filter.h"
#include "gd32f30x.h"
/// @brief 一阶互补滤波器：取a=0~1,本次滤波结果 = (1-a) * 本次采样值 + a * 上次滤波结果
///                 优点：对周期性干扰具有良好的抑制作用适用于波动频率较高的场合
///                 缺点：相位滞后，灵敏度低滞后程度取决于a值大小不能消除滤波频率高于采样频率的1 / 2的干扰信号
/// @param info 滤波器参数
/// @param Value 采样值
/// @return 滤波后的值
float FirstOrderFilter(FirstOrderFilterInfo* info, float Value)
{
    return info->lastValue = (info->a * Value + info->_1_a * info->lastValue);
}

/// @brief 中值滤波：连续采样N次（N取奇数）把N次采样值按大小排列取中间值为本次有效值
///        优点：能有效克服因偶然因素引起的波动干扰；对温度、液位等变化缓慢的被测参数有良好的滤波效果
///        缺点：对流量，速度等快速变化的参数不宜
/// @param N 采样值个数
/// @param getValue 获取采样值的函数
/// @return 滤波后的值
float MiddleValueFilter(uint8_t N, float (*getValue)(void))
{
    float value_buf[N];
    uint8_t i, j, k, temp;
    for (i = 0; i < N; ++i)
        value_buf[i] = getValue( );
    for (j = 0; j < N - 1; ++j)
    {
        for (k = 0; k < N - j - 1; ++k)
        {
            //从小到大排序，冒泡法排序
            if (value_buf[k] > value_buf[k + 1])
            {
                temp = value_buf[k];
                value_buf[k] = value_buf[k + 1];
                value_buf[k + 1] = temp;
            }
        }
    }

    return value_buf[(N - 1) / 2];
}

/// @brief 平均滤波：连续取N个采样值进行算术平均运算;
///       N值较大时：信号平滑度较高，但灵敏度较低
///       N值较小时：信号平滑度较低，但灵敏度较高
///       N值的选取：一般流量，N = 12；压力：N = 4
///       优点：试用于对一般具有随机干扰的信号进行滤波。这种信号的特点是有一个平均值，信号在某一数值范围附近上下波动。
///       缺点：测量速度较慢或要求数据计算较快的实时控制不适用。
/// @param N 采样值个数
/// @param getValue 获取采样值的函数
/// @return 滤波后的值
float AverageFilter(uint8_t N, float (*getValue)(void))
{
    float sum = 0;
    uint8_t i;
    for (i = 0; i < N; ++i)
        sum += getValue( );
    return sum / N;
}

/// @brief 移动平均滤波：连续取N个采样值看成一个队列，队列的长度固定为N。每次采样到一个新数据放入队尾，并扔掉原来队首的数据(先进先出原则)。把队列中的N个数据进行算术平均运算,获得新的滤波结果。
///        N值的选取：流量，N = 12；压力：N = 4；液面，N = 4-12；温度，N = 1-4
///        优点：对周期性干扰有良好的抑制作用，平滑度高；试用于高频振荡的系统
///        缺点：灵敏度低；对偶然出现的脉冲性干扰的抑制作用较差，不适于脉冲干扰较严重的场合比较浪费RAM（改进方法，减去的不是队首的值，而是上一次得到的平均值）
/// @param info 滤波器参数
/// @return 滤波后的值
float MoveAverageFilter(MoveAverageFilterInfo* info, float NewValue)
{
    if (info->curNum < info->N)
    {
        info->curNum++;
        info->valueSum += NewValue;
        return info->lastFilterValue = info->valueSum / info->curNum;
    }
    else
    {
        info->valueSum -= info->lastFilterValue;
        info->valueSum += NewValue;
        return info->lastFilterValue = info->valueSum / info->N;
    }
}

/// @brief 限幅滑动平均滤波：相当于在滑动平均滤波的基础上增加了最大值和最小值的限制，即如果取得的采样值超出了规定的最大值和最小值，则放弃本次采样值，直到采样值在规定范围内。
///         优点：对于偶然出现的脉冲性干扰，可消除有其引起的采样值偏差。
///         缺点：比较浪费RAM,不适于脉冲干扰较严重的场合
/// @param info 滤波器参数
/// @return 滤波后的值
float LAverageFilter(LAverageFilterInfo* info)
{
    float temp = 0;
    uint8_t count = 0;
    do
    {
        if (count++ > info->N)
        {
            temp = 0;
            break;
        }
        temp = info->getValue( );
    } while (temp < info->MinValue || temp > info->MaxValue);
    if (info->curNum < info->N)
    {
        info->valueSum += info->getValue( );
        return info->lastFilterValue = info->valueSum / info->curNum;
    }
    else
    {
        info->valueSum -= info->lastFilterValue;
        info->valueSum += info->getValue( );
        return info->lastFilterValue = info->valueSum / info->N;
    }
}

/// @brief 卡尔曼滤波：根据当前的仪器"测量值" 和上一刻的 “预测量” 和 “误差”，计算得到当前的最优量，再预测下一刻的量。里面比较突出的是观点是：把误差纳入计算，而且分为预测误差和测量误差两种，通称为噪声。还有一个非常大的特点是：误差独立存在，始终不受测量数据的影响。
/// 优点：巧妙的融合了观测数据与估计数据,对误差进行闭环管理,将误差限定在一定范围。适用性范围很广，时效性和效果都很优秀。
/// 缺点：需要调参，参数的大小对滤波的效果影响较大
/// @param info 滤波器参数
/// @param curValue 当前值
/// @return 滤波后的值
float KalmanFilter(KalmanFilterInfo* info, float curValue)
{
    //预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
    info->p = info->p + info->q;
    //卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
    info->kGain = info->p / (info->p + info->r);
    //更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
    curValue = info->lastValue + (info->kGain * (curValue - info->lastValue));
    //更新协方差方程: 将本次的系统协方差付给下一次,为下一次运算准备。
    info->p = (1 - info->kGain) * info->p;
    return info->lastValue = curValue;;    //返回当前滤波值并保存
}




