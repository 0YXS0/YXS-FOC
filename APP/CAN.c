#include "CAN.h"
#include "Command.h"

/// @brief 初始化CAN0
/// @param nodeID 节点ID
void CAN0_config(uint8_t nodeID)
{
    rcu_periph_clock_enable(RCU_CAN0);
    rcu_periph_clock_enable(RCU_GPIOA);

    // 初始化GPIO
    gpio_init(GPIOA, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_11);	// CAN_RX
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);	// CAN_TX

    // 配置CAN0
    can_deinit(CAN0);
    can_parameter_struct can_parameter;
    can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);
    can_parameter.working_mode = CAN_NORMAL_MODE;   // 正常模式
    can_parameter.auto_retrans = DISABLE;   // 禁止自动重传
    can_parameter.auto_bus_off_recovery = ENABLE;   // 自动总线关闭恢复
    can_parameter.auto_wake_up = ENABLE;   // 开启自动唤醒
    can_parameter.rec_fifo_overwrite = DISABLE;  // 禁止接收FIFO溢出时覆盖
    can_parameter.trans_fifo_order = ENABLE;    // 使能发送FIFO优先级
    can_parameter.time_triggered = DISABLE;  // 禁止时间触发模式
    can_parameter.prescaler = 3U;    // 波特率分频器
    can_parameter.resync_jump_width = CAN_BT_SJW_1TQ;    // 同步跳转宽度
    can_parameter.time_segment_1 = CAN_BT_BS1_14TQ;  // 时间段1
    can_parameter.time_segment_2 = CAN_BT_BS2_5TQ;   // 时间段2
    can_init(CAN0, &can_parameter);

    // 配置过滤器
    can_filter_parameter_struct can_filter_param_struct;
    can_filter_param_struct.filter_list_high = nodeID << 10;  // 预设ID高位(节点ID)
    can_filter_param_struct.filter_list_low = 0x3F << 10;   // 预设ID低位(广播ID)
    can_filter_param_struct.filter_mask_high = 0x3F << 10;  // 掩码数高位
    can_filter_param_struct.filter_mask_low = 0x3F << 10;   // 掩码数低位
    can_filter_param_struct.filter_fifo_number = CAN_FIFO0; // 滤过器关联FIFO0
    can_filter_param_struct.filter_number = 0;  // 过滤器编号0
    can_filter_param_struct.filter_mode = CAN_FILTERMODE_MASK;  // 掩码模式
    can_filter_param_struct.filter_bits = CAN_FILTERBITS_16BIT; // 16位
    can_filter_param_struct.filter_enable = ENABLE;
    can_filter_init(&can_filter_param_struct);

    can_interrupt_enable(CAN0, CAN_INT_RFNE0);  // 使能接收FIFO0非空中断
    nvic_irq_enable(USBD_LP_CAN0_RX0_IRQn, 1, 1);
}

/// @brief 设置CANOpen数据发送函数
/// @param can_periph CAN外设
/// @param m CAN数据帧
/// @return 成功返回0，失败返回<0
int8_t canSend(uint32_t can_periph, can_trasnmit_message_struct* m)
{
    uint8_t mailbox_number = CAN_MAILBOX0;
    uint16_t TimeOUT = 0x0FFF;
    mailbox_number = can_message_transmit(can_periph, m); // 发送CAN数据帧
    while (can_transmit_states(can_periph, mailbox_number) == CAN_TRANSMIT_PENDING) // 等待发送完成
    {
        TimeOUT--;
        if (TimeOUT == 0) return -1;  // 发送超时
    }
    return can_transmit_states(can_periph, mailbox_number) == CAN_TRANSMIT_OK ? 0 : -2; // 返回发送状态
}

/// @brief CAN0中断服务函数
void USBD_LP_CAN0_RX0_IRQHandler(void)
{
    static can_receive_message_struct msg;
    if (can_interrupt_flag_get(CAN0, CAN_INT_FLAG_RFL0) == SET)
    {
        can_message_receive(CAN0, CAN_FIFO0, &msg); // 读取CAN数据帧
        CanCommandAnalyze(&msg);    // CAN命令解析
    }
}