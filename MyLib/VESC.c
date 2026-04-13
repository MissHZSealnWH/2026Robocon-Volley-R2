#include "vesc.h"

/**
 * @brief 以电压输入模式控制电机
 * @param vesc VESC句柄
 * @param percentage 电源电压的百分数 取值范围为-100~100
 * @return CAN数据发送情况
 * @note 无
 */
uint32_t VESC_SetVoltage(VESC_t *vesc, float percentage)
{
    uint8_t buffer[4];
    int32_t temp = (int32_t)(percentage * 1000.0f);

    buffer[0] = temp >> 24;
    buffer[1] = temp >> 16;
    buffer[2] = temp >> 8;
    buffer[3] = temp;

    FDCAN_TxHeaderTypeDef head;

    head.Identifier = (CAN_PACKET_SET_DUTY << 8) | vesc->motor_id; // 扩展ID
    head.IdType = FDCAN_EXTENDED_ID;        // 扩展帧
    head.TxFrameType = FDCAN_DATA_FRAME;    // 数据帧
    head.DataLength = FDCAN_DLC_BYTES_4;    // 4字节

    head.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    head.BitRateSwitch = FDCAN_BRS_OFF;     // 经典CAN
    head.FDFormat = FDCAN_CLASSIC_CAN;      // ?? 必须
    head.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    head.MessageMarker = 0;

    return HAL_FDCAN_AddMessageToTxFifoQ(vesc->hfdcan, &head, buffer);
}


/**
 * @brief 以电流输入模式控制电机
 * @param vesc VESC句柄
 * @param ampere 期望电流(安培)
 * @return CAN数据发送情况
 * @note 无
 */
uint32_t VESC_SetCurrent(VESC_t *vesc,float ampere)
{
    FDCAN_TxHeaderTypeDef head;
    uint8_t buffer[4];
    int32_t temp = (int32_t)(ampere * 1000.0f);

    // ===== 原数据打包（不动）=====
    buffer[0] = temp >> 24;
    buffer[1] = temp >> 16;
    buffer[2] = temp >> 8;
    buffer[3] = temp;

    // ===== 只改这里（CAN头）=====
    head.Identifier = (CAN_PACKET_SET_CURRENT << 8) | vesc->motor_id;
    head.IdType = FDCAN_EXTENDED_ID;
    head.TxFrameType = FDCAN_DATA_FRAME;
    head.DataLength = FDCAN_DLC_BYTES_4;

    head.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    head.BitRateSwitch = FDCAN_BRS_OFF;
    head.FDFormat = FDCAN_CLASSIC_CAN;
    head.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    head.MessageMarker = 0;

    // ===== 只改发送函数 =====
    return HAL_FDCAN_AddMessageToTxFifoQ(vesc->hfdcan, &head, buffer);
}

/**
 * @brief 设置刹车电流
 * @param vesc VESC句柄
 * @param ampere 期望刹车电流(安培)
 * @return CAN数据发送情况
 * @note 无
 */
uint32_t VESC_SetBreakCur(VESC_t *vesc,float ampere)
{
    FDCAN_TxHeaderTypeDef head;
    uint8_t buffer[4];
    int32_t temp = (int32_t)(ampere * 1000.0f);

    // 原数据打包（不动）
    buffer[0] = temp >> 24;
    buffer[1] = temp >> 16;
    buffer[2] = temp >> 8;
    buffer[3] = temp;

    // 只改CAN头
    head.Identifier = (CAN_PACKET_SET_CURRENT_BRAKE << 8) | vesc->motor_id;
    head.IdType = FDCAN_EXTENDED_ID;
    head.TxFrameType = FDCAN_DATA_FRAME;
    head.DataLength = FDCAN_DLC_BYTES_4;

    head.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    head.BitRateSwitch = FDCAN_BRS_OFF;
    head.FDFormat = FDCAN_CLASSIC_CAN;
    head.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    head.MessageMarker = 0;

    // 只改发送函数
    return HAL_FDCAN_AddMessageToTxFifoQ(vesc->hfdcan, &head, buffer);
}

/**
 * @brief 以速度输入模式控制电机
 * @param vesc VESC句柄
 * @param rpm 期望的圈数/分（或电圈数/分）
 * @return CAN数据发送情况
 * @note 无
 */
uint32_t VESC_SetRPM(VESC_t *vesc,int32_t rpm)
{
    FDCAN_TxHeaderTypeDef head;
    uint8_t buffer[4] = {
        (rpm >> 24) & 0xFF,
        (rpm >> 16) & 0xFF,
        (rpm >> 8)  & 0xFF,
        (rpm)       & 0xFF
    };

    // 只改CAN头
    head.Identifier = (CAN_PACKET_SET_RPM << 8) | vesc->motor_id;
    head.IdType = FDCAN_EXTENDED_ID;
    head.TxFrameType = FDCAN_DATA_FRAME;
    head.DataLength = FDCAN_DLC_BYTES_4;

    head.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    head.BitRateSwitch = FDCAN_BRS_OFF;
    head.FDFormat = FDCAN_CLASSIC_CAN;
    head.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    head.MessageMarker = 0;

    // 只改发送函数
    return HAL_FDCAN_AddMessageToTxFifoQ(vesc->hfdcan, &head, buffer);
}


/**
 * @brief 以位置输入模式控制电机
 * @param vesc VESC句柄
 * @param pos 期望位置
 * @return CAN数据发送情况
 * @note 无
 */
uint32_t VESC_SetPosition(VESC_t *vesc,int32_t pos)
{
    FDCAN_TxHeaderTypeDef head;
    uint8_t buffer[4] = {
        (pos >> 24) & 0xFF,
        (pos >> 16) & 0xFF,
        (pos >> 8)  & 0xFF,
        (pos)       & 0xFF
    };

    // 只改CAN头
    head.Identifier = (CAN_PACKET_SET_POS << 8) | vesc->motor_id;
    head.IdType = FDCAN_EXTENDED_ID;
    head.TxFrameType = FDCAN_DATA_FRAME;
    head.DataLength = FDCAN_DLC_BYTES_4;

    head.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    head.BitRateSwitch = FDCAN_BRS_OFF;
    head.FDFormat = FDCAN_CLASSIC_CAN;
    head.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    head.MessageMarker = 0;

    // 只改发送函数
    return HAL_FDCAN_AddMessageToTxFifoQ(vesc->hfdcan, &head, buffer);
}


/**
 * @brief 检查CAN中断中获取到的CAN包是否可以被接收
 * @param vesc VESC句柄
 * @param hcan 包来自的CAN外设句柄
 * @param ID CAN包ID
 * @param buf CAN包
 * @return CAN数据发送情况
 * @note 无
 */
uint32_t VESC_ReceiveHandler(VESC_t *vesc,FDCAN_HandleTypeDef *hfdcan,uint32_t ID,uint8_t *buf)
{
    if(hfdcan->Instance!=vesc->hfdcan->Instance)    //来自正确的CAN外设
        return 0;
    else if((ID&0x000000FF)!=vesc->motor_id)      //来自正确的设备
        return 0;
    uint8_t type=((ID>>8)&0xFF);
    if(type==CAN_PACKET_STATUS)    //是反馈包1
    {
        vesc->epm=(int32_t)(((uint32_t)buf[0])<<24)|(((uint32_t)buf[1])<<16)|(((uint32_t)buf[2])<<8)|(((uint32_t)buf[3]));
        vesc->current=((int16_t)((((uint16_t)buf[4])<<8)|(((uint16_t)buf[5]))))*0.1;
        vesc->duty_cycle=((int16_t)((((uint16_t)buf[6])<<8)|(((uint16_t)buf[7]))))*0.001;
    }
    else if(type==CAN_PACKET_STATUS_2)
    {
        vesc->power.consume_ah=((int32_t)((((uint32_t)buf[0])<<24)|(((uint32_t)buf[1])<<16)|(((uint32_t)buf[2])<<8)|(((uint32_t)buf[3]))))*0.0001;
        vesc->power.back_ah=((int32_t)((((uint32_t)buf[4])<<24)|(((uint32_t)buf[5])<<16)|(((uint32_t)buf[6])<<8)|(((uint32_t)buf[7]))))*0.0001;
    }
    else if(type==CAN_PACKET_STATUS_3)
    {
        vesc->power.consume_wh=((int32_t)((((uint32_t)buf[0])<<24)|(((uint32_t)buf[1])<<16)|(((uint32_t)buf[2])<<8)|(((uint32_t)buf[3]))))*0.0001;
        vesc->power.back_wh=((int32_t)((((uint32_t)buf[4])<<24)|(((uint32_t)buf[5])<<16)|(((uint32_t)buf[6])<<8)|(((uint32_t)buf[7]))))*0.0001;
    }
    else if(type==CAN_PACKET_STATUS_4)
    {
        vesc->state.mos_temp=((int16_t)((((uint16_t)buf[0])<<8)|(((uint16_t)buf[1]))))*0.1;
        vesc->state.motor_temp=((int16_t)((((uint16_t)buf[2])<<8)|(((uint16_t)buf[3]))))*0.1;
        vesc->state.input_cur=((int16_t)((((uint16_t)buf[4])<<8)|(((uint16_t)buf[5]))))*0.1;
        vesc->state.pid_value=((int16_t)((((uint16_t)buf[6])<<8)|(((uint16_t)buf[7]))))*0.02;
    }
    else if(type==CAN_PACKET_STATUS_5)
    {
        vesc->state.estimate_vel=((int32_t)((((uint32_t)buf[0])<<24)|(((uint32_t)buf[1])<<16)|(((uint32_t)buf[2])<<8)|(((uint32_t)buf[3]))));
        vesc->state.input_vol=((int16_t)((((uint16_t)buf[4])<<8)|(((uint16_t)buf[5]))))*0.1;
    }
    else
    {
		return 0;
    }
    return 1;
}
