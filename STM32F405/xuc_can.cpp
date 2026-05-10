#include "xuc_can.h"
#include "control.h"
#include "judgement.h"
#include "imu.h"
#include "CRC.h"
#include <string.h>
#include <cmath>

extern int CNT_1; // 全局系统计时（ms）

void XUC::Init(UART* huart, USART_TypeDef* Instance, uint32_t BaudRate)
{
    m_uart = huart;
    if (m_uart != nullptr) {
        m_uart->Init(Instance, BaudRate).DMARxInit(nullptr);
        queueHandler = &m_uart->UartQueueHandler;
    }
    else {
        queueHandler = NULL;
    }

    m_canQueue = NULL;
    target = {};
    fire_auto = 0;
}

void XUC::InitCAN(QueueHandle_t canQueue, uint16_t can_id, float scale)
{
    m_uart = nullptr;
    queueHandler = NULL;
    m_canQueue = canQueue;
    m_can_id = can_id;
    m_scale = scale;
}

void XUC::Decode()
{
    static uint32_t last_rx_tick = 0;
    constexpr uint32_t RX_TIMEOUT_MS = 200;

    if (queueHandler == NULL || *queueHandler == NULL) {
        return;
    }

    // 只在该周期真正收到数据时才处理
    if (xQueueReceive(*queueHandler, frame, 0) != pdPASS) {
        // 无新数据：检查是否超时，超时则切回 POS
        if (CNT_1 - last_rx_tick > RX_TIMEOUT_MS) {
            if (&can1_motor[7] != nullptr) {
                can1_motor[7].mode = POS;
            }
        }
        return;
    }

    // 从 DMA 缓冲区中扫描有效的 RxPacket_TJ
    const uint32_t packLen = (uint32_t)sizeof(RxPacket_TJ);
    for (uint32_t i = 0; i + packLen <= UART_MAX_LEN; ++i)
    {
        if (frame[i] != 'S' || frame[i + 1] != 'P') {
            continue;
        }

        uint8_t packet[sizeof(RxPacket_TJ)]{};
        memcpy(packet, &frame[i], sizeof(RxPacket_TJ));

        if (!VerifyCRC16CheckSum(packet, (uint32_t)sizeof(RxPacket_TJ))) {
            continue;
        }

        memcpy(&Rx_TJ, packet, sizeof(RxPacket_TJ));

        target.yaw = Rx_TJ.yaw_TJ;
        target.pitch = Rx_TJ.pitch_TJ;
        fire_auto = Rx_TJ.shoot_TJ;

        if (Rx_TJ.control_TJ == 0) {
            target.yaw = 0.0f;
            target.pitch = 0.0f;
        }

        // 收到有效数据：仅当包内 IMU 数据有效时才切 POS_IMU
        last_rx_tick = CNT_1;
        if (&can1_motor[7] != nullptr && IMUDataValid()) {
            can1_motor[7].mode = POS_IMU;
        }

        return;
    }
}

    // 保留 XUC CAN 通讯接口（当前改为串口，暂不启用）
    // if (m_canQueue == NULL) return;

    // CanRxMsg_t msg{};
    // CanRxMsg_t last{};
    // bool got = false;

    // while (xQueueReceive(m_canQueue, &msg, 0) == pdTRUE) {
    //     if (msg.id == m_can_id && msg.dlc == 8) {
    //         last = msg;
    //         got = true;
    //     }
    // }

    // if (!got) return;

    // int16_t y_q = rd_i16_le(&last.data[0]);
    // int16_t p_q = rd_i16_le(&last.data[2]);
    // fire_auto   = last.data[4];
    // target.yaw = (float)y_q / m_scale;
    // target.pitch = (float)p_q / m_scale;
//}

bool XUC::IMUDataValid()
{
    const float yaw   = Rx_TJ.imu_yaw_TJ;
    const float pitch = Rx_TJ.imu_pitch_TJ;

    // 两个都是 0 → 上位机没有陀螺仪数据
    if (yaw == 0.0f && pitch == 0.0f) return false;

    // NaN / Inf 检查
    if (!std::isfinite(yaw) || !std::isfinite(pitch)) return false;

    return true;
}

void XUC::Encode()
{
    if (m_uart == nullptr) {
        return;
    }

    static uint16_t bullet_count = 0;

    memset(&Tx_TJ, 0, sizeof(Tx_TJ));

    Tx_TJ.head[0] = 'S';
    Tx_TJ.head[1] = 'P';

    // 模式：0 空闲，1 自瞄，2 小符，3 大符
    Tx_TJ.mode_TJ = (ctrl.mode == CONTROL::AUTOAIM) ? 1 : 0;

    // 机器人ID
    Tx_TJ.robot_id = judgement.data.robot_status_t.robot_id;

    // 使用裁判系统弹速
    const float bullet_speed = judgement.data.shoot_data_t.bullet_speed;
    Tx_TJ.bullet_speed_TJ = std::isfinite(bullet_speed) ? bullet_speed : 0.0f;
    Tx_TJ.bullet_count_TJ = bullet_count++;

    // 计算数据包的总大小
    const uint32_t packet_size = (uint32_t)sizeof(TxPacket_TJ);

    // 将数据包复制到发送缓冲区并附加 CRC16（与上位机一致）
    memcpy(tx_data, &Tx_TJ, packet_size);
    AppendCRC16CheckSum(tx_data, packet_size);

    // 同步回结构体，便于调试观察 crc 字段
    memcpy(&Tx_TJ, tx_data, packet_size);

    // 发送数据
    m_uart->UARTTransmit(tx_data, packet_size);
}