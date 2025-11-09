#include "main.h"
#include "mavlink/common/mavlink.h"
#include "ms5837.h" // BlueRobotics MS5837 kütüphanesi

// Tarih: 25 Ekim 2025, 15:52 +03
UART_HandleTypeDef huart2;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_usart2_rx; // UART DMA için
uint8_t rx_buffer[512]; // Artırılmış buffer (MAVLink için)
float current_depth = 0.0;
float cable_angle = 0.0; // Custom MAVLink mesajı için
float depth_buffer[5] = {0}; // Derinlik filtreleme için
uint8_t depth_buffer_index = 0;

// UART DMA kesmesi: MAVLink mesajlarını alır
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    static mavlink_message_t msg;
    static mavlink_status_t status;
    for (int i = 0; i < sizeof(rx_buffer); i++) {
        if (mavlink_parse_char(MAVLINK_COMM_0, rx_buffer[i], &msg, &status)) {
            if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
                mavlink_command_long_t cmd;
                mavlink_msg_command_long_decode(&msg, &cmd);
                if (cmd.command == MAV_CMD_DO_SET_SERVO) {
                    uint16_t servo_num = cmd.param1;
                    uint16_t pwm_value = cmd.param2;
                    if (servo_num >= 1 && servo_num <= 6) {
                        uint32_t channel = (servo_num == 1) ? TIM_CHANNEL_1 :
                                           (servo_num == 2) ? TIM_CHANNEL_2 :
                                           (servo_num == 3) ? TIM_CHANNEL_3 :
                                           (servo_num == 4) ? TIM_CHANNEL_4 :
                                           (servo_num == 5) ? TIM_CHANNEL_1 : TIM_CHANNEL_2;
                        uint32_t timer = (servo_num <= 4) ? (uint32_t)&htim3 : (uint32_t)&htim3;
                        __HAL_TIM_SET_COMPARE(timer, channel, pwm_value);
                    }
                }
            }
            // Custom mesaj: Kablo açısı
            else if (msg.msgid == MAVLINK_MSG_ID_CUSTOM) {
                mavlink_custom_t custom;
                mavlink_msg_custom_decode(&msg, &custom);
                cable_angle = custom.custom_data; // Jetson’dan gelen kablo açısı
            }
        }
    }
    HAL_UART_Receive_DMA(&huart2, rx_buffer, sizeof(rx_buffer)); // DMA ile yeniden başlat
}

// UART hata yönetimi (overrun, framing vb.)
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->ErrorCode & HAL_UART_ERROR_ORE) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        HAL_UART_Receive_DMA(&huart2, rx_buffer, sizeof(rx_buffer)); // Yeniden başlat
    }
}

// I2C hata yönetimi
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_NONE) {
        HAL_I2C_MspDeInit(&hi2c1);
        HAL_I2C_MspInit(&hi2c1); // Bus reset
    }
}

// MS5837 derinlik sensöründen veri okuma (filtreleme ile)
float read_ms5837_depth() {
    uint8_t data[3];
    float pressure, temperature;
    
    // BlueRobotics kütüphanesi ile kalibrasyonlu okuma
    if (ms5837_read_pressure_and_temperature(&pressure, &temperature) == HAL_OK) {
        float depth = (pressure - 1013.25) * 100 / (1025 * 9.80665 + temperature_correction(temperature));
        depth_buffer[depth_buffer_index] = depth;
        depth_buffer_index = (depth_buffer_index + 1) % 5;
        
        // 5 okuma ortalaması (gürültü azaltma)
        float sum = 0.0;
        for (int i = 0; i < 5; i++) {
            sum += depth_buffer[i];
        }
        return sum / 5.0;
    }
    return current_depth; // Hata durumunda son değeri döndür
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_USART2_UART_Init();
    MX_I2C1_Init();
    MX_TIM3_Init();
    MX_DMA_Init(); // DMA başlat
    
    // PWM kanallarını başlat
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    
    // MS5837 başlat
    ms5837_init(&hi2c1);
    
    // UART DMA ile başlat (MAVLink için düşük CPU yükü)
    HAL_UART_Receive_DMA(&huart2, rx_buffer, sizeof(rx_buffer));

    while (1) {
        // Derinlik oku ve MAVLink ile gönder
        current_depth = read_ms5837_depth();
        mavlink_message_t msg;
        mavlink_msg_scaled_pressure_pack(1, 200, &msg, 0, current_depth * 1000, 20 * 100, 0);
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        HAL_UART_Transmit(&huart2, buf, len, 100);
        
        // Heartbeat mesajı (10Hz, sistem durumu)
        mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_SUBMARINE, MAV_AUTOPILOT_GENERIC, MAV_MODE_MANUAL_ARMED, 0, MAV_STATE_ACTIVE);
        len = mavlink_msg_to_send_buffer(buf, &msg);
        HAL_UART_Transmit(&huart2, buf, len, 100);
        
        // Custom mesaj: Kablo açısı (örnek genişletme)
        mavlink_msg_custom_pack(1, 200, &msg, cable_angle);
        len = mavlink_msg_to_send_buffer(buf, &msg);
        HAL_UART_Transmit(&huart2, buf, len, 100);
        
        HAL_Delay(100); // 10Hz döngü
    }
}