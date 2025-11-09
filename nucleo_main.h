/* main.h
 * STM32 Nucleo-G0B1RE için Otonom Siyah Kablo Takip ROV Sistemi
 * Tarih: 25 Ekim 2025, 15:59 +03
 * Açıklama: UART (MAVLink), I2C (MS5837), TIM3 (PWM) ve custom tanımlamalar
 */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Kütüphane Dahil Etmeleri */
#include "stm32g0xx_hal.h" // HAL kütüphanesi
#include "mavlink/common/mavlink.h" // MAVLink v2 kütüphanesi
#include "ms5837.h" // BlueRobotics MS5837 kütüphanesi

/* Periferi Handler Tanımlamaları */
extern UART_HandleTypeDef huart2; // UART2: Jetson Nano ile iletişim (MAVLink)
extern I2C_HandleTypeDef hi2c1; // I2C1: MS5837 derinlik sensörü
extern TIM_HandleTypeDef htim3; // TIM3: Motor PWM (6 kanal)
extern DMA_HandleTypeDef hdma_usart2_rx; // UART DMA için

/* Sabit Tanımlamalar */
#define RX_BUFFER_SIZE 512 // UART DMA buffer boyutu (MAVLink için artırıldı)
#define DEPTH_BUFFER_SIZE 5 // Derinlik filtreleme için
#define MAVLINK_SYSTEM_ID 1 // Nucleo sistem ID’si (MAVLink)
#define MAVLINK_COMPONENT_ID 200 // Nucleo bileşen ID’si
#define MS5837_I2C_ADDR (0x76 << 1) // MS5837 I2C adresi
#define PWM_FREQUENCY 50 // PWM frekansı (Hz, motorlar için)
#define PWM_PERIOD 20000 // PWM periyodu (1ms = 1000us için)

// Pin Tanımlamaları
#define UART2_TX_Pin GPIO_PIN_2 // PA2: UART TX (Jetson RX)
#define UART2_RX_Pin GPIO_PIN_3 // PA3: UART RX (Jetson TX)
#define I2C1_SCL_Pin GPIO_PIN_8 // PB8: I2C SCL (MS5837)
#define I2C1_SDA_Pin GPIO_PIN_9 // PB9: I2C SDA (MS5837)
#define PWM_MOTOR1_Pin GPIO_PIN_6 // PA6: Motor 1 PWM
#define PWM_MOTOR2_Pin GPIO_PIN_7 // PA7: Motor 2 PWM
#define PWM_MOTOR3_Pin GPIO_PIN_0 // PB0: Motor 3 PWM
#define PWM_MOTOR4_Pin GPIO_PIN_1 // PB1: Motor 4 PWM
#define PWM_MOTOR5_Pin GPIO_PIN_4 // PB4: Motor 5 PWM
#define PWM_MOTOR6_Pin GPIO_PIN_5 // PB5: Motor 6 PWM

/* Global Değişkenler */
extern uint8_t rx_buffer[RX_BUFFER_SIZE]; // UART DMA buffer
extern float current_depth; // Güncel derinlik (m)
extern float cable_angle; // Kablo açısı (custom MAVLink)
extern float depth_buffer[DEPTH_BUFFER_SIZE]; // Derinlik filtreleme
extern uint8_t depth_buffer_index; // Derinlik buffer indeksi

/* Fonksiyon Prototipleri */
void SystemClock_Config(void); // Saat konfigürasyonu
void MX_GPIO_Init(void); // GPIO başlatma
void MX_USART2_UART_Init(void); // UART2 başlatma
void MX_I2C1_Init(void); // I2C1 başlatma
void MX_TIM3_Init(void); // TIM3 PWM başlatma
void MX_DMA_Init(void); // DMA başlatma
float read_ms5837_depth(void); // MS5837’den derinlik okuma
float temperature_correction(float temp); // Sıcaklık düzeltmesi (MS5837)

/* MAVLink Custom Mesaj Tanımlaması */
#define MAVLINK_MSG_ID_CUSTOM 200 // Custom mesaj ID’si (kablo açısı için)
typedef struct __mavlink_custom_t {
    float custom_data; // Kablo açısı veya başka veri
} mavlink_custom_t;
void mavlink_msg_custom_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, float custom_data);
void mavlink_msg_custom_decode(const mavlink_message_t* msg, mavlink_custom_t* custom);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */