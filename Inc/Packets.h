/*
 * Packets.h
 *
 *  Created on: 29 Jan. 2023
 *      Author: Angus McLennan
 */

#ifndef INC_PACKETS_H_
#define INC_PACKETS_H_

#include <stdint.h>
#include <stdlib.h>
#include "arm_math.h"
#include "Controller.h"
#include "Testing.h"
#include "Sensors.h"

#ifdef STM32F4
	#include "stm32f4xx_hal.h"
#elif defined (STM32H7)
	#include "stm32h7xx_hal.h"
#else
#error "STM32 series not defined"
#endif

/* Enable Hardware In The Loop by uncommenting */
//#define RUN_HITL

#define MAX_PKT_SIZE							0xFF

#define RF_CALIBRATE_SENORS						0x01
#define RF_TEST_GIMBAL							0x02
#define RF_REQUEST_CONTINUITY					0x03
#define RF_REQUEST_BATT_VOLT					0x04
#define RF_REQUEST_STREAM_PACKET				0x05
#define RF_FIRE_DROGUE_REQ						0x06
#define RF_FIRE_DROGUE_ACK						0x07
#define RF_FIRE_DROGUE_CMD						0x08
#define RF_FIRE_MAIN_REQ						0x09
#define RF_FIRE_MAIN_ACK						0x0A
#define RF_FIRE_MAIN_CMD						0x0B
#define RF_ACTUATE_VALVE_REQ					0x0C
#define RF_ACTUATE_VALVE_ACK					0x0D
#define RF_ACTUATE_VALVE_CMD					0x0E
#define STREAM_DATA_HEADER0						0xAA
#define STREAM_DATA_HEADER1						0xBB
#define STREAM_DATA_HEADER2						0xAA
#define	STREAM_DATA_HEADER3						0xBB

#define STREAM_CTL_HEADER						0x01
#define STREAM_TUNE_PID_HEADER					0x02
#define STREAM_HITL_IMU_HEADER					0x0A
#define STREAM_HITL_BARO_HEADER					0x0B
#define STREAM_HITL_GPS_HEADER					0x0C

enum event_cmd_code {
	ready,
	not_ready,
};

typedef struct {
	float Kp;
	float Ki;
	float Kd;
	uint32_t checksum;
} Tune_PID_Pkt;

typedef struct {
	float target_vec[3];		// Target TVC vector
} USB_Control_Pkt;

typedef struct {
	float accelData[3];
	float gyroData[3];
	float magData[3];
	uint32_t crc32;
} HITL_IMU_Pkt;

typedef struct {
	float pressure;
	float temperature;
	float altitude;
	uint32_t crc32;
} HITL_Baro_Pkt;

typedef struct {
	uint8_t identifier;
	uint8_t payload_length;
	float battery_voltage;
	uint32_t crc32;
} Battery_Voltage_Pkt;

typedef struct {
	uint8_t identifier;
	uint8_t payload_length;
	uint8_t result;
	uint32_t crc32;
} Calibrate_Sensors_Pkt;

typedef struct {
	uint8_t identifier;
	uint8_t payload_length;
	enum event_cmd_code ack;
	uint32_t crc32;
} Event_Cmd_Ack_Pkt;

extern PID pid;			// Bring pid struct in so it can be modified
extern arm_matrix_instance_f32 desired_vec_norm;
extern CRC_HandleTypeDef hcrc;
extern testing_configuration_t test_config;
extern BMX055_Data_Handle bmx055_data;
extern MS5611_Data_Handle ms5611_data;
// Below defines are used for system microsecond timer. Need to be defined in main to be used
// Be sure the set the timer chosen as the uS timer correctly below
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef *Micros_Timer;
extern uint32_t micros(TIM_HandleTypeDef *timer);

/*	Ground Station Packet Definitions  */
//void handle_lora_packet(uint8_t *data, size_t len);
//void calibrate_sensor_response_packet(uint8_t* data, size_t len, uint8_t calibration_state);
/*	USB Interface Definitions  */
void Handle_USB_Rx(uint8_t* Rx_Buffer, size_t len);
void Change_PID_Gain(uint8_t* Rx_Buffer, size_t len);
void Change_Target_Vector(uint8_t* Rx_Buffer, size_t len);
void Receive_HITL_IMU_Packet(uint8_t* Rx_Buffer, size_t len);
void Receive_HITL_Baro_Packet(uint8_t* Rx_Buffer, size_t len);
void Receive_HITL_GPS_Packet(uint8_t* Rx_Buffer, size_t len);
uint32_t Calculate_CRC32(uint8_t *payload_data, size_t len);

/*	Power Board Packet Definitions  */

#endif /* INC_PACKETS_H_ */
