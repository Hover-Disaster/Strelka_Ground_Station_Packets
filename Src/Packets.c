/*
 * Packets.c
 *
 *  Created on: 29 Jan. 2023
 *      Author: Angus McLennan
 */
#include "Packets.h"

void Handle_USB_Rx(uint8_t *Rx_Buffer, size_t len) {
	uint8_t header = Rx_Buffer[0];

	switch (header) {
	case STREAM_CTL_HEADER:
		Change_Target_Vector(Rx_Buffer, len);
		break;
	case STREAM_TUNE_PID_HEADER:
//		Change_PID_Gain(Rx_Buffer, len);
		break;
	case STREAM_HITL_IMU_HEADER:
		Receive_HITL_IMU_Packet(Rx_Buffer, len);
		break;
	case STREAM_HITL_BARO_HEADER:
		Receive_HITL_Baro_Packet(Rx_Buffer, len);
		break;
	case STREAM_HITL_GPS_HEADER:
		Receive_HITL_GPS_Packet(Rx_Buffer, len);
		break;
	}
}

//void Change_PID_Gain(uint8_t *Rx_Buffer, size_t len) {
//	Tune_PID_Pkt pid_pkt;
//	if (len < sizeof(pid_pkt))
//		return;
//	memcpy(&pid_pkt, Rx_Buffer, sizeof(pid_pkt));
//	pid.kp = pid_pkt.Kp;
//	pid.ki = pid_pkt.Ki;
//	pid.kd = pid_pkt.Kd;
//}

void Change_Target_Vector(uint8_t *Rx_Buffer, size_t len) {
	USB_Control_Pkt usb_ctl_pkt;
	if (len < sizeof(usb_ctl_pkt))
		return;
	memcpy(&usb_ctl_pkt, Rx_Buffer, sizeof(usb_ctl_pkt));
	desired_vec_norm.pData[0] = usb_ctl_pkt.target_vec[0];
	desired_vec_norm.pData[1] = usb_ctl_pkt.target_vec[1];
	desired_vec_norm.pData[2] = usb_ctl_pkt.target_vec[2];
}

void Receive_HITL_IMU_Packet(uint8_t *Rx_Buffer, size_t len) {
	HITL_IMU_Pkt imu_pkt;
	if (len < sizeof(imu_pkt))
		return;
	uint32_t CRC_calculated = Calculate_CRC32(Rx_Buffer, len);
	memcpy(&imu_pkt, &Rx_Buffer[1], sizeof(imu_pkt));
	if (imu_pkt.crc32 == CRC_calculated) {
#ifdef RUN_HITL
		// Copy the contents of the HITL packet into the BMX055_Data structure
		// Copy acc x, y, z and current sample time
		memcpy(bmx055_data.accel, imu_pkt.accelData, 3 * sizeof(float));
		bmx055_data.accel[4] = bmx055_data.accel[3];
		bmx055_data.accel[3] = micros(Micros_Timer);
		// Copy gyro x, y, z and current sample time
		memcpy(bmx055_data.gyro, imu_pkt.gyroData, 3 * sizeof(float));
		bmx055_data.gyro[4] = bmx055_data.gyro[3];
		bmx055_data.gyro[3] = micros(Micros_Timer);
		// Copy mag x, y, z and current sample time
		memcpy(bmx055_data.mag, imu_pkt.magData, 3 * sizeof(float));
		bmx055_data.mag[4] = bmx055_data.mag[3];
		bmx055_data.mag[3] = micros(Micros_Timer);

#endif
	}
}

void Receive_HITL_Baro_Packet(uint8_t *Rx_Buffer, size_t len) {
	HITL_Baro_Pkt baro_pkt;
	if (len < sizeof(baro_pkt))
		return;
	uint32_t CRC_calculated = Calculate_CRC32(Rx_Buffer, len);
	memcpy(&baro_pkt, &Rx_Buffer[1], sizeof(baro_pkt));
	if (baro_pkt.crc32 == CRC_calculated) {
#ifdef RUN_HITL
		// Copy the contents of the HITL packet into the ms5611_data
		ms5611_data.pressure = baro_pkt.pressure;
		ms5611_data.temperature = baro_pkt.temperature;
		ms5611_data.altitude = baro_pkt.altitude;

#endif
	}
}

void Receive_HITL_GPS_Packet(uint8_t *Rx_Buffer, size_t len) {

}

uint32_t Calculate_CRC32(uint8_t *payload_data, size_t len) {
	uint32_t *payload = (uint32_t*) malloc((len - 4) * sizeof(uint32_t));
	// Copy payload_data into uint32_t array without including the last 4 bytes (the CRC)
	for (int i = 0; i < len - sizeof(uint32_t); i++) {
		payload[i] = payload_data[i];
	}
	uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) payload, (uint32_t) len - sizeof(uint32_t));
	free(payload);
	return crc;
}
