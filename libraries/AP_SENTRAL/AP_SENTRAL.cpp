/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <assert.h>
#include <utility>

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>

#include "AP_SENTRAL.h"

//#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#define SENTRAL_I2C_ADDR								0x28

#define SENTRAL_WIA										0x90
#define SENTRAL_Device_ID								0x86
#define SENTRAL_CHIP_CONTROL_REG						0x34
#define SENTRAL_CHIP_CONTROL_CPU_RUN					0x01
#define SENTRAL_HOST_STATUS_REG							0x35
#define SENTRAL_INT_STATUS_REG							0x36
#define SENTRAL_CHIP_STATUS_REG							0x37

#define SENTRAL_ERR_REG									0x50

#define SENTRAL_PARAM_PAGE_SENSOR_CONF					5
#define SENTRAL_PARAM_ACK_REG							0x3A
#define SENTRAL_PARAM_PAGE_SELECT_REG					0x54
#define SENTRAL_PARAM_LOAD_REG							0x5C
#define SENTRAL_PARAM_REQUEST_REG						0x64

#define SENTRAL_BYTES_REMANING_REG						0x38

#define SENTRAL_RESET_REQ_REG							0x9B

//Sentral sensor type defines
#define SENSOR_TYPE_ACCELEROMETER                       1
#define SENSOR_TYPE_MAGNETIC_FIELD                      2
#define SENSOR_TYPE_ORIENTATION                         3
#define SENSOR_TYPE_GYROSCOPE                           4
#define SENSOR_TYPE_LIGHT                               5
#define SENSOR_TYPE_PRESSURE                            6
#define SENSOR_TYPE_TEMPERATURE                         7
#define SENSOR_TYPE_PROXIMITY                           8
#define SENSOR_TYPE_GRAVITY                             9
#define SENSOR_TYPE_LINEAR_ACCELERATION                 10
#define SENSOR_TYPE_ROTATION_VECTOR                     11
#define SENSOR_TYPE_RELATIVE_HUMIDITY                   12
#define SENSOR_TYPE_AMBIENT_TEMPERATURE                 13
#define SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED         14
#define SENSOR_TYPE_GAME_ROTATION_VECTOR                15
#define SENSOR_TYPE_GYROSCOPE_UNCALIBRATED              16
#define SENSOR_TYPE_SIGNIFICANT_MOTION                  17
#define SENSOR_TYPE_STEP_DETECTOR                       18
#define SENSOR_TYPE_STEP_COUNTER                        19
#define SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR         20
#define SENSOR_TYPE_HEART_RATE                          21
#define SENSOR_TYPE_TILT_DETECTOR                       22
#define SENSOR_TYPE_WAKE_GESTURE                        23
#define SENSOR_TYPE_GLANCE_GESTURE                      24
#define SENSOR_TYPE_PICK_UP_GESTURE                     25
#define SENSOR_TYPE_PDR									26
#define SENSOR_TYPE_ACTIVITY                            31
#define SENSOR_TYPE_VISIBLE_END							63


#define SENSOR_TYPE_ACCELEROMETER_WAKE                  65
#define SENSOR_TYPE_MAGNETIC_FIELD_WAKE                 66
#define SENSOR_TYPE_ORIENTATION_WAKE                    67
#define SENSOR_TYPE_GYROSCOPE_WAKE                      68
#define SENSOR_TYPE_LIGHT_WAKE                          69
#define SENSOR_TYPE_PRESSURE_WAKE                       70
#define SENSOR_TYPE_TEMPERATURE_WAKE                    71
#define SENSOR_TYPE_PROXIMITY_WAKE                      72
#define SENSOR_TYPE_GRAVITY_WAKE                        73
#define SENSOR_TYPE_LINEAR_ACCELERATION_WAKE            74
#define SENSOR_TYPE_ROTATION_VECTOR_WAKE                75
#define SENSOR_TYPE_RELATIVE_HUMIDITY_WAKE              76
#define SENSOR_TYPE_AMBIENT_TEMPERATURE_WAKE            77
#define SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED_WAKE    78
#define SENSOR_TYPE_GAME_ROTATION_VECTOR_WAKE           79
#define SENSOR_TYPE_GYROSCOPE_UNCALIBRATED_WAKE         80
#define SENSOR_TYPE_SIGNIFICANT_MOTION_WAKE             81
#define SENSOR_TYPE_STEP_DETECTOR_WAKE                  82
#define SENSOR_TYPE_STEP_COUNTER_WAKE                   83
#define SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR_WAKE    84
#define SENSOR_TYPE_HEART_RATE_WAKE                     85
#define SENSOR_TYPE_TILT_DETECTOR_WAKE                  86
#define SENSOR_TYPE_WAKE_GESTURE_WAKE                   87
#define SENSOR_TYPE_GLANCE_GESTURE_WAKE                 88
#define SENSOR_TYPE_PICK_UP_GESTURE_WAKE                89
#define SENSOR_TYPE_PDR_WAKE							90
#define SENSOR_TYPE_ACTIVITY_WAKE                       95

/** System events - SENSOR_TYPE >= 0x40 */
#define SENSOR_TYPE_DEBUG                               245
#define SENSOR_TYPE_TIMESTAMP_WAKE                      246
#define SENSOR_TYPE_TIMESTAMP_OVERFLOW_WAKE             247
#define SENSOR_TYPE_META_WAKE                           248
#define SENSOR_TYPE_RAW_GYRO                            249
#define SENSOR_TYPE_RAW_MAG                             250
#define SENSOR_TYPE_RAW_ACCEL                           251
#define SENSOR_TYPE_TIMESTAMP                           252
#define SENSOR_TYPE_TIMESTAMP_OVERFLOW                  253
#define SENSOR_TYPE_META                                254

#define SENSOR_TYPE_RAW_GYRO_ENABLE                     125
#define SENSOR_TYPE_RAW_MAG_ENABLE                      126
#define SENSOR_TYPE_RAW_ACCEL_ENABLE                    127

/* Param Page 1 = PARAMETERS */

extern const AP_HAL::HAL &hal;

ParamInfo param_orient = { SENSOR_TYPE_ORIENTATION, 2 };
ParamInfo param_baro = { SENSOR_TYPE_PRESSURE, 2 };
ParamInfo param_accel = { SENSOR_TYPE_ACCELEROMETER, 2 };
ParamInfo param_gyro = { SENSOR_TYPE_GYROSCOPE, 2 };
ParamInfo param_mag = { SENSOR_TYPE_MAGNETIC_FIELD, 2 };

// Default constructor. Empty.
AP_SENTRAL::AP_SENTRAL() {

}

//Initialize Sentral
bool AP_SENTRAL::init()
{
	//Turn on orientation sensor by setting sensor rate
	uint8_t paramPage = SENTRAL_PARAM_PAGE_SENSOR_CONF;

	//ParamInfo param = { SENSOR_TYPE_ORIENTATION, 2 };
	uint8_t rate = 200;

	hal.scheduler->suspend_timer_procs();
	AP_HAL::Semaphore *i2c_sem = hal.i2c->get_semaphore();

	if (!i2c_sem || !i2c_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
		hal.console->printf("Sentral: Unable to get bus semaphore\n");
		if (!i2c_sem) {
			hal.console->printf("bus semaphore is NULL!!!!! x.x\n");
		}
		goto fail_sem;
	}

	if (!_check_id()) {
		hal.console->printf("Sentral : Wrong id\n");
		goto fail;
	}
	
	//Enable Sentral CPU
	if (!_sentral_init()) {
		hal.console->printf("Sentral : init failed...\n");
		goto fail;
	}
	
	//Set scale later

	
	// Turn on sensor by setting rates to it
	if (!(_7186_param_write((uint8_t*)&rate, paramPage, &param_orient, 1))) {
		hal.console->printf("Sentral : param write failed...\n");
		goto fail;
	}

	// Turn on sensor by setting rates to it
	if (!(_7186_param_write((uint8_t*)&rate, paramPage, &param_baro, 1))) {
		hal.console->printf("Sentral : param write failed...\n");
		goto fail;
	}
	
	// Turn on sensor by setting rates to it
	//if (!(_7186_param_write((uint8_t*)&rate, paramPage, &param_accel, 1))) {
	//	hal.console->printf("Sentral : param write failed...\n");
	//	goto fail;
	//}
	/*/
	// Turn on sensor by setting rates to it
	if (!(_7186_param_write((uint8_t*)&rate, paramPage, &param_gyro, 1))) {
		hal.console->printf("Sentral : param write failed...\n");
		goto fail;
	}
	// Turn on sensor by setting rates to it
	if (!(_7186_param_write((uint8_t*)&rate, paramPage, &param_mag, 1))) {
		hal.console->printf("Sentral : param write failed...\n");
		goto fail;
	}
	*/
	_initialized = true;

	i2c_sem->give();
	hal.scheduler->resume_timer_procs();

	return true;

fail:
	i2c_sem->give();
fail_sem:
	hal.scheduler->resume_timer_procs();

	return false;
}


void AP_SENTRAL::read()
{
	uint8_t sensorId = 0;
	uint16_t bytes_remaining = 0;
	//Check interrupt status first


	//read bytes remaining
	if (hal.i2c->readRegisters(SENTRAL_I2C_ADDR, SENTRAL_BYTES_REMANING_REG, 2, (uint8_t*)&bytes_remaining))
		hal.console->printf("Can't read bytes remaining in FIFO...\n");

	hal.console->printf("Number of bytes in FIFO: %d\n", bytes_remaining);
	//read from reg 0 by number of available bytes
	if (hal.i2c->readRegisters(SENTRAL_I2C_ADDR, 0, bytes_remaining, fifo_buf))
		hal.console->printf("Error reading FIFO...\n");

	hal.console->printf("FIFO read into the buffer!\n");
	//parse the read data
	_7186_parse_fifo(fifo_buf, bytes_remaining);
	//check read data
	sensorId = fifo_buf[0];

	hal.console->printf("FIFO read Sensor ID: %d\n", sensorId);
	hal.console->printf("FIFO read Sensor ID: %d\n", fifo_buf[3]);

}

uint8_t AP_SENTRAL::interrupt()
{
	uint8_t i2c_read_buf = 0;
	//read int status reg
	if (hal.i2c->readRegisters(SENTRAL_I2C_ADDR, SENTRAL_INT_STATUS_REG, 1, &i2c_read_buf))
		hal.console->printf("Can't read INT STATUS REG!\n");
	//hal.console->printf("Interrupt status: %d\n", i2c_read_buf);
	return i2c_read_buf;
}

bool AP_SENTRAL::_check_id()
{
	for (int i = 0; i < 5; i++) {
		uint8_t deviceid = 0;
		if (!(hal.i2c->readRegister(SENTRAL_I2C_ADDR, SENTRAL_WIA, &deviceid))) {
			if (deviceid == SENTRAL_Device_ID) {
				hal.console->printf("Sentral Device ID: %d\n", deviceid);
				return true;
			}
		}

	}

	return false;
}

bool AP_SENTRAL::_sentral_init() {

	//Turn on orientation sensor by setting sensor rate
	uint8_t paramPage = SENTRAL_PARAM_PAGE_SENSOR_CONF;

	//ParamInfo param_orient = { SENSOR_TYPE_ORIENTATION, 2 };
	//ParamInfo param_baro = { SENSOR_TYPE_PRESSURE, 2 };

	uint8_t i2c_write_buf = 0;
	uint8_t i2c_read_buf = 0;
	uint8_t reset_buf = 1;

	//reset sentral
	
	if (hal.i2c->readRegisters(SENTRAL_I2C_ADDR, SENTRAL_RESET_REQ_REG, 1, &reset_buf))
		hal.console->printf("Sentral Reset failed...\n");
	hal.console->printf("Sentral Reset!\n");

	//wait 1 sec
	for (int i = 0; i < 60000; i++){}
	
	
	//Set 0 rate to all sensors
	// Turn off sensor by setting 0 rate to it
	if (!(_7186_param_write(0, paramPage, &param_orient, 1))) {
		hal.console->printf("Sentral : param orientation write failed...\n");
	}

	if (!(_7186_param_write(0, paramPage, &param_baro, 1))) {
		hal.console->printf("Sentral : param barometer write failed...\n");
	}

	//if (!(_7186_param_write(0, paramPage, &param_accel, 1))) {
	//	hal.console->printf("Sentral : param accel write failed...\n");
	//}
	/*
	if (!(_7186_param_write(0, paramPage, &param_gyro, 1))) {
		hal.console->printf("Sentral : param gyro write failed...\n");
	}
	if (!(_7186_param_write(0, paramPage, &param_mag, 1))) {
		hal.console->printf("Sentral : param mag write failed...\n");
	}
	*/
	if (hal.i2c->readRegisters(SENTRAL_I2C_ADDR, SENTRAL_CHIP_CONTROL_REG, 1, &i2c_read_buf))
		hal.console->printf("Sentral reg read failed...\n");

	hal.console->printf("Sentral init begin. Reg content: %d\n", i2c_read_buf);
	_7186_displayStatusRegisters();

	i2c_write_buf = SENTRAL_CHIP_CONTROL_CPU_RUN;
	if (!(hal.i2c->writeRegister(SENTRAL_I2C_ADDR, SENTRAL_CHIP_CONTROL_REG, i2c_write_buf))) {
		hal.i2c->readRegisters(SENTRAL_I2C_ADDR, SENTRAL_CHIP_CONTROL_REG, 1, &i2c_read_buf);
		hal.console->printf("Sentral init success!. Reg content: %d\n", i2c_read_buf);
		for (int i = 0; i < 100000; i++) {

		}
		// Check sentral status after turning on
		_7186_displayStatusRegisters();

		return true;
	}
	
	hal.i2c->readRegisters(SENTRAL_I2C_ADDR, SENTRAL_CHIP_CONTROL_REG, 1, &i2c_read_buf);
	hal.console->printf("Sentral init failed. Reg content: %d\n", i2c_read_buf);
	return false;
}

bool AP_SENTRAL::_7186_param_write(uint8_t *values, uint8_t page, ParamInfo *paramList, uint8_t numParams) {
	uint8_t i, paramAck, paramNum, pageSelectValue;
	uint16_t valIndex = 0;

	uint8_t i2c_read_buf = 0;

	hal.console->printf("Sentral param write start\n");

	for (i = 0; i < numParams; i++)
	{
		pageSelectValue = page | (paramList[i].size << 4);
		hal.i2c->writeRegister(SENTRAL_I2C_ADDR, SENTRAL_PARAM_PAGE_SELECT_REG, pageSelectValue);
		
		hal.i2c->writeRegister(SENTRAL_I2C_ADDR, SENTRAL_PARAM_LOAD_REG, *values);
		hal.i2c->readRegisters(SENTRAL_I2C_ADDR, SENTRAL_PARAM_LOAD_REG, 1, &i2c_read_buf);

		hal.console->printf("Param load reg value: %d\n", i2c_read_buf);

		/*
		for (j = 0; j < (uint16_t)paramList[i].size; j++) {
			uint8_t rate_value = *values >> (j * 2);
			hal.console->printf("Rate at %d: %d\n", j, rate_value);
			if (hal.i2c->writeRegister(SENTRAL_I2C_ADDR, SENTRAL_PARAM_LOAD_REG + j, rate_value)) {
				hal.console->printf("writeRegisters failed...\n");
				return false;
			}
		}
		*/
		paramNum = paramList[i].paramNo | 0x80;
		hal.i2c->writeRegister(SENTRAL_I2C_ADDR, SENTRAL_PARAM_REQUEST_REG, paramNum);
		hal.console->printf("paramNum in: %d\n", paramNum);
		/*
		do
		{
			hal.console->printf("In do while wait loop\n");
			*/
			hal.i2c->readRegisters(SENTRAL_I2C_ADDR, SENTRAL_PARAM_ACK_REG, 1, &paramAck);
			hal.console->printf("paramAck out: %d\n", paramAck);
			if (paramAck == 0x80)
			{
				hal.i2c->writeRegister(SENTRAL_I2C_ADDR, SENTRAL_PARAM_REQUEST_REG, 0);
				hal.i2c->writeRegister(SENTRAL_I2C_ADDR, SENTRAL_PARAM_PAGE_SELECT_REG, 0);

				return false;
			}
		//} while (paramAck != paramNum);
		
		valIndex += paramList[i].size;
	}

	hal.i2c->writeRegister(SENTRAL_I2C_ADDR, SENTRAL_PARAM_REQUEST_REG, 0);
	hal.i2c->writeRegister(SENTRAL_I2C_ADDR, SENTRAL_PARAM_PAGE_SELECT_REG, 0);

	return true;
}

void AP_SENTRAL::_7186_displayStatusRegisters()
{
	uint8_t buf[4];
	char str[17];
	hal.console->printf("\n------------ Displaying Status Registers -----------\n");
	hal.i2c->readRegisters(SENTRAL_I2C_ADDR, SENTRAL_HOST_STATUS_REG, 3, buf);
	hal.console->printf("Host Status:       %5u, %s\n", buf[0], strBits(&buf[0], sizeof(buf[0]), str));
	hal.console->printf("Interrupt Status:  %5u, %s\n", buf[1], strBits(&buf[1], sizeof(buf[0]), str));
	hal.console->printf("Chip Status:       %5u, %s\n", buf[2], strBits(&buf[2], sizeof(buf[0]), str));
	hal.i2c->readRegisters(SENTRAL_I2C_ADDR, SENTRAL_ERR_REG, 4, buf);
	hal.console->printf("Error Register:    %5u, %s\n", buf[0], strBits(&buf[0], sizeof(buf[0]), str));
	hal.console->printf("Interrupt State:   %5u, %s\n", buf[1], strBits(&buf[1], sizeof(buf[0]), str));
	hal.console->printf("Debug Value:       %5u, %s\n", buf[2], strBits(&buf[2], sizeof(buf[0]), str));
	hal.console->printf("Debug State:       %5u, %s\n", buf[3], strBits(&buf[3], sizeof(buf[0]), str));
	hal.i2c->readRegisters(SENTRAL_I2C_ADDR, SENTRAL_BYTES_REMANING_REG, 2, buf);
	uint16_t* v = (uint16_t*)&buf;
	hal.console->printf("Bytes Remaining:   %5u, %s\n\n", v[0], strBits(&v[0], sizeof(v[0]), str));
}

char* AP_SENTRAL::strBits(void const * const ptr, uint8_t numBytes, char* str)
{
	uint8_t *bytes = (uint8_t*)ptr;
	uint8_t i, j;
	for (i = 0; i < numBytes; i++)
	{
		for (j = 0; j < 8; j++)
		{
			str[i * 8 + (7 - j)] = bytes[(numBytes - 1) - i] & (1 << j) ? '1' : '0';
		}
	}
	str[numBytes * 8] = '\0';
	return str;
}


uint32_t AP_SENTRAL::_7186_parse_fifo(uint8_t* buffer, uint32_t size) {
	uint32_t index = 0;
	uint32_t bytesUsed;
	uint32_t bytesRemaining = size;

	if (!size) return size;

	do { 
		bytesUsed = _7186_parse_next_fifo_block(&buffer[index], bytesRemaining);
		index += bytesUsed;
		bytesRemaining -= bytesUsed;
	} while (bytesUsed > 0 && bytesRemaining > 0);

	return size - bytesRemaining;
}

uint32_t AP_SENTRAL::_7186_parse_next_fifo_block(uint8_t* buffer, uint32_t size) {
	uint8_t sensorId = buffer[0];
	uint32_t sentral_timestamp;
	uint16_t *sentral_timestampPtr;

	if (sensorId < SENSOR_TYPE_ACCELEROMETER_WAKE ||
		sensorId == SENSOR_TYPE_DEBUG ||
		sensorId == SENSOR_TYPE_TIMESTAMP ||
		sensorId == SENSOR_TYPE_TIMESTAMP_OVERFLOW ||
		sensorId == SENSOR_TYPE_META ||
		sensorId == SENSOR_TYPE_RAW_GYRO ||
		sensorId == SENSOR_TYPE_RAW_MAG ||
		sensorId == SENSOR_TYPE_RAW_ACCEL
		)
	{
		sentral_timestamp = sentral_timestampNonWake;
		sentral_timestampPtr = (uint16_t*)&sentral_timestampNonWake;
		hal.console->printf("Timestamp: %d\n", sentral_timestamp);
	}
	else
	{
		sentral_timestamp = sentral_timestampWake;
		sentral_timestampPtr = (uint16_t*)&sentral_timestampWake;
		hal.console->printf("Timestamp: %d\n", sentral_timestamp);
	}

	switch (sensorId)
	{
	case 0:
	{
		//printf("Padding: %d\n", size);
		return size;
	}
	case SENSOR_TYPE_ACCELEROMETER:
	case SENSOR_TYPE_ACCELEROMETER_WAKE:
		SensorData3Axis sensorData;
		//get_3_axis_sensor_data(&sensorData, buffer);
		//hal.console->printf("Accel: %f, %f, %f, %f\n", (double)sensorData.x, (double)sensorData.y, (double)sensorData.z, (double)sensorData.extra);
		//return 8;
	case SENSOR_TYPE_MAGNETIC_FIELD:
	case SENSOR_TYPE_MAGNETIC_FIELD_WAKE:
		//get_3_axis_sensor_data(&sensorData, buffer);
		//hal.console->printf("Mag: %f, %f, %f, %f\n", (double)sensorData.x, (double)sensorData.y, (double)sensorData.z, (double)sensorData.extra);
		//return 8;
	case SENSOR_TYPE_GYROSCOPE:
	case SENSOR_TYPE_GYROSCOPE_WAKE:
		//get_3_axis_sensor_data(&sensorData, buffer);
		//hal.console->printf("Gyro: %f, %f, %f, %f\n", (double)sensorData.x, (double)sensorData.y, (double)sensorData.z, (double)sensorData.extra);
		//return 8;
	case SENSOR_TYPE_GRAVITY:
	case SENSOR_TYPE_GRAVITY_WAKE:
	case SENSOR_TYPE_LINEAR_ACCELERATION:
	case SENSOR_TYPE_LINEAR_ACCELERATION_WAKE:
	case SENSOR_TYPE_ORIENTATION:
	case SENSOR_TYPE_ORIENTATION_WAKE:
	{
		get_3_axis_sensor_data(&sensorData, buffer);
		hal.console->printf("Orient: %f, %f, %f, %f\n", (double)sensorData.x, (double)sensorData.y, (double)sensorData.z, (double)sensorData.extra);
		return 8;
	}
	/*
	case SENSOR_TYPE_LIGHT:
	case SENSOR_TYPE_LIGHT_WAKE:
	case SENSOR_TYPE_PROXIMITY:
	case SENSOR_TYPE_PROXIMITY_WAKE:
	case SENSOR_TYPE_RELATIVE_HUMIDITY:
	case SENSOR_TYPE_RELATIVE_HUMIDITY_WAKE:
	case SENSOR_TYPE_ACTIVITY:
	case SENSOR_TYPE_ACTIVITY_WAKE:
	{
		float sensorData = (float)((buffer[2] << 8) + buffer[1]) * em7186_sensor_scale[sensorId];
		if (printData) printf("%s: %fLux\n", em7186_sensor_name[sensorId], sensorData);
		if (datalog) log_1_axis_data(sensorId, timestamp, sensorData);
		if (queueData) queue_1_axis_data(sensorId, timestamp, sensorData);
		return 3;
	}
	*/
	case SENSOR_TYPE_PRESSURE:
	case SENSOR_TYPE_PRESSURE_WAKE:
	{
		float pressure = (float)((buffer[3] << 16) + (buffer[2] << 8) + buffer[1]);// *em7186_sensor_scale[sensorId];
		hal.console->printf("Pressure: %fPa\n", pressure);
		return 4;
	}
	/*
	case SENSOR_TYPE_TEMPERATURE:
	case SENSOR_TYPE_TEMPERATURE_WAKE:
	case SENSOR_TYPE_AMBIENT_TEMPERATURE:
	case SENSOR_TYPE_AMBIENT_TEMPERATURE_WAKE:
	{
		s16 *sensorData = (s16*)&buffer[1];
		float temp = (float)(sensorData[0]) * em7186_sensor_scale[sensorId];
		if (printData) printf("%s: %fC\n", em7186_sensor_name[sensorId], temp);
		if (datalog) log_1_axis_data(sensorId, timestamp, temp);
		if (queueData) queue_1_axis_data(sensorId, timestamp, temp);
		return 3;
	}
	case SENSOR_TYPE_ROTATION_VECTOR:
	case SENSOR_TYPE_ROTATION_VECTOR_WAKE:
	case SENSOR_TYPE_GAME_ROTATION_VECTOR:
	case SENSOR_TYPE_GAME_ROTATION_VECTOR_WAKE:
	case SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR:
	case SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR_WAKE:
	case SENSOR_TYPE_PDR:
	case SENSOR_TYPE_PDR_WAKE:
	case 60:
	{
		SensorData4Axis rotationVector;
		get_rotation_vector(&rotationVector, em7186_sensor_scale[sensorId], buffer);
		if (printData) printf("%s: %f, %f, %f, %f, %f\n", em7186_sensor_name[sensorId], rotationVector.x, rotationVector.y, rotationVector.z, rotationVector.w, rotationVector.extra);
		if (datalog) log_4_axis_data(sensorId, timestamp, rotationVector);
		if (queueData) queue_4_axis_data(sensorId, timestamp, rotationVector);
		return 11;
	}
	case SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED:
	case SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED_WAKE:
	case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
	case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED_WAKE:
	{
		SensorData6Axis sensorDataUncal;
		get_3_axis_uncal_sensor_data(&sensorDataUncal, em7186_sensor_scale[sensorId], buffer);
		if (printData) printf("%s: %f, %f, %f, %f, %f, %f, %f\n", em7186_sensor_name[sensorId], sensorDataUncal.x, sensorDataUncal.y, sensorDataUncal.z, sensorDataUncal.x_bias, sensorDataUncal.y_bias, sensorDataUncal.z_bias, sensorDataUncal.extra);
		if (datalog) log_6_axis_data(sensorId, timestamp, sensorDataUncal);
		if (queueData) queue_6_axis_data(sensorId, timestamp, sensorDataUncal);
		return 14;
	}
	case SENSOR_TYPE_STEP_COUNTER:
	case SENSOR_TYPE_STEP_COUNTER_WAKE:
	{
		u16 sensorData = (buffer[2] << 8) + buffer[1];
		if (printData) printf("%s: %u\n", em7186_sensor_name[sensorId], sensorData);
		if (datalog) log_1_axis_data(sensorId, timestamp, (float)sensorData);
		if (queueData) queue_1_axis_data(sensorId, timestamp, (float)sensorData);
		return 3;
	}
	case SENSOR_TYPE_HEART_RATE:
	case SENSOR_TYPE_HEART_RATE_WAKE:
	{
		u8 sensorData = buffer[1];
		if (printData) printf("%s: %u\n", em7186_sensor_name[sensorId], sensorData);
		if (datalog) log_1_axis_data(sensorId, timestamp, (float)sensorData);
		if (queueData) queue_1_axis_data(sensorId, timestamp, (float)sensorData);
		return 2;
	}
	case SENSOR_TYPE_SIGNIFICANT_MOTION:
	case SENSOR_TYPE_SIGNIFICANT_MOTION_WAKE:
	case SENSOR_TYPE_STEP_DETECTOR:
	case SENSOR_TYPE_STEP_DETECTOR_WAKE:
	case SENSOR_TYPE_TILT_DETECTOR:
	case SENSOR_TYPE_TILT_DETECTOR_WAKE:
	case SENSOR_TYPE_PICK_UP_GESTURE:
	case SENSOR_TYPE_PICK_UP_GESTURE_WAKE:
	case SENSOR_TYPE_WAKE_GESTURE:
	case SENSOR_TYPE_WAKE_GESTURE_WAKE:
	{
		if (printData) printf("%s\n", em7186_sensor_name[sensorId]);
		if (datalog) log_0_axis_data(sensorId, timestamp);
		if (queueData) queue_0_axis_data(sensorId, timestamp);
		return 1;
	}
	*/
	case SENSOR_TYPE_TIMESTAMP:
	case SENSOR_TYPE_TIMESTAMP_WAKE:
	{
		uint16_t* uPacket = (uint16_t*)&buffer[1];
		sentral_timestampPtr[0] = uPacket[0];
		sentral_timestamp = *(uint32_t*)sentral_timestampPtr;
		//if (datalog) fprintf(datalog, "%u, %u, %u\n", sensorId, timestamp, uPacket[0]);
		hal.console->printf("Sensor Timestamp: %d\n", sentral_timestamp);

		return 3;
	}
	case SENSOR_TYPE_TIMESTAMP_OVERFLOW:
	case SENSOR_TYPE_TIMESTAMP_OVERFLOW_WAKE:
	{
		uint16_t* uPacket = (uint16_t*)&buffer[1];
		sentral_timestampPtr[1] = uPacket[0];
		sentral_timestampPtr[0] = 0;
		sentral_timestamp = *(uint32_t*)sentral_timestampPtr;
		//if (datalog) fprintf(datalog, "%u, %u, %u\n", sensorId, timestamp, uPacket[0]);
		hal.console->printf("Sensor Timestamp Overflow: %d\n", sentral_timestamp);

		return 3;
	}
	case SENSOR_TYPE_META:
	case SENSOR_TYPE_META_WAKE:
	{
		hal.console->printf("Meta: %u, %u, %u\n", buffer[1], buffer[2], buffer[3]);
		return 4;
	}
	/*
	case SENSOR_TYPE_RAW_ACCEL:
	case SENSOR_TYPE_RAW_GYRO:
	case SENSOR_TYPE_RAW_MAG:
	{
		SensorData3Axis sensorData;
		float* fPacket = (float*)&buffer[1];
		sensorData.x = fPacket[0];
		sensorData.y = fPacket[1];
		sensorData.z = fPacket[2];
		sensorData.extra = fPacket[3];
		if (printData) printf("%s: %f, %f, %f\n", em7186_sensor_name[sensorId], fPacket[0], fPacket[1], fPacket[2]);
		if (datalog) log_3_axis_data(sensorId, timestamp, sensorData);
		//if (queueData) queue_3_axis_data(sensorId, timestamp, sensorData);
		return 17;
	}
	
	case SENSOR_TYPE_DEBUG:
	{
		u8 packetSize = buffer[1] & 0x3F;
		u8 i;
		for (i = 0; i < packetSize; i++)
		{
			printf("%c", (char)buffer[2 + i]);
		}
		return 14;
	}
	*/
	default:
	{
		// Parsing error or unkown sensor type. Clear out the rest of the 
		// buffer and start clean on the next read.
		hal.console->printf("Other: %d: %d bytes skipped\n", buffer[0], size);
		/*
		if (datalog)
		{
			fprintf(datalog, "%u, %u", sensorId, timestamp);
			u16 i;
			for (i = 0; i < size; i++)
			{
				fprintf(datalog, ", %u", buffer[i]);
			}
			fprintf(datalog, "\n");
		}
		break;
		*/
	}
	} // end switch 
	return 0;
}

uint8_t AP_SENTRAL::get_3_axis_sensor_data(SensorData3Axis *data, uint8_t* buffer)
{
	SensorData3AxisRaw rawData;
	memcpy(&rawData, &buffer[1], sizeof(rawData));
	data->x = (float)rawData.x * 360.0f / 32768.0f;
	data->y = (float)rawData.y * 360.0f / 32768.0f;
	data->z = (float)rawData.z * 360.0f / 32768.0f;
	data->extra = rawData.status;

	return 1;
}