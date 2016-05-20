/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

struct SensorData3Axis
{
	float x;
	float y;
	float z;
	float extra;
};

struct SensorData3AxisRaw
{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t status;
};

struct ParamInfo
{
	uint8_t paramNo;
	uint8_t size;
};

class AP_SENTRAL
{
public:
	AP_SENTRAL();
    bool init();
    void read();
	uint8_t interrupt();

private:
	uint8_t fifo_buf[24576];

	uint32_t sentral_timestampNonWake;
	uint32_t sentral_timestampWake;

    bool _check_id();
	bool _sentral_init();

	bool _7186_param_write(uint8_t *values, uint8_t page, ParamInfo *paramList, uint8_t numParams);

	uint32_t _7186_parse_fifo(uint8_t* buffer, uint32_t size);
	uint32_t _7186_parse_next_fifo_block(uint8_t* buffer, uint32_t size);
	void _7186_displayStatusRegisters();
	char* strBits(void const * const ptr, uint8_t numBytes, char* str);

	uint8_t get_3_axis_sensor_data(SensorData3Axis *data, uint8_t* buffer);

    uint32_t _dev_id;

    bool _initialized;
    bool _timesliced;
};
