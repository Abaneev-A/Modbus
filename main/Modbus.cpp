#include "Modbus.h"

Modbus::Modbus(uint8_t address)
{
	this->address = address;
}

void Modbus::parsing(uint8_t* RX, uint8_t* TX, uint16_t* storage, uint16_t sizeRX, uint16_t& sizeTX)
{
	if (1)//crc_16(RX, sizeRX) == 0
	{
		switch (RX[1])
		{
		case 0x03:
			code3(RX, TX, storage, sizeTX);
			break;
		case 0x06:
			code6(RX, TX, storage, sizeTX);
			break;
		case 0x10:
			code10(RX, TX, storage, sizeTX);
			break;
		default:
			error(RX, TX, sizeTX, 1);
			break;
		}
	}
}

void Modbus::code3(uint8_t* RX, uint8_t* TX, uint16_t* storage, uint16_t& sizeTX)
{
	uint16_t num_reg = num_register(RX);

	if (num_reg > NUMBER_REG || num_reg < 1)
	{
		error(RX, TX, sizeTX, 3);
		return;
	}

	uint16_t reg_adr = reg_adress(RX);

	if ((reg_adr > MAX_REG_ADR) || ((reg_adr + num_reg) > NUMBER_REG))
	{
		error(RX, TX, sizeTX, 2);
		return;
	}

	uint8_t num_byte = sizeof(num_reg) * num_reg;

	TX[0] = address;
	TX[1] = 0x03;
	TX[2] = num_byte;

	uint8_t index = 0;

	for (uint8_t i = 0; i < num_reg; i++)
	{
		TX[3 + 2 * i] = get_high_byte(storage[reg_adr + i]);
		TX[4 + 2 * i] = get_low_byte(storage[reg_adr + i]);

		index = i;
	}

	uint16_t CRC = crc_16(TX, 5 + 2 * index);

	TX[5 + 2 * index] = get_high_byte(CRC);
	TX[6 + 2 * index] = get_low_byte(CRC);

	sizeTX = 7 + 2 * index;
}

void Modbus::code6(uint8_t* RX, uint8_t* TX, uint16_t* storage, uint16_t& sizeTX)
{
	uint16_t new_num = new_number(RX);

	if (new_num > MAX_NUMBER)
	{
		error(RX, TX, sizeTX, 3);
		return;
	}

	uint16_t reg_adr = reg_adress(RX);

	if (reg_adr > MAX_REG_ADR)
	{
		error(RX, TX, sizeTX, 2);
		return;
	}

	TX[0] = address;
	TX[1] = 0x06;
	TX[2] = RX[2];
	TX[3] = RX[3];

	storage[reg_adr] = new_num;

	TX[4] = get_high_byte(storage[reg_adr]);
	TX[5] = get_low_byte(storage[reg_adr]);

	uint16_t CRC = crc_16(TX, 6);

	TX[6] = get_high_byte(CRC);
	TX[7] = get_low_byte(CRC);

	sizeTX = 8;
}

void Modbus::code10(uint8_t* RX, uint8_t* TX, uint16_t* storage, uint16_t& sizeTX)
{
	uint16_t num_reg = num_register(RX);

	if ((num_reg > NUMBER_REG || num_reg < 1) || (RX[6] != (num_reg * 2)))
	{
		error(RX, TX, sizeTX, 3);
		return;
	}

	uint16_t reg_adr = reg_adress(RX);

	if ((reg_adr > MAX_REG_ADR) || ((reg_adr + num_reg) > NUMBER_REG))
	{
		error(RX, TX, sizeTX, 2);
		return;
	}

	TX[0] = address;
	TX[1] = 0x10;
	TX[2] = RX[2];
	TX[3] = RX[3];
	TX[4] = RX[4];
	TX[5] = RX[5];

	uint8_t i;
	for (i = 0; i < num_reg; i++)
	{
		storage[reg_adr + i] = get_word(RX[7 + 2 * i], RX[8 + 2 * i]);
	}

	uint16_t CRC = crc_16(TX, 6);

	TX[6] = get_high_byte(CRC);
	TX[7] = get_low_byte(CRC);

	sizeTX = 8;
}

uint16_t Modbus::crc_16(uint8_t* buffer, uint16_t buffer_size)
{
	uint8_t temp = 0;
	uint16_t crc = 0xFFFF;

	for (uint16_t byte = 0; byte < buffer_size; byte++)
	{
		crc = crc ^ buffer[byte];

		for (uint8_t j = 0; j < 8; j++)
		{
			temp = crc & 0x0001;
			crc = crc >> 1;

			if (temp)
			{
				crc = crc ^ 0xA001;
			}
		}
	}

	temp = crc & 0x00FF;
	crc = (crc >> 8) | (temp << 8);

	return crc;
}

void Modbus::error(uint8_t* RX, uint8_t* TX, uint16_t& sizeTX, uint16_t exception_code)
{
	TX[0] = address;
	TX[1] = (RX[1] | 0b10000000);
	TX[2] = exception_code;

	uint16_t CRC = crc_16(TX, 3);

	TX[3] = get_high_byte(CRC);
	TX[4] = get_low_byte(CRC);
	sizeTX = 5;
}

uint16_t Modbus::reg_adress(uint8_t* RX)
{
	return get_word(RX[2], RX[3]);
}

uint16_t Modbus::num_register(uint8_t* RX)
{
	return get_word(RX[4], RX[5]);
}

uint16_t Modbus::new_number(uint8_t* RX)
{
	return get_word(RX[4], RX[5]);
}

uint8_t Modbus::get_high_byte(uint16_t word)
{
	return word >> 8;
}

uint8_t Modbus::get_low_byte(uint16_t word)
{
	return word & 0x00FF;
}

uint16_t Modbus::get_word(uint8_t high, uint8_t low)
{
	return  (((uint16_t)high) << 8) | ((uint16_t)low);
}