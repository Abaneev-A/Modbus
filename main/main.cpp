﻿#include <iostream>
#include <stdint.h>

#define NUMBER_REG 4
#define MAX_REG_ADR 3
#define MAX_NUMBER 1000

using namespace std;

enum param {
	PIN_STATE,
	VERSION,
	CONFIGURATION = 0x03
};

class Modbus
{
	uint8_t adress;
public:

	Modbus(uint8_t adress)
	{
		this->adress = adress;
	}
	
    void parsing(uint8_t* RX, uint8_t* TX, uint16_t* storage, uint16_t sizeRX, uint16_t &sizeTX)
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
				error_code1(RX, TX, sizeTX);
				break;
			}
		}
    }

    void code3(uint8_t* RX, uint8_t* TX, uint16_t* storage, uint16_t& sizeTX)
    {
		uint16_t num_reg = 0;
		num_reg = num_register(RX);

		if (num_reg > NUMBER_REG || num_reg < 1)
		{
			error_code3(RX, TX, sizeTX);
		}
		else
		{
			uint16_t reg_adr = 0;
			reg_adr = reg_adress(RX);

			if ((reg_adr <= MAX_REG_ADR) && ((reg_adr + num_reg) <= NUMBER_REG))
			{
				uint8_t num_byte = 0;
				num_byte = sizeof(num_reg) * num_reg;

				TX[0] = adress;
				TX[1] = 0x03;
				TX[2] = num_byte;

				uint8_t i;
				for (i = 0; i < num_reg; i++)
				{
					uint8_t value_regH[5] = { 0 };
					uint8_t value_regL[5] = { 0 };
					value_regH[i] = (uint8_t)(storage[reg_adr + i] >> 8);
					value_regL[i] = (uint8_t)(storage[reg_adr + i] & 0x00FF);
					TX[3 + 2 * i] = value_regH[i];
					TX[4 + 2 * i] = value_regL[i];
				}

				i--;

				uint16_t CRC = 0;
				CRC = crc_16(TX, 5 + 2 * i);

				TX[5 + 2 * i] = (uint8_t)(CRC >> 8);
				TX[6 + 2 * i] = (uint8_t)(CRC & 0x00FF);

				sizeTX = 7 + 2 * i;
			}
			else
			{
				error_code2(RX, TX, sizeTX);
			}
		}
    }

    void code6(uint8_t* RX, uint8_t* TX, uint16_t* storage, uint16_t& sizeTX)
    {
		uint16_t new_num = 0;
		new_num = new_number(RX);
		
		if (new_num > MAX_NUMBER)
		{
			error_code3(RX, TX, sizeTX);
		}
		else
		{
			uint16_t reg_adr = 0;
			reg_adr = reg_adress(RX);

			if (reg_adr > MAX_REG_ADR)
			{
				error_code2(RX, TX, sizeTX);
			}
			else
			{
				TX[0] = adress;
				TX[1] = 0x06;
				TX[2] = RX[2];
				TX[3] = RX[3];

				storage[reg_adr] = new_num;

				uint8_t value_regH06 = 0;
				uint8_t value_regL06 = 0;
				value_regH06 = (uint8_t)(storage[reg_adr] >> 8);
				value_regL06 = (uint8_t)(storage[reg_adr] & 0x00FF);

				TX[4] = value_regH06;
				TX[5] = value_regL06;

				uint16_t CRC = 0;
				CRC = crc_16(TX, 6);

				TX[6] = (uint8_t)(CRC >> 8);
				TX[7] = (uint8_t)(CRC & 0x00FF);

				sizeTX = 8;
			}
		}
    }

    void code10(uint8_t* RX, uint8_t* TX, uint16_t* storage, uint16_t& sizeTX)
    {
		uint16_t num_reg = 0;
		num_reg = num_register(RX);
		

		if ((num_reg > NUMBER_REG || num_reg < 1) || (RX[6] != (num_reg * 2)))
		{
			error_code3(RX, TX, sizeTX);
		}
		else
		{
			uint16_t reg_adr = 0;
			reg_adr = reg_adress(RX);

			if ((reg_adr <= MAX_REG_ADR) && ((reg_adr + num_reg) <= NUMBER_REG))
			{
				TX[0] = adress;
				TX[1] = 0x10;
				TX[2] = RX[2];
				TX[3] = RX[3];
				TX[4] = RX[4];
				TX[5] = RX[5];

				uint8_t i;
				for (i = 0; i < num_reg; i++)
				{
					uint16_t new_num = 0;
					new_num = ((uint16_t)RX[8 + 2 * i]) | (((uint16_t)RX[7 + 2 * i]) << 8);
					storage[reg_adr + i] = new_num;
				}

				uint16_t CRC = 0;
				CRC = crc_16(TX, 6);

				TX[6] = (uint8_t)(CRC >> 8);
				TX[7] = (uint8_t)(CRC & 0x00FF);

				sizeTX = 8;
			}
			else
			{
				error_code2(RX, TX, sizeTX);
			}			
		}
    }

	uint16_t crc_16(uint8_t* buffer, uint16_t buffer_size)
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

	void error_code1(uint8_t* RX, uint8_t* TX, uint16_t& sizeTX)
	{
		TX[0] = adress;
		TX[1] = (RX[1] | 0b10000000);
		TX[2] = 1;

		uint16_t CRC = 0;
		CRC = crc_16(TX, 3);

		TX[3] = (uint8_t)(CRC >> 8);
		TX[4] = (uint8_t)(CRC & 0x00FF);
		sizeTX = 5;
	}

	void error_code2(uint8_t* RX, uint8_t* TX, uint16_t& sizeTX)
	{
		TX[0] = adress;
		TX[1] = (RX[1] | 0b10000000);
		TX[2] = 2;

		uint16_t CRC = 0;
		CRC = crc_16(TX, 3);

		TX[3] = (uint8_t)(CRC >> 8);
		TX[4] = (uint8_t)(CRC & 0x00FF);
		sizeTX = 5;
	}

	void error_code3(uint8_t* RX, uint8_t* TX, uint16_t& sizeTX)
	{
		TX[0] = adress;
		TX[1] = (RX[1] | 0b10000000);
		TX[2] = 3;

		uint16_t CRC = 0;
		CRC = crc_16(TX, 3);

		TX[3] = (uint8_t)(CRC >> 8);
		TX[4] = (uint8_t)(CRC & 0x00FF);
		sizeTX = 5;
	}

	uint16_t reg_adress(uint8_t* RX)
	{
		uint16_t reg_adr = 0;
		reg_adr = ((uint16_t)RX[3]) | (((uint16_t)RX[2]) << 8);
		return reg_adr;
	}

	uint16_t num_register(uint8_t* RX)
	{
		uint16_t num_reg = 0;
		num_reg = ((uint16_t)RX[5]) | (((uint16_t)RX[4]) << 8);
		return num_reg;
	}

	uint16_t new_number(uint8_t* RX)
	{
		uint16_t new_number = 0;
		new_number = ((uint16_t)RX[5]) | (((uint16_t)RX[4]) << 8);
		return new_number;
	}
};

int main()
{
	uint16_t storage[100] = { 0 };

	storage[PIN_STATE] = 300;
	storage[VERSION] = 345;
	storage[VERSION + 1] = 654;
	storage[CONFIGURATION] = 3;

    uint8_t Modbus_RX[250] = { 0x11, 0x10, 0x00, 0x01, 0x00, 0x02, 0x04, 0x00, 0x05, 0x00, 0x05, 0x76, 0x9 };

    uint8_t Modbus_TX[250] = { 0 };
	
	//uint16_t n = 8;
	uint16_t n = 13;

	uint16_t m;
	
    Modbus master(0x11);

    master.parsing(Modbus_RX, Modbus_TX, storage, n, m);

	for (int i = 0; i < n; i++)
	{
		cout <<hex<< (short)Modbus_RX[i] << "\t";
	}
	cout << endl;

	for (int i = 0; i < m; i++)
	{
		cout << hex << (short)Modbus_TX[i] << "\t";
	}

	cout << endl << endl;

	for (int i = 0; i < 4; i++)
	{
		cout  << storage[i] << "\t";
	}

    return 0;
}