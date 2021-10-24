#include <iostream>
#include <stdint.h>
#include "Modbus.h"

using namespace std;

enum param {
	PIN_STATE,
	VERSION,
	CONFIGURATION = 0x03
};

int main()
{
	uint16_t storage[100] = { 0 };

	storage[PIN_STATE] = 300;
	storage[VERSION] = 345;
	storage[VERSION + 1] = 654;
	storage[CONFIGURATION] = 3;

    uint8_t Modbus_RX[250] = { 0x11, 0x10, 0x00, 0x04, 0x00, 0x02, 0x4, 0x00, 0xff, 0x00, 0xff};

    uint8_t Modbus_TX[250] = { 0 };
	
	//uint16_t n = 8;
	uint16_t n = 13;

	uint16_t m;
	
    Modbus master(0x11);

	uint16_t crc = master.crc_16(Modbus_RX, n - 2);

	Modbus_RX[n - 2] = (uint8_t)(crc >> 8);

	Modbus_RX[n - 1] = (uint8_t)(crc & 0x00FF);

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