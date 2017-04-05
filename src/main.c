#include "lib.h"

void main()
{
	int result;
	serialPort_t* port = (serialPort_t*)(malloc(sizeof(serialPort_t)));
	port = usartInitAllIOSignals();

	resetMspPort(&mspPorts[0],port);

	while(1)
	{
		mspSerialProcess();
	}
}
