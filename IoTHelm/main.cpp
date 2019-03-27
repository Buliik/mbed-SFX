#include "mbed.h"
#include "Shields.h"
#include "LDC1101.h"

#define USR_BUFFER_SIZE			128
#define SFX_RET_SUCCESS			0
#define MAX_SFX_TX				140

//Initialize Serial instance
Serial pc(USBTX, USBRX);

//Initialize object for sigfox interface
AX8052F143_SFX sfx;

//Initialize the sensor
//LDC1101 *cidlo = new LDC1101(p5, p6, p4, p14, 120E-12, 16E6);

/*
DigitalOut clkin(p15);

int main() {

	char sfxBuff[USR_BUFFER_SIZE];
	char dataBuf[USR_BUFFER_SIZE];

	wait(1);

	pc.printf("Sample Sigfox_SPI program\r\n");

	if (sfx.init() == SFX_FAILURE) {
		pc.printf("Sigfox Init failed\r\n");
		return false;
	} else {
		pc.printf("Sigfox init successful\r\n");
	}

	//Send dummy AT command to avoid reading junk data the very 1st time.
	if (sfx.sendCmdWithResponse("AT\n", sfxBuff, USR_BUFFER_SIZE) == SFX_SUCCESS)
		sfx.cleanBuffer(sfxBuff, USR_BUFFER_SIZE);

	if (sfx.getChipInfo(SOFTWARE_VERSION, sfxBuff,
		USR_BUFFER_SIZE) == SFX_SUCCESS) {
		pc.printf("Software Ver. = %s\r\n", sfxBuff);
		sprintf(dataBuf, "%s\n", sfxBuff);
		sfx.cleanBuffer(dataBuf, USR_BUFFER_SIZE);
		sfx.cleanBuffer(sfxBuff, USR_BUFFER_SIZE);
		wait(2);
	}

	if (sfx.getChipInfo(PAC, sfxBuff, USR_BUFFER_SIZE) == SFX_SUCCESS) {
		pc.printf("PAC= %s\r\n", sfxBuff);
		sprintf(dataBuf, "PAC= %s\n", sfxBuff);
		sfx.cleanBuffer(dataBuf, USR_BUFFER_SIZE);
		sfx.cleanBuffer(sfxBuff, USR_BUFFER_SIZE);
		wait(2);
	}

	wait(2);
	int i = 0;
	uint32_t curData;
	while(i < 100)
	{
		if(!cidlo->is_Oscillation_Error())
		{
			curData = cidlo->get_RP_Data();
			sprintf(dataBuf, "RP %d", curData);

				if (sfx.sendFrame(dataBuf, 0, sfxBuff, USR_BUFFER_SIZE) != SFX_SUCCESS) {
							pc.printf("Sending to Sigfox failed\r\n");
						} else {
						pc.printf("Success in sending to Sigfox\r\n");
						}
				pc.printf("%d\n", curData);
		}
		else pc.printf("Oscillation error\n");

		wait(1);
		i++;
	}


}*/

int main() {

	char sfxBuff[USR_BUFFER_SIZE];
	char dataBuf[USR_BUFFER_SIZE];
	char curData[7] = {0, 96, 0, 219, 143, 0, 5};

	pc.printf("Sample Sigfox program\r\n");

	if (sfx.init() == SFX_FAILURE) {
		pc.printf("Sigfox Init failed\r\n");
		return false;
	} else {
		pc.printf("Sigfox init successful\r\n");
	}

	//Send dummy AT command to avoid reading junk data the very 1st time.
	if (sfx.sendCmdWithResponse("AT\n", sfxBuff, USR_BUFFER_SIZE) == SFX_SUCCESS)
		sfx.cleanBuffer(sfxBuff, USR_BUFFER_SIZE);

	if (sfx.getChipInfo(SOFTWARE_VERSION, sfxBuff,
	USR_BUFFER_SIZE) == SFX_SUCCESS) {
		pc.printf("Software Ver. = %s\r\n", sfxBuff);
		sprintf(dataBuf, "%s\n", sfxBuff);
		sfx.cleanBuffer(dataBuf, USR_BUFFER_SIZE);
		sfx.cleanBuffer(sfxBuff, USR_BUFFER_SIZE);
		wait(2);
	}

	if (sfx.getChipInfo(PAC, sfxBuff, USR_BUFFER_SIZE) == SFX_SUCCESS) {
		pc.printf("PAC= %s\r\n", sfxBuff);
		sprintf(dataBuf, "PAC= %s\n", sfxBuff);
		sfx.cleanBuffer(dataBuf, USR_BUFFER_SIZE);
		sfx.cleanBuffer(sfxBuff, USR_BUFFER_SIZE);
		wait(2);
	}


	wait(1);

		/*curData[0] = 0;
		curData[1] = 96;
		curData[2] = 0;
		curData[3] = 219;
		curData[4] = 143;
		curData[5] = 0;
		curData[6] = 5;*/

		sprintf(dataBuf, curData);

		if (sfx.sendFrame(dataBuf, 0, sfxBuff, USR_BUFFER_SIZE) != SFX_SUCCESS) {
						pc.printf("Sending to Sigfox failed\r\n");
					} else {
					pc.printf("Success in sending to Sigfox\r\n");
					}
			//pc.printf("%d\n", curData);

			wait(1);


	pc.printf("Maximum message limit reached. Exiting.....\r\n");
	return SFX_RET_SUCCESS;
}

