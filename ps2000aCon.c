#pragma pack()

#include <stdint.h>
#include <stdio.h>
#include "windows.h"
#include <conio.h>
#include "ps2000aApi.h"
#include <time.h>

#define PREF4 __stdcall

#define		BUFFER_SIZE 	1024
#define		DUAL_SCOPE		2
#define		QUAD_SCOPE		4

#define		AWG_DAC_FREQUENCY      20e6
#define		AWG_DAC_FREQUENCY_MSO  2e6
#define		AWG_PHASE_ACCUMULATOR  4294967296.0

int32_t cycles = 0;

typedef enum
{
	ANALOGUE,
	DIGITAL,
	AGGREGATED,
	MIXED
}MODE;

typedef struct
{
	int16_t DCcoupled;
	int16_t range;
	int16_t enabled;
	float analogOffset;
}CHANNEL_SETTINGS;

typedef struct tTriggerDirections
{
	PS2000A_THRESHOLD_DIRECTION channelA;
	PS2000A_THRESHOLD_DIRECTION channelB;
	PS2000A_THRESHOLD_DIRECTION channelC;
	PS2000A_THRESHOLD_DIRECTION channelD;
	PS2000A_THRESHOLD_DIRECTION ext;
	PS2000A_THRESHOLD_DIRECTION aux;
}TRIGGER_DIRECTIONS;

typedef struct tPwq
{
	PS2000A_PWQ_CONDITIONS* conditions;
	int16_t nConditions;
	PS2000A_THRESHOLD_DIRECTION direction;
	uint32_t lower;
	uint32_t upper;
	PS2000A_PULSE_WIDTH_TYPE type;
}PWQ;

typedef struct
{
	int16_t					handle;
	PS2000A_RANGE			firstRange;
	PS2000A_RANGE			lastRange;
	uint8_t					signalGenerator;
	uint8_t					ETS;
	int16_t                 channelCount;
	int16_t					maxValue;
	CHANNEL_SETTINGS		channelSettings[PS2000A_MAX_CHANNELS];
	int16_t					digitalPorts;
	int16_t					awgBufferSize;
	double					awgDACFrequency;
}UNIT;

// Use this struct to help with streaming data collection
typedef struct tBufferInfo
{
	UNIT* unit;
	MODE mode;
	int16_t** driverBuffers;
	int16_t** appBuffers;
	int16_t** driverDigBuffers;
	int16_t** appDigBuffers;

} BUFFER_INFO;

// Global Variables
uint32_t	timebase = 0;
int16_t     oversample = 1;
BOOL		scaleVoltages = TRUE;

uint16_t inputRanges[PS2000A_MAX_RANGES] = {	10,
												20,
												50,
												100,
												200,
												500,
												1000,
												2000,
												5000,
												10000,
												20000,
												50000 };

BOOL     		g_ready = FALSE;
int32_t 		g_times[PS2000A_MAX_CHANNELS];
int16_t     	g_timeUnit;
int32_t      	g_sampleCount;
uint32_t		g_startIndex;
int16_t			g_autoStopped;
int16_t			g_trig = 0;
uint32_t		g_trigAt = 0;
int16_t			g_overflow = 0;

char RapidBlockFile[30] = "rapidBlock.txt";
char RapidTimeTags[30] = "rapidTimeTags.bin";
char RapidBlockFileBin[30] = "./data/rapidBlock";
char RapidTimeTagsBin[30] = "rapidTimeTagsBin.bin";
char StreamFile[30] = "stream.txt";

/****************************************************************************
* mv_to_adc
*
* Convert a millivolt value into a 16-bit ADC count
*
*  (useful for setting trigger thresholds)
****************************************************************************/
int16_t mv_to_adc(int16_t mv, int16_t ch, UNIT* unit)
{
	return (mv * unit->maxValue) / inputRanges[ch];
}

/****************************************************************************
* adc_to_mv
*
* Convert an 16-bit ADC count into millivolts
****************************************************************************/
int32_t adc_to_mv(int32_t raw, int32_t ch, UNIT* unit)
{
	return (raw * inputRanges[ch]) / unit->maxValue;
}

/****************************************************************************
* timeUnitsToString
*
* Converts PS2000A_TIME_UNITS enumeration to string (used for streaming mode)
*
****************************************************************************/
int8_t* timeUnitsToString(PS2000A_TIME_UNITS timeUnits)
{
	int8_t* timeUnitsStr = (int8_t*)"ns";

	switch (timeUnits)
	{
	case PS2000A_FS:

		timeUnitsStr = (int8_t*)"fs";
		break;

	case PS2000A_PS:

		timeUnitsStr = (int8_t*)"ps";
		break;

	case PS2000A_NS:

		timeUnitsStr = (int8_t*)"ns";
		break;

	case PS2000A_US:

		timeUnitsStr = (int8_t*)"us";
		break;

	case PS2000A_MS:

		timeUnitsStr = (int8_t*)"ms";
		break;

	case PS2000A_S:

		timeUnitsStr = (int8_t*)"s";
		break;

	default:

		timeUnitsStr = (int8_t*)"ns";
	}

	return timeUnitsStr;

}

/****************************************************************************
* Initialise unit' structure with Variant specific defaults
****************************************************************************/
void get_info(UNIT* unit)
{
	int8_t description[11][25] = { "Driver Version",
									"USB Version",
									"Hardware Version",
									"Variant Info",
									"Serial",
									"Cal Date",
									"Kernel",
									"Digital H/W",
									"Analogue H/W",
									"Firmware 1",
									"Firmware 2" };

	int16_t i, r = 0;
	int8_t line[80];
	PICO_STATUS status = PICO_OK;
	int16_t numChannels = DUAL_SCOPE;
	int8_t channelNum = 0;
	int8_t character = 'A';

	unit->signalGenerator = TRUE;
	unit->ETS = FALSE;
	unit->firstRange = PS2000A_20MV; // This is for new PicoScope 220X B, B MSO, 2405A and 2205A MSO models, older devices will have a first range of 50 mV
	unit->lastRange = PS2000A_20V;
	unit->channelCount = DUAL_SCOPE;
	unit->digitalPorts = 0;
	unit->awgBufferSize = PS2000A_MAX_SIG_GEN_BUFFER_SIZE;

	if (unit->handle)
	{
		for (i = 0; i < 11; i++)
		{
			status = ps2000aGetUnitInfo(unit->handle, (int8_t*)line, sizeof(line), &r, i);

			if (i == PICO_VARIANT_INFO)
			{
				// Check if device has four channels

				channelNum = line[1];
				numChannels = atoi(&channelNum);

				if (numChannels == QUAD_SCOPE)
				{
					unit->channelCount = QUAD_SCOPE;
				}

				// Set first range for voltage if device is a 2206/7/8, 2206/7/8A or 2205 MSO
				if (numChannels == DUAL_SCOPE)
				{
					if (strlen(line) == 4 || (strlen(line) == 5 && strcmpi(&line[4], "A") == 0) || (strcmpi(line, "2205MSO")) == 0)
					{
						unit->firstRange = PS2000A_50MV;
					}
				}

				// Check if device is an MSO 
				if (strstr(line, "MSO"))
				{
					unit->digitalPorts = 2;
				}

			}
			printf("%s: %s\n", description[i], line);
		}
	}
}

/****************************************************************************
* SetDefaults - restore default settings
****************************************************************************/
void SetDefaults(UNIT* unit)
{
	PICO_STATUS status;
	int32_t i;

	status = ps2000aSetEts(unit->handle, PS2000A_ETS_OFF, 0, 0, NULL); // Turn off ETS

	for (i = 0; i < unit->channelCount; i++) // reset channels to most recent settings
	{
		status = ps2000aSetChannel(unit->handle, (PS2000A_CHANNEL)(PS2000A_CHANNEL_A + i),
			unit->channelSettings[PS2000A_CHANNEL_A + i].enabled,
			(PS2000A_COUPLING)unit->channelSettings[PS2000A_CHANNEL_A + i].DCcoupled,
			(PS2000A_RANGE)unit->channelSettings[PS2000A_CHANNEL_A + i].range, 
			(float)unit->channelSettings[PS2000A_CHANNEL_A + i].analogOffset);
	}
}

/****************************************************************************
* SetTrigger
*
* Parameters
* - *unit               - pointer to the UNIT structure
* - *channelProperties  - pointer to the PS2000A_TRIGGER_CHANNEL_PROPERTIES structure
* - nChannelProperties  - the number of PS2000A_TRIGGER_CHANNEL_PROPERTIES elements in channelProperties
* - *triggerConditions  - pointer to the PS2000A_TRIGGER_CONDITIONS structure
* - nTriggerConditions  - the number of PS2000A_TRIGGER_CONDITIONS elements in triggerConditions
* - *directions         - pointer to the TRIGGER_DIRECTIONS structure
* - *pwq                - pointer to the pwq (Pulse Width Qualifier) structure
* - delay               - Delay time between trigger & first sample
* - auxOutputEnable     - Not used
* - autoTriggerMs       - timeout period if no trigger occurrs
* - *digitalDirections  - pointer to the PS2000A_DIGITAL_CHANNEL_DIRECTIONS structure
* - nDigitalDirections  - the number of PS2000A_DIGITAL_CHANNEL_DIRECTIONS elements in digitalDirections
*
* Returns			    - PICO_STATUS - to show success or if an error occurred
* 
***************************************************************************/
PICO_STATUS SetTrigger(UNIT* unit,
	PS2000A_TRIGGER_CHANNEL_PROPERTIES* channelProperties,
	int16_t nChannelProperties,
	PS2000A_TRIGGER_CONDITIONS* triggerConditions,
	int16_t nTriggerConditions,
	TRIGGER_DIRECTIONS* directions,
	PWQ* pwq,
	uint32_t delay,
	int16_t auxOutputEnabled,
	int32_t autoTriggerMs,
	PS2000A_DIGITAL_CHANNEL_DIRECTIONS* digitalDirections,
	int16_t nDigitalDirections)

	//unit, NULL, 0, NULL, 0, & directions, & pulseWidth, 0, 0, 0, 0, 0
{
	PICO_STATUS status;

	if ((status = ps2000aSetTriggerChannelProperties(unit->handle,
		channelProperties,
		nChannelProperties,
		auxOutputEnabled,
		autoTriggerMs)) != PICO_OK)
	{
		printf("SetTrigger:ps2000aSetTriggerChannelProperties ------ Ox%8lx \n", status);
		return status;
	}

	if ((status = ps2000aSetTriggerChannelConditions(unit->handle, triggerConditions, nTriggerConditions)) != PICO_OK)
	{
		printf("SetTrigger:ps2000aSetTriggerChannelConditions ------ 0x%8lx \n", status);
		return status;
	}

	if ((status = ps2000aSetTriggerChannelDirections(unit->handle,
		directions->channelA,
		directions->channelB,
		directions->channelC,
		directions->channelD,
		directions->ext,
		directions->aux)) != PICO_OK)
	{
		printf("SetTrigger:ps2000aSetTriggerChannelDirections ------ 0x%08lx \n", status);
		return status;
	}

	if ((status = ps2000aSetTriggerDelay(unit->handle, delay)) != PICO_OK)
	{
		printf("SetTrigger:ps2000aSetTriggerDelay ------ 0x%08lx \n", status);
		return status;
	}
	/*
	if ((status = ps2000aSetPulseWidthQualifier(unit->handle,
		pwq->conditions,
		pwq->nConditions,
		pwq->direction,
		pwq->lower,
		pwq->upper,
		pwq->type)) != PICO_OK)
	{
		printf("SetTrigger:ps2000aSetPulseWidthQualifier ------ 0x%08lx \n", status);
		return status;
	}
	*/
	if (unit->digitalPorts)					// ps2000aSetTriggerDigitalPortProperties function only applies to MSO	
	{
		if ((status = ps2000aSetTriggerDigitalPortProperties(unit->handle,
			digitalDirections,
			nDigitalDirections)) != PICO_OK)
		{
			printf("SetTrigger:ps2000aSetTriggerDigitalPortProperties ------ 0x%08lx \n", status);
			return status;
		}
	}
	return status;
}

/****************************************************************************
* OpenDevice
* Parameters
* - unit        pointer to the UNIT structure, where the handle will be stored
*
* Returns
* - PICO_STATUS to indicate success, or if an error occurred
***************************************************************************/
PICO_STATUS OpenDevice(UNIT* unit, int16_t lowTrigger, int16_t highTrigger)
{
	int16_t value = 0;
	int32_t i;

	PICO_STATUS status = ps2000aOpenUnit(&(unit->handle), NULL);

	printf("Handle: %d\n", unit->handle);

	if (status != PICO_OK)
	{
		printf("Unable to open device\n");
		printf("Error code : %d\n", (int32_t)status);
		while (!_kbhit());
		exit(99); // exit program
	}

	printf("Device opened successfully, cycle %d\n\n", ++cycles);

	// setup devices
	get_info(unit);
	timebase = 0;

	ps2000aMaximumValue(unit->handle, &value);
	unit->maxValue = value;

	for (i = 0; i < unit->channelCount; i++)
	{
		// only activates CH1 and CH2
		if (i == 0 || i == 1) {
			unit->channelSettings[i].enabled = TRUE;
			unit->channelSettings[i].DCcoupled = TRUE;
			unit->channelSettings[i].range = PS2000A_200MV;
			unit->channelSettings[i].analogOffset = -0.12;				// -120 mV analog offset
		}
		else
			unit->channelSettings[i].enabled = FALSE;
	}

	SetDefaults(unit);

	/* Trigger setup */

	uint16_t triggerVoltage_lowA = mv_to_adc(lowTrigger, unit->channelSettings[PS2000A_CHANNEL_A].range, unit);
	uint16_t triggerVoltage_lowB = mv_to_adc(lowTrigger, unit->channelSettings[PS2000A_CHANNEL_B].range, unit);
	uint16_t triggerVoltage_highA = mv_to_adc(highTrigger, unit->channelSettings[PS2000A_CHANNEL_A].range, unit);
	uint16_t triggerVoltage_highB = mv_to_adc(highTrigger, unit->channelSettings[PS2000A_CHANNEL_B].range, unit);

	float triggerHighHist = 0.015;

	PS2000A_TRIGGER_CHANNEL_PROPERTIES triggerLevels[2] = { { triggerVoltage_highA,		// thr upper
																(uint16_t)(triggerHighHist * triggerVoltage_highA),	// histeresis
																triggerVoltage_lowA,	// thr lower
																(uint16_t)(triggerHighHist * triggerVoltage_lowA),	// histeresis
																PS2000A_CHANNEL_A,		// channel
																PS2000A_LEVEL },		// type: PS2000A_LEVEL or PS2000A_WINDOW
															{ triggerVoltage_highB,
																(uint16_t)(triggerHighHist * triggerVoltage_highB),
																triggerVoltage_lowB,
																(uint16_t)(triggerHighHist * triggerVoltage_lowB),
																PS2000A_CHANNEL_B,
																PS2000A_LEVEL } };

	TRIGGER_DIRECTIONS directions = {							PS2000A_ABOVE,			// Channel A, or PS2000A_INSIDE?
																PS2000A_ABOVE,			// Channel B
																PS2000A_NONE,			// Channel C
																PS2000A_NONE,			// Channel D
																PS2000A_NONE,			// ext
																PS2000A_NONE };			// aux

	PS2000A_TRIGGER_CONDITIONS conditions = {					PS2000A_CONDITION_TRUE,				// Channel A
																PS2000A_CONDITION_TRUE,				// Channel B 
																PS2000A_CONDITION_DONT_CARE,		// Channel C
																PS2000A_CONDITION_DONT_CARE,		// Channel D
																PS2000A_CONDITION_DONT_CARE,		// external
																PS2000A_CONDITION_DONT_CARE,		// aux
																PS2000A_CONDITION_DONT_CARE,		// PWQ
																PS2000A_CONDITION_DONT_CARE };		// digital

	SetTrigger(unit, &triggerLevels, 2, &conditions, 1, &directions, NULL, 0, 0, 0, 0, 0);
	//ps2000aHoldOff(unit->handle, 10E-9, 0);
	printf("trigger set to %d ADC units", triggerVoltage_highA);

	return status;
}

/****************************************************************************
* Callback
* used by ps2000a data streaming collection calls, on receipt of data.
* used to set global flags etc checked by user routines
****************************************************************************/
void PREF4 CallBackStreaming(int16_t handle,
	int32_t noOfSamples,
	uint32_t	startIndex,
	int16_t overflow,
	uint32_t triggerAt,
	int16_t triggered,
	int16_t autoStop,
	void* pParameter)
{
	int32_t channel;
	int32_t digiPort;
	BUFFER_INFO* bufferInfo = NULL;

	if (pParameter != NULL)
	{
		bufferInfo = (BUFFER_INFO*)pParameter;
	}

	// used for streaming
	g_sampleCount = noOfSamples;
	g_startIndex = startIndex;
	g_autoStopped = autoStop;
	g_overflow = overflow;

	// flag to say done reading data
	g_ready = TRUE;

	// flags to show if & where a trigger has occurred
	g_trig = triggered;
	g_trigAt = triggerAt;

	if (bufferInfo != NULL && noOfSamples)
	{
		if (bufferInfo->mode == ANALOGUE)
		{
			for (channel = 0; channel < bufferInfo->unit->channelCount; channel++)
			{
				if (bufferInfo->unit->channelSettings[channel].enabled)
				{
					if (bufferInfo->appBuffers && bufferInfo->driverBuffers)
					{
						if (bufferInfo->appBuffers[channel * 2] && bufferInfo->driverBuffers[channel * 2])
						{
							memcpy_s(&bufferInfo->appBuffers[channel * 2][startIndex], noOfSamples * sizeof(int16_t),
								&bufferInfo->driverBuffers[channel * 2][startIndex], noOfSamples * sizeof(int16_t));
						}
						if (bufferInfo->appBuffers[channel * 2 + 1] && bufferInfo->driverBuffers[channel * 2 + 1])
						{
							memcpy_s(&bufferInfo->appBuffers[channel * 2 + 1][startIndex], noOfSamples * sizeof(int16_t),
								&bufferInfo->driverBuffers[channel * 2 + 1][startIndex], noOfSamples * sizeof(int16_t));
						}
					}
				}
			}
		}
	}
}

/****************************************************************************
* Stream Data Handler
* - Used by the two stream data examples - untriggered and triggered
* Inputs:
* - unit - the unit to sample on
***************************************************************************/
void StreamDataHandler(UNIT* unit, MODE mode, uint32_t numSamples, uint32_t numSegments)
{
	int8_t* timeUnitsStr;

	int16_t  autostop;
	uint16_t portValue, portValueOR, portValueAND;
	uint32_t segmentIndex = 0;

	int16_t* buffers[PS2000A_MAX_CHANNEL_BUFFERS];
	int16_t* appBuffers[PS2000A_MAX_CHANNEL_BUFFERS];
	int16_t* digiBuffers[PS2000A_MAX_DIGITAL_PORTS];
	int16_t* appDigiBuffers[PS2000A_MAX_DIGITAL_PORTS];

	int32_t index = 0;
	int32_t totalSamples;
	int32_t bit;
	int32_t i, j;

	int32_t sampleCount = numSamples * numSegments; /*make sure buffer large enough */
	uint32_t postTrigger; uint32_t preTrigger;
	uint32_t downsampleRatio = 1;
	uint32_t sampleInterval;
	uint32_t triggeredAt = 0;

	BUFFER_INFO bufferInfo;
	FILE* fp = NULL;

	PICO_STATUS status;
	PS2000A_TIME_UNITS timeUnits;
	PS2000A_RATIO_MODE ratioMode;

	if (mode == ANALOGUE)		// Analogue 
	{
		for (i = 0; i < unit->channelCount; i++)
		{
			if (unit->channelSettings[i].enabled)
			{
				buffers[i * 2] = (int16_t*)malloc(sampleCount * sizeof(int16_t));
				buffers[i * 2 + 1] = (int16_t*)malloc(sampleCount * sizeof(int16_t));
				status = ps2000aSetDataBuffers(unit->handle, (int32_t)i, buffers[i * 2], buffers[i * 2 + 1], sampleCount, segmentIndex, PS2000A_RATIO_MODE_NONE);

				appBuffers[i * 2] = (int16_t*)malloc(sampleCount * sizeof(int16_t));
				appBuffers[i * 2 + 1] = (int16_t*)malloc(sampleCount * sizeof(int16_t));

				printf(status ? "StreamDataHandler:ps2000aSetDataBuffers(channel %ld) ------ 0x%08lx \n" : "", i, status);
			}
		}

		timeUnits = PS2000A_NS;
		sampleInterval = 16;
		postTrigger = 24;
		preTrigger = 24;
		autostop = TRUE;
	}

	bufferInfo.unit = unit;
	bufferInfo.mode = mode;
	bufferInfo.driverBuffers = buffers;
	bufferInfo.appBuffers = appBuffers;
	bufferInfo.driverDigBuffers = digiBuffers;
	bufferInfo.appDigBuffers = appDigiBuffers;

	g_autoStopped = FALSE;

	status = ps2000aRunStreaming(unit->handle, &sampleInterval, timeUnits, preTrigger, postTrigger,
		autostop, downsampleRatio, PS2000A_RATIO_MODE_NONE, (uint32_t)sampleCount);

	if (status == PICO_OK)
	{
		timeUnitsStr = timeUnitsToString(timeUnits);
		printf("Streaming data... (interval: %d %s) Press a key to stop\n", sampleInterval, timeUnitsStr);
	}
	else
	{
		printf("StreamDataHandler:ps2000aRunStreaming ------ 0x%08lx \n", status);
	}

	fopen_s(&fp, StreamFile, "w");

	totalSamples = 0;

	// Capture data unless a key is pressed or the g_autoStopped flag is set in the streaming callback
	while (!_kbhit() && !g_autoStopped)
	{
		/* Poll until data is received. Until then, GetStreamingLatestValues wont call the callback */
		g_ready = FALSE;

		status = ps2000aGetStreamingLatestValues(unit->handle, CallBackStreaming, &bufferInfo);
		index++;

		if (g_ready && g_sampleCount > 0) /* can be ready and have no data, if autoStop has fired */
		{
			if (g_trig)
			{
				triggeredAt = totalSamples + g_trigAt;		// calculate where the trigger occurred in the total samples collected
			}
			totalSamples += g_sampleCount;

			for (i = g_startIndex; i < (int32_t)(g_startIndex + g_sampleCount); i++)
			{
				if (mode == ANALOGUE)
				{
					if (fp != NULL)
					{
						for (j = 0; j < unit->channelCount; j++)
						{
							if (unit->channelSettings[j].enabled){
								fprintf(fp, "%d\t", appBuffers[j * 2][i]);
							}
							fprintf(fp, "%c", '\t');
						}
						fprintf(fp, "%c", '\n');
					}
					else
					{
						printf("Cannot open the file stream.txt for writing.\n");
					}

				}

			}
		}
	}

	ps2000aStop(unit->handle);

	if (!g_autoStopped)
	{
		printf("\nData collection aborted.\n");
		_getch();
	}

	if (g_overflow)
	{
		printf("Overflow on voltage range.\n");
	}

	if (fp != NULL)
	{
		fclose(fp);
	}

	if (mode == ANALOGUE)		// Only if we allocated these buffers
	{
		for (i = 0; i < unit->channelCount; i++)
		{
			if (unit->channelSettings[i].enabled)
			{
				free(buffers[i * 2]);
				free(buffers[i * 2 + 1]);

				free(appBuffers[i * 2]);
				free(appBuffers[i * 2 + 1]);
			}
		}
	}

	ClearDataBuffers(unit);
}

/****************************************************************************
* CollectStreamingTriggered
*  this function demonstrates how to collect a stream of data
*  from the unit (start collecting on trigger)
***************************************************************************/
void CollectStreamingTriggered(UNIT* unit, uint32_t numSamples, uint32_t numSegments)
{

	printf("Collect streaming triggered...\n");
	printf("Data is written to disk file (%s)\n", StreamFile);
	
	printf("Press a key to start...\n");
	_getch();

	//SetDefaults(unit);

	StreamDataHandler(unit, ANALOGUE, numSamples, numSegments);
}

/****************************************************************************
* Callback
* used by ps2000a data block collection calls, on receipt of data.
* used to set global flags etc checked by user routines
****************************************************************************/
void PREF4 CallBackBlock(int16_t handle, PICO_STATUS status, void* pParameter)
{
	if (status != PICO_CANCELLED)
	{
		g_ready = TRUE;
	}
}

/****************************************************************************
* ClearDataBuffers
*
* stops GetData writing values to memory that has been released
****************************************************************************/
PICO_STATUS ClearDataBuffers(UNIT* unit)
{
	int32_t i;
	PICO_STATUS status;

	for (i = 0; i < unit->channelCount; i++)
	{
		if ((status = ps2000aSetDataBuffers(unit->handle, (int16_t)i, NULL, NULL, 0, 0, PS2000A_RATIO_MODE_NONE)) != PICO_OK)
		{
			printf("ClearDataBuffers:ps2000aSetDataBuffers(channel %d) ------ 0x%08lx \n", i, status);
		}
	}


	for (i = 0; i < unit->digitalPorts; i++)
	{
		if ((status = ps2000aSetDataBuffer(unit->handle, (PS2000A_CHANNEL)(i + PS2000A_DIGITAL_PORT0), NULL, 0, 0, PS2000A_RATIO_MODE_NONE)) != PICO_OK)
		{
			printf("ClearDataBuffers:ps2000aSetDataBuffer(port 0x%X) ------ 0x%08lx \n", i + PS2000A_DIGITAL_PORT0, status);
		}
	}

	return status;
}

/****************************************************************************
* CollectRapidBlock
*  this function demonstrates how to collect a set of captures using
*  rapid block mode.
****************************************************************************/
void CollectRapidBlock(UNIT* unit, const uint32_t numSegments, uint32_t numSamplesPerSegment, uint32_t selTimebase, int number)
{
	int16_t i;
	int16_t channel;
	int16_t*** rapidBuffers;
	int16_t* overflow;

	uint32_t nCaptures;
	uint32_t capture;
	uint32_t maxSegments = 0;

	int32_t nMaxSamples;
	int32_t timeIndisposed;

	uint32_t nSamples = numSamplesPerSegment;
	uint32_t nCompletedCaptures;

	PICO_STATUS status;

	printf("Collect rapid block triggered...\n");

	//SetDefaults(unit);

	// Set the number of captures
	nCaptures = numSegments;

	// Find the maximum number of segments for the device
	status = ps2000aGetMaxSegments(unit->handle, &maxSegments);
	printf("\nmaximum segments allowed per block: %d\n", maxSegments);

	if (nCaptures > maxSegments)
	{
		nCaptures = maxSegments;
	}
	printf("\nActual segments to acquire: %d\n", nCaptures);

	// Segment the memory
	status = ps2000aMemorySegments(unit->handle, nCaptures, &nMaxSamples);

	// Set the number of captures
	status = ps2000aSetNoOfCaptures(unit->handle, nCaptures);

	// Run
	timebase = selTimebase;		// Refer to the Programmer's Guide Timebases section
	status = ps2000aRunBlock(unit->handle, (int)(nSamples / 2), (int)(nSamples / 2), timebase, 1, &timeIndisposed, 0, CallBackBlock, NULL);

	// Wait until data ready
	g_ready = 0;

	while (!g_ready && !_kbhit())
	{
		Sleep(0);
	}

	if (!g_ready)
	{
		_getch();
		status = ps2000aStop(unit->handle);
		status = ps2000aGetNoOfCaptures(unit->handle, &nCompletedCaptures);

		printf("Rapid capture aborted. %lu complete blocks were captured\n", nCompletedCaptures);
		printf("\nPress any key...\n\n");
		_getch();

		if (nCompletedCaptures == 0)
		{
			return;
		}

		// Only display the blocks that were captured
		nCaptures = (uint16_t)nCompletedCaptures;
	}

	// Allocate memory
	rapidBuffers = (int16_t***)calloc(unit->channelCount, sizeof(int16_t*));
	overflow = (int16_t*)calloc(unit->channelCount * nCaptures, sizeof(int16_t));

	for (channel = 0; channel < unit->channelCount; channel++)
	{
		rapidBuffers[channel] = (int16_t**)calloc(nCaptures, sizeof(int16_t*));
	}

	for (channel = 0; channel < unit->channelCount; channel++)
	{
		if (unit->channelSettings[channel].enabled)
		{
			for (capture = 0; capture < nCaptures; capture++)
			{
				rapidBuffers[channel][capture] = (int16_t*)calloc(nSamples, sizeof(int16_t));
			}
		}
	}

	for (channel = 0; channel < unit->channelCount; channel++)
	{
		if (unit->channelSettings[channel].enabled)
		{
			for (capture = 0; capture < nCaptures; capture++)
			{
				status = ps2000aSetDataBuffer(unit->handle, channel, rapidBuffers[channel][capture], nSamples, capture, PS2000A_RATIO_MODE_NONE);
			}
		}
	}

	// Get data
	status = ps2000aGetValuesBulk(unit->handle, &nSamples, 0, nCaptures - 1, 1, PS2000A_RATIO_MODE_NONE, overflow);

	// Stop
	status = ps2000aStop(unit->handle);

	FILE* fp = NULL;

	// ascii
	/*
	fopen_s(&fp, RapidBlockFile, "w");
	if (fp != NULL) {
		for (capture = 0; capture < nCaptures; capture++)
		{
			for (int i = 0; i < nSamples; i++)
				fprintf(fp, "%d\t%d\n", rapidBuffers[0][capture][i], rapidBuffers[1][capture][i]);
		}
		fclose(fp);
	}
	*/

	// for timestamps - DOESN'T WORK! This actually gives the time offset between the trigger and the first post trigger sample
	// of a given segment
	/*
	int64_t *times;
	PS2000A_TIME_UNITS* timeUnits;
	times = (int64_t*)malloc(sizeof(int64_t) * nCaptures);
	if (times == NULL) {
		printf("malloc of size %d failed!\n", nCaptures);   // could also call perror here
		exit(1);   // or return an error to caller
	}

	timeUnits = (PS2000A_TIME_UNITS*)malloc(sizeof(PS2000A_TIME_UNITS) * nCaptures);
	if (timeUnits == NULL) {
		printf("malloc of size %d failed!\n", nCaptures);   // could also call perror here
		exit(1);   // or return an error to caller
	}

	ps2000aGetValuesTriggerTimeOffsetBulk64(unit->handle, times, timeUnits, 0, nCaptures - 1);

	for (int i = 0; i < 10; i++) {
		printf("%d ", times[i]);
		printf("%s\n", timeUnitsToString(timeUnits[i]));
	}
	*/

	/* in case we need data processed during acquisition (not acquiring waveforms) */
	/* data to save for both channels (per segment):
	 * original data: 200B 
	 * 1. dt - float, 4B
	 * 2. baseline - uint8_t, 1B * 2 = 2B
	 * 3. baseline std - uint8_t, 1B * 2 = 2B
	 * 4. pulse area - uint16_t, 2B * 2 = 4B
	 * 5. pulse max - uint8_t, 1B * 2 = 2B
	 * 6. pulse min - int8_t, 1B * 2 = 2B
	 * 
	 * TOTAL: 16B (~92% reduction or a factor of 1/10)
	 */

	// binary
	char str[1024];
	sprintf(str, "%s_%d.bin", RapidBlockFileBin, number);

	fopen_s(&fp, str, "wb");
	double downScaling = 256.0 / 65535.0;
	int iter = 0;
	if (fp != NULL) {
		for (capture = 0; capture < nCaptures; capture++)
		{
			for (int i = 0; i < nSamples; i++) {

				int8_t first = (int8_t)((double)rapidBuffers[0][capture][i] * downScaling);
				int8_t second = (int8_t)((double)rapidBuffers[1][capture][i] * downScaling);

				fwrite(&first, sizeof(int8_t), 1, fp);		// 1 byte
				fwrite(&second, sizeof(int8_t), 1, fp);		// 1 byte
				iter++;
			}
		}
		fclose(fp);
	}

	float samplesPerSegmentReal = iter / nCaptures;
	printf("\nNumber of samples per segment: %f\n", samplesPerSegmentReal);
	
	// Free memory
	free(overflow);

	for (channel = 0; channel < unit->channelCount; channel++)
	{
		if (unit->channelSettings[channel].enabled)
		{
			for (capture = 0; capture < nCaptures; capture++)
			{
				free(rapidBuffers[channel][capture]);
			}
		}
	}

	for (channel = 0; channel < unit->channelCount; channel++)
	{
		free(rapidBuffers[channel]);
	}

	free(rapidBuffers);

	//free(times);
	//free(timeUnits);

	// Set number of segments and captures back to 1
	// status = ps2000aMemorySegments(unit->handle, 1, &nMaxSamples);
	// status = ps2000aSetNoOfCaptures(unit->handle, 1);

}

void CreateLogFile(uint32_t timebase, uint32_t numSegments, uint32_t numSamplesPerSegment, 
	uint32_t lowTrigger, uint32_t highTrigger, PS2000A_THRESHOLD_DIRECTION trigType)
{
	FILE* fp = NULL;
	fp = fopen_s(&fp, "logfile.txt", "w");
	
	if (fp != NULL) {
		fprintf(fp, "timebase: %d\nsegments per file: %d\nsamples per segment: %d\nlow trigger: %d\nhigh trigger: %d\ntrigger type (both channels): %d\n", timebase, numSegments, numSamplesPerSegment, lowTrigger, highTrigger, trigType);
		fclose(fp);
	}
	else {
		printf("Couldn't create logfile!\n");
		exit(1);
	}

	return;
}

uint32_t main(void)
{
	int8_t ch;

	PICO_STATUS status;
	UNIT unit;

	printf("PicoScope 2000 Series DAQ\n");
	printf("\n\nOpening the device...\n");

	float lowTrigger = -40;	// mV
	float highTrigger = -40;	// mV

	const uint32_t numSegments			= 150000;
	uint32_t numSamplesPerSegment		= 100;

	status = OpenDevice(&unit, lowTrigger, highTrigger);
	printf("\n");

	int samplesPerSecond = numSegments * numSamplesPerSegment / 1E6;
	printf("Requesting %d MS per rapid block\n", samplesPerSecond);

	// this mode has a minimum sampling rate of 62.5 MHz (insufficient for SiPM pulse digitalization)
	//CollectStreamingTriggered(&unit, numSamplesPerSegment, numSegments);

	uint32_t timebase				= 1;	// 1 = lower temporal timebase, sampling @ 500 Ms/s - works with 50 samples per segment
	int iter						= 0;

	//CreateLogFile(timebase, numSegments, numSamplesPerSegment, lowTrigger, highTrigger, PS2000A_ABOVE);

	//double timeMax = 275000;
	clock_t begin_s = clock();
	while(1) {

		clock_t begin = clock();
		CollectRapidBlock(&unit, numSegments, numSamplesPerSegment, timebase, iter);
		clock_t end = clock();

		double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
		double rate = numSegments / time_spent;
		//printf("\nRapid block acquired in %f seconds\n", time_spent);
		printf("Effective rate: %d/s\n", (int)rate);
		iter++;
		clock_t end_s = clock();
		double timeSpent = (double)(end_s - begin_s) / CLOCKS_PER_SEC;
		//if (timeSpent > timeMax)
			//break;
	}

}
