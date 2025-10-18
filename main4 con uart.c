typedef enum{
	IDLE,
	MEASURE,
	PROCESS,
}States;

typedef struct{
	uint8_t bytes [BUFFER_MAX_UART];
	uint16_t longitud;
}Transfer_Config;

typedef struct{
	float32_t mean;
	float32_t std;
	float32_t max;
	uint32_t maxIndex;
	uint32_t minIndex;
	float32_t min;
	float32_t rms;
}Stats_Values;

typedef struct{
	volatile States devState;
	uint16_t buffer_ADC[ADC_BUFFER_SIZE]; //General Array for RAW ADC values (300SAMPLES)
	uint16_t buffer_LDR1 [ADC_BUFFER_SIZE/2]; //ADC values for CH1 (150 SAMPLES)
	uint16_t buffer_LDR2 [ADC_BUFFER_SIZE/2]; //ADC values for CH2 (150 SAMPLES)
	float32_t LDR1_Voltage[ADC_BUFFER_SIZE/2]; //Buffer Voltage values LDR1 for CH1
	float32_t LDR2_Voltage[ADC_BUFFER_SIZE/2]; //Buffer Voltage values LDR2 for CH2
	bool startMeasure; //start in false. To start measurements
	bool dataReady; //start in false. Data available.
	Stats_Values LDR1_Stats;
	Stats_Values LDR2_Stats;
}Device_Config;
