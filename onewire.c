/******************************************************************************** 
 * 
 * Datasheets
 *
 * 1-Wire delays ref : https://www.maximintegrated.com/en/design/technical-documents/app-notes/1/126.html
 *
 * DS18S20 ref : https://datasheets.maximintegrated.com/en/ds/DS18S20.pdf
 *
 * DS18B20 ref : https://datasheets.maximintegrated.com/en/ds/DS18B20.pdf
 *
********************************************************************************/

// DS1820 1-Wire Commands
#define SKIP_ROM 			0xCC
#define CONVERT_T			0x44
#define READ_SCRATCHPAD		0xBE

// 1-Wire struct containing GPIO infos
struct ONEWIRE_Config {
	GPIO_TypeDef* port; // Port to use
	uint16_t pin; // Pin number to use
};

// microseconds (µs) delay
void delayUs (uint32_t useconds)
{
	// Resetting timer's counter
	__HAL_TIM_SET_COUNTER(&htim1, 0);

	// Works with an 1 MHz timer
	while(useconds < __HAL_TIM_GET_COUNTER(&htim1));
}

uint8_t ONEWIRE_Reset(struct ONEWIRE_Config *config)
{
	HAL_GPIO_WritePin(config->port, config->pin, GPIO_PIN_RESET);
	delayUs(480);

	HAL_GPIO_WritePin(config->port, config->pin, GPIO_PIN_SET);
	delayUs(70);

	// if Read == '0' => a slave answered
	uint8_t slave_answered = !HAL_GPIO_ReadPin(config->port, config->pin);

	delayUs(410);

	/* Return values :
	 * 0 : no slave on the bus
	 * 1 : at least a slave on the bus
	 */
	return slave_answered;
}

void ONEWIRE_WriteBit0(struct ONEWIRE_Config *config)
{
	HAL_GPIO_WritePin(config->port, config->pin, GPIO_PIN_RESET);
	delayUs(70);
}

void ONEWIRE_WriteBit1(struct ONEWIRE_Config *config)
{
	HAL_GPIO_WritePin(config->port, config->pin, GPIO_PIN_RESET);
	delayUs(6);

	HAL_GPIO_WritePin(config->port, config->pin, GPIO_PIN_SET);
	delayUs(64);
}

uint8_t ONEWIRE_ReadBit(struct ONEWIRE_Config *config)
{
	HAL_GPIO_WritePin(config->port, config->pin, GPIO_PIN_RESET);
	delayUs(6);

	HAL_GPIO_WritePin(config->port, config->pin, GPIO_PIN_SET);
	delayUs(9);

	uint8_t read_bit = HAL_GPIO_ReadPin(config->port, config->pin);
	delayUs(55);

	return read_bit;
}

void ONEWIRE_WriteByte(struct ONEWIRE_Config *config, uint8_t byte_to_write)
{
	for(uint8_t i = 0; i < 8; i++) {
		switch((byte_to_write >> i) & 1) {
			case 0:
				ONEWIRE_WriteBit0(config);
				break;
			case 1:
				ONEWIRE_WriteBit1(config);
				break;
		}
	}
}

uint8_t ONEWIRE_ReadByte(struct ONEWIRE_Config *config)
{
	uint8_t read_byte = 0x00;
	for(uint8_t i = 0; i < 8; i++) {
		read_byte += ONEWIRE_ReadBit(config) << i;
	}

	return read_byte;
}

uint8_t DS1820_GetTemp8Bits(struct ONEWIRE_Config *config)
{
	ONEWIRE_Reset(config);
	ONEWIRE_WriteByte(config, SKIP_ROM);
	ONEWIRE_WriteByte(config, CONVERT_T);

	delayUs(10000); //delaying 10 ms = 10 000 µs

	ONEWIRE_Reset(config);
	ONEWIRE_WriteByte(config, SKIP_ROM);
	ONEWIRE_WriteByte(config, READ_SCRATCHPAD);

	// Reading DS1820's temp (Scratchpad's byte 0)
	uint8_t ds12820_temp_8_bits = ONEWIRE_ReadByte(config);

	return ds12820_temp_8_bits;
}

int16_t DS1820_GetTemp16Bits(struct ONEWIRE_Config *config)
{
	ONEWIRE_Reset(config);
	ONEWIRE_WriteByte(config, SKIP_ROM);
	ONEWIRE_WriteByte(config, CONVERT_T);

	delayUs(10000); //delaying 10 ms = 10 000 µs

	ONEWIRE_Reset(config);
	ONEWIRE_WriteByte(config, SKIP_ROM);
	ONEWIRE_WriteByte(config, READ_SCRATCHPAD);

	// Reading DS1820's temp first part (Scratchpad's byte 0)
	int16_t ds12820_temp_full_res = ONEWIRE_ReadByte(config);

	// Reading DS1820's temp last part (Scratchpad's byte 1)
	ds12820_temp_full_res += ONEWIRE_ReadByte(config) << 8;

	return ds12820_temp_full_res; // Don't forget to convert it to the selected res
}

float convert_temperature_using_res(int16_t temp_to_convert, float res)
{
	return temp_to_convert * res;
}

int main (void)
{
	/* TIMER PARAMETERS : 
		We're using TIM1
		Internal Clock = 84 MHz
		We want 1 µs => PRESCALER = 84
		Resulting timer frequency => CLK_SPEED / PRESCALER = 1 MHz = 1 µs
	*/

	// Configure 1-Wire bus
	struct ONEWIRE_Config config = {0};
	config.port = /**/; //GPIOx
	config.pin = /**/;	//GPIO_PIN_x

	// 1-Wire GPIO Config
	//RCC Register // Port clocking
	SET_BIT(GPIOA->MODER, (1 << (2 * x))); 	// Output
	SET_BIT(GPIOA->OTYPER, (1 << x)); 		// Open Drain
	SET_BIT(GPIOA->PUPDR, (1 << (2 * x))); 	// Pull-up Enabled

	// PA5 LED Config
	SET_BIT(GPIOA->MODER, (1 << (2 * 5))); 	// Output
	CLEAR_BIT(GPIOA->ODR, (1 << 5)); 		// Setting LED OFF

	// UART Config ?

	// Enable TIM1
	__HAL_TIM_ENABLE(&htim1);

// Valid values for next define : FIRST_PHASE, SECOND_PHASE, THIRD_PHASE
#define FIRST_PHASE

	while(1) {

#ifdef FIRST_PHASE
		// 1st step : Hello World => validate first 1-Wire bus implementation
			
		// Sets PA5 LED ON for 1 second if a device is on this 1-Wire Bus
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, ONEWIRE_Reset(&config));

		// Polling device every 2 seconds
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // Resetting LED state
		HAL_Delay(1000);

#elif defined(SECOND_PHASE)
		// 2nd step : Getting 8 bit temperature from DS1820 => validate communication w/ DS1820
		// Print received temp on UART

		// Get DS1820's temperature (restricted to 8 bits)
		uint8_t temp_8_bits = DS1820_GetTemp8Bits(&config); // unit : depends on resolution !

		// temp_8_bits has a 0.5°C res, we move it to a 1°C res
		float current_temperature = convert_temperature_using_res(temp_8_bits, 0.5);

		// Printing to UART

		HAL_Delay(2000);

#elif defined(THIRD_PHASE)
		// 3rd step : Getting temp full res after button press ?
		// Mean of the temp every 10 seconds ?
		// Alarm Signaling ? (ref in datasheet)

		// Get DS1820's temperature (full resolution)
		int16_t temp_16_bits = DS1820_GetTemp16Bits(config); // unit : depends on resolution !

		// You need to convert temp_16_bits before printing it !
		float current_temperature = convert_temperature_using_res(temp_16_bits, 0.5);

		// Printing to UART

		HAL_Delay(2000);
#else
#error Please define a valid macro name for phase !
#endif
	}

	return 0;
}
