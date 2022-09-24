#define SKIP_ROM 			0xCC
#define CONVERT_T			0x44
#define READ_SCRATCHPAD		0xBE

struct ONEWIRE_Config {
	GPIO_TypeDef*	port; // Port to use
	uint16_t		pint; // Pin number to use
};

// ONEWIRE delays ref : https://www.maximintegrated.com/en/design/technical-documents/app-notes/1/126.html

// microseconds (µs) delay
void delayUs (uint32_t useconds)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while(useconds < __HAL_TIM_GET_COUNTER(&htim1));
}

uint8_t ONEWIRE_Reset(struct ONEWIRE_Config *config)
{
	HAL_GPIO_WritePin(config->port, config->pin, GPIO_PIN__RESET);
	delayUs(480);

	HAL_GPIO_WritePin(config->port, config->pin, GPIO_PIN__SET);
	delayUs(70);

	// if Read == '0' => a slave answered
	uint8_t slave_answered = !HAL_GPIO_ReadPin(config->port, config->pin);

	delayUs(410);
	return slave_answered;
}

void ONEWIRE_WriteBit0(struct ONEWIRE_Config *config)
{
	HAL_GPIO_WritePin(config->port, config->pin, GPIO_PIN__RESET);
	delayUs(70);
}

void ONEWIRE_WriteBit1(struct ONEWIRE_Config *config)
{
	HAL_GPIO_WritePin(config->port, config->pin, GPIO_PIN__RESET);
	delayUs(6);
	HAL_GPIO_WritePin(config->port, config->pin, GPIO_PIN__SET);
	delayUs(64);
}

uint8_t ONEWIRE_ReadBit(struct ONEWIRE_Config *config)
{
	HAL_GPIO_WritePin(config->port, config->pin, GPIO_PIN__RESET);
	delayUs(6);
	HAL_GPIO_WritePin(config->port, config->pin, GPIO_PIN__SET);
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

	// Reading DS1820's temp
	return ONEWIRE_ReadByte(config);
}

int16_t DS1820_GetTemp16Bits(struct ONEWIRE_Config *config)
{
	//ONEWIRE_Reset(config);
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

	// Configure GPIO
	// GPIOA-> ...

	// Config 1-Wire GPIO
	??? // Port clocking
	GPIOx->MODER |= (1 << (2 * GPIO_PIN_x)); // Output
	GPIOx->OTYPER |= (1 << GPIO_PIN_x); // Open Drain
	GPIOx->PUPDR |= (1 << (2 * GPIO_PIN_x)); // Pull-up Enabled

	// PA5 LED Config
	GPIOA->MODER |= (1 << (2*GPIO_PIN_5)); // Output
	GPIOA->ODR &= ~(1 << GPIO_PIN_5); // Setting LED OFF

	// Config liaison série ?

	// Enable TIM1
	__HAL_TIM_ENABLE(&htim1);

	enum step_e {FIRST_PHASE = 0, SECOND_PHASE, THIRD_PHASE};
	enum step_e current_step = FIRST_STAGE;

	while(1) {
		switch (current_step) {
			case FIRST_STAGE:
				// 1st step : Hello World => validate first 1-Wire bus implementation
				
				// Sets PA5 LED ON for 1 second if a device is on this 1-Wire Bus
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, ONEWIRE_Reset(&config));

				// Polling device every 2 seconds
				HAL_Delay(1000);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN__RESET);
				HAL_Delay(1000);

				break;
			case SECOND_STAGE:
				// 2nd step : Getting 8 bit temperature from DS1820 => validate communication w/ DS1820
				uint8_t temp = DS1820_GetTemp8Bits(&config);
				HAL_Delay(1000);

				break;
			case THIRD_STAGE:
				// 3rd step : Getting temp after button press
				// Mean of the temp every 10 seconds ?

				break;
			default:
				break;
		}
	}

	return 0;
}

