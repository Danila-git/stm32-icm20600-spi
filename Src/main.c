#include "stm32g474xx.h"
#include "stdint.h"

#define ACCEL_SCALE 0b11 //00 = 2g, 01 = 4g, 10 = 8g, 11 = 16g
#define GYRO_SCALE 0b11  //00 = 250dps, 01 = 500dps, 10 = 1000dps, 11 = 2000dps

void dummy_delay(uint32_t duration);
void SPI_Set(void);
void w_data_SPI(uint8_t address,uint8_t data);
uint8_t r_data_SPI(uint8_t address);
float accel_x_data;
float accel_y_data;
float accel_z_data;
float gyro_x_data;
float gyro_y_data;
float gyro_z_data;
uint8_t storage[14];
float accel_sensitivity[4] = {16384,8192,4096,2048};
float gyro_sensitivity[4] = {131,65.5,32.8,16.4};
volatile uint8_t who_am_i;


int main(void){
    SCB->CPACR |= 0x3 << 20;

RCC->APB2ENR |=	RCC_APB2ENR_SPI1EN;//включение тактирования SPI
RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;//включение тактирования порта A

//включение пинов 5-7 в альтернативные функции, а 4 в выходной сигнал
GPIOA->MODER &= ~(GPIO_MODER_MODE4_Msk | GPIO_MODER_MODE5_Msk | GPIO_MODER_MODE6_Msk | GPIO_MODER_MODE7_Msk);
GPIOA->MODER |= (1 << GPIO_MODER_MODE4_Pos) | (2 << GPIO_MODER_MODE5_Pos) | (2 << GPIO_MODER_MODE6_Pos) | (2 << GPIO_MODER_MODE7_Pos);

//альтернативные функции SPI для SCLK, MOSI, MISO
GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL5_Msk | GPIO_AFRL_AFSEL6_Msk | GPIO_AFRL_AFSEL7_Msk);
GPIOA->AFR[0] |=  (5 << GPIO_AFRL_AFSEL5_Pos) | (5 << GPIO_AFRL_AFSEL6_Pos) | (5 << GPIO_AFRL_AFSEL7_Pos);

//настройка пина CS
GPIOA->BSRR = GPIO_BSRR_BS4;//высокий уровень - неактивный
GPIOA->OTYPER &= ~GPIO_OTYPER_OT4;

SPI_Set();//функция настройки SPI
dummy_delay(50000);
SPI1->CR1 |= SPI_CR1_SPE;//включаю SPI
dummy_delay(50000);
w_data_SPI(0x70,0b01000000);//отключаю I2C
dummy_delay(50000);
w_data_SPI(0x6B,0b01);//отключаю sleep mode
dummy_delay(50000);
who_am_i = r_data_SPI(0x75);//проверяю наличие связи с ICM
w_data_SPI(0x6C,0b0);//включаю все оси акселерометра и гироскопа
w_data_SPI(0x1B,GYRO_SCALE << 3); //устанавливаю заданную чувствительность гироскопа
w_data_SPI(0x1C,ACCEL_SCALE << 3); //устанавливаю заданную чувствительность акселерометра

while(1)
{
	        uninterrupt_r_data_SPI(0x3B);//функция для непрерывного считывания данных в 14 регистрах
         //вычисляю из исходных значений по формуле значения ускорения по осям и угловые скорости
		    accel_x_data = (float)(int16_t)((storage[0] << 8) | storage[1])/accel_sensitivity[ACCEL_SCALE];
		    accel_y_data = (float)(int16_t)((storage[2] << 8) | storage[3])/accel_sensitivity[ACCEL_SCALE];
		    accel_z_data = (float)(int16_t)((storage[4] << 8) | storage[5])/accel_sensitivity[ACCEL_SCALE];
			gyro_x_data = (float)(int16_t)((storage[8] << 8) | storage[9])/gyro_sensitivity[GYRO_SCALE];
			gyro_y_data = (float)(int16_t)((storage[10] << 8) | storage[11])/gyro_sensitivity[GYRO_SCALE];
			gyro_z_data = (float)(int16_t)((storage[12] << 8) | storage[13])/gyro_sensitivity[GYRO_SCALE];
}
}

void SPI_Set(void)
{
	//настройка SPI
	SPI1->CR1 |= 0b0 << SPI_CR1_BIDIMODE_Pos;//2 линии передачи
	SPI1->CR1 |= 0b0 << SPI_CR1_RXONLY_Pos;
	SPI1->CR1 |= 0b1 << SPI_CR1_SSM_Pos;//программное управление CS
	SPI1->CR1 |= 0b1 << SPI_CR1_SSI_Pos;
	SPI1->CR1 |= 0b0 << SPI_CR1_LSBFIRST_Pos; //сначала передается старший бит
	SPI1->CR1 |= 0b001 << SPI_CR1_BR_Pos;//частота 4МГц(максимальная у ICM 10 MГц)
	SPI1->CR1 |= 0b1 << SPI_CR1_MSTR_Pos;//включаю конфигурации мастера
	SPI1->CR1 |= 0b1 << SPI_CR1_CPOL_Pos;//полярность 1
	SPI1->CR1 |= 0b1 << SPI_CR1_CPHA_Pos;//фаза 1
	SPI1->CR2 |= 0b0111 << SPI_CR2_DS_Pos;//длина слова 8 бит
    SPI1->CR2 |= 0b1 << SPI_CR2_FRXTH_Pos;//флаг заполнения RXFIFO при 8 битах
}


void w_data_SPI(uint8_t address, uint8_t data) {
    volatile uint8_t trash;

	GPIOA->BSRR = GPIO_BSRR_BR4;
	while (!(SPI1->SR & SPI_SR_TXE));

	*((volatile uint8_t*) &SPI1->DR) = address;

	while (!(SPI1->SR & SPI_SR_RXNE));

	trash = SPI1->DR;

	while (!(SPI1->SR & SPI_SR_TXE));

	*((volatile uint8_t*) &SPI1->DR) = data;

	while (!(SPI1->SR & SPI_SR_RXNE));

	trash = SPI1->DR;

    while (SPI1->SR & SPI_SR_BSY);

    GPIOA->BSRR = GPIO_BSRR_BS4;
}

uint8_t  r_data_SPI(uint8_t address)
{
	volatile uint8_t trash;
	uint8_t r_data = 0;
	address = address | 0x80;

	    GPIOA->BSRR = GPIO_BSRR_BR4;
	    while (!(SPI1->SR & SPI_SR_TXE));

	    *((volatile uint8_t *) &SPI1->DR) = address;

	    while (!(SPI1->SR & SPI_SR_RXNE));

	    trash = SPI1->DR;

	    while (!(SPI1->SR & SPI_SR_TXE));

	    *((volatile uint8_t *) &SPI1->DR) = 0x55;

	    while (!(SPI1->SR & SPI_SR_RXNE));

	    r_data = SPI1->DR;

	    while (SPI1->SR & SPI_SR_BSY);
	    GPIOA->BSRR = GPIO_BSRR_BS4;
	    return r_data;
}

void  uninterrupt_r_data_SPI(uint8_t address)
{
	volatile uint8_t trash;
	address = address | 0x80;

	    GPIOA->BSRR = GPIO_BSRR_BR4;
	    while (!(SPI1->SR & SPI_SR_TXE));

	    *((volatile uint8_t *) &SPI1->DR) = address;

	    while (!(SPI1->SR & SPI_SR_RXNE));

	    trash = SPI1->DR;

	    for(int i = 0; i <= 13; i += 1)
	    {
	    while (!(SPI1->SR & SPI_SR_TXE));

	    *((volatile uint8_t *) &SPI1->DR) = 0x55;

	    while (!(SPI1->SR & SPI_SR_RXNE));

	    storage[i] = SPI1->DR;
	    }
	    while (SPI1->SR & SPI_SR_BSY);
	    GPIOA->BSRR = GPIO_BSRR_BS4;
}

void dummy_delay(uint32_t duration) {
	for (uint32_t i = 0; i < duration; i+=1);
}
