#include "stm32g474xx.h"
#include "stdint.h"

#define ACCEL_SCALE 0b11 //00 = 2g, 01 = 4g, 10 = 8g, 11 = 16g
#define GYRO_SCALE 0b11  //00 = 250dps, 01 = 500dps, 10 = 1000dps, 11 = 2000dps
#define MAX_ARRAY_SIZE 50

void dummy_delay(uint32_t duration);
void spi_set(void);
void rewrite_data_spi(uint8_t address, uint8_t data); // function to rewrite register
void add_data_spi(uint8_t address, uint8_t data);     // function to add data to the register
void uninterrupt_r_data_spi(uint8_t address, uint32_t num_of_addresses);
void uninterrupt_w_data_spi(uint8_t address, uint32_t num_of_addresses, uint8_t storage[MAX_ARRAY_SIZE]);
uint8_t send_recieve_byte_of_data(uint8_t send_data);
uint8_t r_data_spi(uint8_t address);
float accel_x_data, accel_y_data, accel_z_data, gyro_x_data, gyro_y_data, gyro_z_data;
uint8_t storage[MAX_ARRAY_SIZE];
float accel_sensitivity[4] = {16384, 8192, 4096, 2048};
float gyro_sensitivity[4] = {131, 65.5, 32.8, 16.4};
uint8_t who_am_i;

int main(void) {
    SCB->CPACR |= 0x3 << 20;

    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // enable SPI clock
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; // enable port A clock

    // configure pins 5-7 as alternate function, pin 4 as output
    GPIOA->MODER &= ~(GPIO_MODER_MODE4_Msk | GPIO_MODER_MODE5_Msk | GPIO_MODER_MODE6_Msk | GPIO_MODER_MODE7_Msk);
    GPIOA->MODER |= (1 << GPIO_MODER_MODE4_Pos) | (2 << GPIO_MODER_MODE5_Pos) | (2 << GPIO_MODER_MODE6_Pos) | (2 << GPIO_MODER_MODE7_Pos);

    // set alternate function SPI for SCLK, MOSI, MISO
    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL5_Msk | GPIO_AFRL_AFSEL6_Msk | GPIO_AFRL_AFSEL7_Msk);
    GPIOA->AFR[0] |= (5 << GPIO_AFRL_AFSEL5_Pos) | (5 << GPIO_AFRL_AFSEL6_Pos) | (5 << GPIO_AFRL_AFSEL7_Pos);

    // configure CS pin
    GPIOA->BSRR = GPIO_BSRR_BS4; // Set high - inactive

    spi_set(); // SPI configuration function
    dummy_delay(50000);
    SPI1->CR1 |= SPI_CR1_SPE; // enable SPI
    dummy_delay(50000);
    add_data_spi(0x70, 0b01000000); // disable I2C
    dummy_delay(50000);
    add_data_spi(0x6B, 0b01); // disable sleep mode
    dummy_delay(50000);
    who_am_i = r_data_spi(0x75); // Check communication with ICM
    add_data_spi(0x6C, 0b0); // Enable all accelerometer and gyroscope axes
    add_data_spi(0x1B, GYRO_SCALE << 3); // Set gyroscope sensitivity
    add_data_spi(0x1C, ACCEL_SCALE << 3); // Set accelerometer sensitivity

    while (1) {
        uninterrupt_r_data_spi(0x3B, 14); // Function for continuous reading of 14 registers

        // Calculate acceleration values and angular rates from raw data
        accel_x_data = (float)(int16_t)((storage[0] << 8) | storage[1]) / accel_sensitivity[ACCEL_SCALE];
        accel_y_data = (float)(int16_t)((storage[2] << 8) | storage[3]) / accel_sensitivity[ACCEL_SCALE];
        accel_z_data = (float)(int16_t)((storage[4] << 8) | storage[5]) / accel_sensitivity[ACCEL_SCALE];
        gyro_x_data = (float)(int16_t)((storage[8] << 8) | storage[9]) / gyro_sensitivity[GYRO_SCALE];
        gyro_y_data = (float)(int16_t)((storage[10] << 8) | storage[11]) / gyro_sensitivity[GYRO_SCALE];
        gyro_z_data = (float)(int16_t)((storage[12] << 8) | storage[13]) / gyro_sensitivity[GYRO_SCALE];
    }
}

void spi_set(void) {
    SPI1->CR1 |= 0b1 << SPI_CR1_SSM_Pos; // software CS management
    SPI1->CR1 |= 0b1 << SPI_CR1_SSI_Pos;
    SPI1->CR1 |= 0b001 << SPI_CR1_BR_Pos; // frequency 4MHz (ICM max 10MHz)
    SPI1->CR1 |= 0b1 << SPI_CR1_MSTR_Pos; // master configuration
    SPI1->CR1 |= 0b1 << SPI_CR1_CPOL_Pos; // CPOL = 1
    SPI1->CR1 |= 0b1 << SPI_CR1_CPHA_Pos; // CPHA = 1
    SPI1->CR2 |= 0b0111 << SPI_CR2_DS_Pos; // 8 bit data length
    SPI1->CR2 |= 0b1 << SPI_CR2_FRXTH_Pos; // RXFIFO threshold flag for 8 bits
}

void rewrite_data_spi(uint8_t address, uint8_t data) {
    GPIOA->BSRR = GPIO_BSRR_BR4;

    send_recieve_byte_of_data(address);
    send_recieve_byte_of_data(data);

    while (SPI1->SR & SPI_SR_BSY);

    GPIOA->BSRR = GPIO_BSRR_BS4;
}

void add_data_spi(uint8_t address, uint8_t data) {
    uint8_t data_for_send = r_data_spi(address) | data;

    GPIOA->BSRR = GPIO_BSRR_BR4;

    send_recieve_byte_of_data(address);
    send_recieve_byte_of_data(data_for_send);

    while (SPI1->SR & SPI_SR_BSY);

    GPIOA->BSRR = GPIO_BSRR_BS4;
}

uint8_t r_data_spi(uint8_t address) {
    uint8_t r_data = 0;
    address = address | 0x80;

    GPIOA->BSRR = GPIO_BSRR_BR4;

    send_recieve_byte_of_data(address);
    r_data = send_recieve_byte_of_data(0x55);

    while (SPI1->SR & SPI_SR_BSY);
    GPIOA->BSRR = GPIO_BSRR_BS4;

    return r_data;
}

void uninterrupt_r_data_spi(uint8_t address, uint32_t num_of_addresses) {
    address = address | 0x80;

    GPIOA->BSRR = GPIO_BSRR_BR4;

    send_recieve_byte_of_data(address);

    for (int i = 0; i < num_of_addresses; i += 1) {
        storage[i] = send_recieve_byte_of_data(0x55);
    }

    while (SPI1->SR & SPI_SR_BSY);
    GPIOA->BSRR = GPIO_BSRR_BS4;
}

void uninterrupt_w_data_spi(uint8_t address, uint32_t num_of_addresses, uint8_t storage[MAX_ARRAY_SIZE]) {
    GPIOA->BSRR = GPIO_BSRR_BR4;

    send_recieve_byte_of_data(address);

    for (int i = 0; i < num_of_addresses; i += 1) {
        send_recieve_byte_of_data(storage[i]);
    }

    while (SPI1->SR & SPI_SR_BSY);
    GPIOA->BSRR = GPIO_BSRR_BS4;
}

uint8_t send_recieve_byte_of_data(uint8_t send_data) {
    uint8_t recieve_data;

    while (!(SPI1->SR & SPI_SR_TXE));

    *((volatile uint8_t *) &SPI1->DR) = send_data;

    while (!(SPI1->SR & SPI_SR_RXNE));

    recieve_data = SPI1->DR;

    return recieve_data;
}

void dummy_delay(uint32_t duration) {
    for (uint32_t i = 0; i < duration; i += 1);
}
