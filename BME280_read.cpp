/*
 * This is a sample code for Raspberry Pi Pico 
 * Read data from BME280, and display the result on OLED display 
 * Assuming i2c1 port (pin 14 and 15) are used.  BME280 address is set to 0x76
 *
 * BME280 code is based on pico-examples from SPI. 
 * Original code is found in:  https://github.com/raspberrypi/pico-examples
 *  
 * SSD1306 library is needed.  Please perform following on the base directory of this project example
 * git clone https://github.com/mbober1/RPi-Pico-SSD1306-library.git
 * 
 * Copyright Takuya Otani
 * mailto:ta98otani@gmail.com
 */


#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "bme280_i2c.hpp"
#include <logo.hpp>
#include <GFX.hpp>


int main() {

    stdio_init_all();

    // This example will use i2c1, and Pin 14 and 15 on pico
    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(14, GPIO_FUNC_I2C);
    gpio_set_function(15, GPIO_FUNC_I2C);
    gpio_pull_up(14);
    gpio_pull_up(15);

    int32_t humidity, pressure, temperature;

    GFX oled(0x3C, size::W128xH32, i2c1);   //Declare oled instance
    oled.display(logo);                     //Display bitmap
    
    //Setup BME280 Sensor 
    //Address of 0x76 is default for I2C line
    BME280 Sensor(i2c1, 0x76);

    char humidityChar[32];
    char pressureChar[32];
    char tempChar[32];

    while (1) {
        Sensor.bme280_read(&humidity, &pressure, &temperature);

        sprintf(humidityChar, "Humidity = %.2f%%",   humidity / 1024.0);
        sprintf(pressureChar, "Pressure = %.2fhPa",  pressure / 100.0);
        sprintf(tempChar,     "Temp     = %.2fC",    temperature / 100.0);

        oled.clear();
        oled.drawString(0,0,  tempChar);
        oled.drawString(0,10, humidityChar);
        oled.drawString(0,20, pressureChar);
        oled.display();                     //Send buffer to the screen 
        sleep_ms(2000);
    }

    return 0;

}
