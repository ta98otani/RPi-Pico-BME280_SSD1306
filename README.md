# Raspberry Pico C++ Example Code 
This is an example code to code raspberry pi pico in C++  
It simply retrieve data from BME280 sensors (temp, humidity, and pressure), and show on SSD1306 based OLED display.  

## Hardware
Connect your SSD1306 oled display, and BME280 to i2c1 port  
In this example, connection was made as follows:  

SDA->GP14(pin 19)  
SCL->GP15(pin 20)  
GND->GND  
VCC->3V3(OUT)  

## Software
Please make sure you have pico-sdk installed with necessary environment setup  
git clone https://github.com/raspberrypi/pico-sdk.git  

For this example, use following     
git clone https://github.com/ta98otani/RPi-Pico-BME280_SSD1306.git  
cd RPi-Pico-BME280_SSD1306  
git clone https://github.com/mbober1/RPi-Pico-SSD1306-library.git  

(This example uses SSD1306 code above.  Thanks to @mbober1)  

--------  
Takuya Otani  
<ta98otani@gmail.com>  

