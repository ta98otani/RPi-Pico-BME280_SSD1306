# Raspberry Pi Pico C++ Example Code 
This is an example code to code Raspberry pi pico in C++  
It simply retrieve data from BME280 sensors (temp, humidity, and pressure), and show on SSD1306 based OLED display.  
Raspberry Pi Picoのテストプロジェクトです。BME280からの値をSSD1306にて表示するC++サンプルコードです
私の環境は、macOS Big Sur 11.3 ですが、windowsやその他の環境でも問題無いと思います

## Hardware Setup
Connect your SSD1306 oled display, and BME280 to i2c1 port  
In this example, connection was made as follows:  
このサンプルでは、下記の様にブレッドボード上にBME280とSSD1306を配置しました

```
SDA->GP14(pin 19)  
SCL->GP15(pin 20)  
GND->GND  
VCC->3V3(OUT)  
```
![alt text](https://github.com/ta98otani/RPi-Pico-BME280_SSD1306/blob/master/BME280_SSD1306.jpeg?raw=true)  

BME280 from Amazon / BME280は下記を使いました  
https://www.amazon.co.jp/gp/product/B01M98R905/ref=ppx_yo_dt_b_asin_title_o06_s00?ie=UTF8&psc=1  
SSD1306 from Amazon / SSD1306は下記です  
https://www.amazon.co.jp/gp/product/B07D9H83R4/ref=ppx_yo_dt_b_asin_title_o02_s00?ie=UTF8&psc=1  

## Software
Please make sure you have pico-sdk installed with necessary environment setup  
```
git clone https://github.com/raspberrypi/pico-sdk.git  
```

For this example, use following     
```
git clone https://github.com/ta98otani/RPi-Pico-BME280_SSD1306.git  
cd RPi-Pico-BME280_SSD1306  
git clone https://github.com/mbober1/RPi-Pico-SSD1306-library.git  
```
(This example uses SSD1306 code above.  Thanks to @mbober1)  

--------  
Takuya Otani  
<ta98otani@gmail.com>  

