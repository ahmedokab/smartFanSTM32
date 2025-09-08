# smartFanSTM32
A Smart Fan with a Telemetry system programmed in C using an STM32
Project by Ahmed Okab

## 🔌 Circuit Diagram!
<p align="center">
  <img src="https://github.com/user-attachments/assets/5a2f8220-a0d4-435c-8e2c-9c0904757b90" width="500" />
</p>

##  Overview
This project implements a **closed-loop smart fan system** using an STM32 microcontroller.  
The fan’s speed is automatically adjusted based on temperature readings, with live telemetry sent to a PC terminal ( I used PuTTY personally ).

The project demonstrates integration of multiple STM32 peripherals:
- **PWM** for fan motor speed control  
- **ADC** to read a temperature sensor (NTC thermistor)  
- **UART** to stream telemetry data to a serial terminal  
-  **I²C** with BME280 sensor and **SPI/I²C OLED display** for on-device visualization
-  In the end, I switched to the I2C option for greater accuracy, but the  code provides an option for both via an if statement

---

##  Features
- Reads temperature in real time using an **NTC thermistor** voltage divider  
- Uses a **feedback loop** to adjust fan PWM duty cycle according to temperature  
- Streams temperature and fan speed data to **PuTTY / Arduino Serial Monitor** at 115200 baud  
- Tested with **9V motor + N-MOSFET driver circuit** sf
- Modular design allows swapping the thermistor for a **BME280 digital sensor** over I²C
  

---

##  Hardware Used
- STM32 Nucleo F411RE
- NTC Thermistor (10kΩ, Vishay NTCLE100E3) + 10kΩ resistor voltage divider  
- MOSFET (FQP30N06L) for fan driver (logic-level N-channel)  
- DC Fan 12V) powered from external adapter or battery  
- BME280 for digital temp/humidity/pressure over I²C  
- SSD1306 OLED for real-time display via SPI/I²C  
- Breadboard + jumper wires + a bunch of resistors
- USB cable for STM32 programming and UART telemetry  

## Software
- **IDE**: STM32CubeIDE  
- **Drivers**: STM32 HAL (generated via CubeMX)  
- **Language**: C  
- **UART Terminal**: PuTTY / Arduino Serial Monitor (115200 baud, 8N1)

Results over PuTTY when in NTC mode: 


<img width="285" height="318" alt="image" src="https://github.com/user-attachments/assets/44117303-2aca-4638-9802-3a1fd89f0079" />

Used 84Mhz frequency, period = 1000. So when the duty is stated to be 500, it means a 50% duty cycle as consistent with my STM32 IDE setup.



