# ESP32-ADXL345-MicroPython-Driver
A basic driver for the ESP-WROOM-32 to read data from the ADXL-345.

I'm using:  
ESP-WROOM-32 (DEVKITV1) board, aka, an ESP32 for my controller.  
ADXL345 (GY-291) accelerometer breakout board.  

I wrote this driver to retreive data from the accelerometer, and then I wrote a test program to display it in a constant loop.  
You can modify the pins in the test program to suit where yours are plugged in. Other than that, it's really just plug and play.  
Data is displayed exactly how it is received. You can convert it to G's fairly easily if you want to, but I didn't want to assume anything.

Save the adxl345.py file to your ESP32, and you can just run the test program without saving it directly to the device.

The correct wiring is as follows:
| ESP32        | ADXL345        |
| ------------- |:-------------:|
| 3V3      | VCC |
| GND      | GND      |
| SDA | Pin 21      |
| SCL | Pin 22      |

To-Do:  
Develop more features into the driver  
G-unit to m/s^2 conversion  
Peak-hold acceleration  
Speed calculation  
Etc...?
