#define polepairs 11

#include <Arduino.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include <encoders/smoothing/SmoothingSensor.h>

BLDCMotor motor = BLDCMotor(polepairs);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
HallSensor sensor = HallSensor(PB6, PB7, PB8, polepairs);
SmoothingSensor smooth(sensor, motor);

float anticog_phase = 0.0f;
float anticog_ampl = 2.0f;
float anticog_multiplier = 11.0f;
float voltage_override = 1.0f;
float maybe_o = 0.0f;
float target_voltage = 7;
const int lut_size = 1000; // Adjust size as needed
float lut[lut_size]; // Lookup table
int indexa = 0;
void receiveLUT() {
    int index2 = 0;
    int counter = 0;
    Serial.println("ready for lut");

    while (index2 < lut_size) {
        if (Serial.available() >= 5) { // 4 bytes float + 1 bytes checksum
            byte buffer[4];
            for (int i = 0; i < 4; i++) {
                buffer[i] = Serial.read();
            }

            byte received_checksum = Serial.read();
            byte end_marker = Serial.read(); // Expecting '\n' or specific marker

            // Calculate checksum
            byte calculated_checksum = 0;
            for (int i = 0; i < 4; i++) {
                calculated_checksum ^= buffer[i];
            }

            if (calculated_checksum == received_checksum) {
                // Convert bytes to float
                float received_value;
                unsigned char *valueBytes = (unsigned char*)&received_value;
                for (int i = 0; i < 4; i++) {
                    valueBytes[i] = buffer[i];
                }

                lut[index2] = received_value;

                // Send acknowledgment
                Serial.println("A");


                index2++;
            } else {
                Serial.println("N");
            }
        }
            motor.loopFOC();
            motor.move(-1 * target_voltage);
    
    }

    //Serial.println("LUT Received");
}


void initialize_lut() {
    for (int i = 0; i < lut_size; i++) {
        lut[i] = 0.0f;
    }
}

void SerialComm() {
    if (Serial.available() > 0) {
        switch (Serial.peek()) {
            case 's': Serial.read(); Serial.print("s"); Serial.println(motor.shaftVelocity()); break;
            case 'b': Serial.read(); Serial.print("b"); Serial.println(anticog_multiplier); break;
            case 'U': Serial.read(); anticog_multiplier = Serial.parseFloat(); break;
            case 'L': Serial.read();  receiveLUT(); break;
            case 'O':
                
                Serial.read();
                maybe_o = Serial.parseFloat();
                voltage_override = (maybe_o >= 0.999) ? 1.0f : 0.0f;
                break;
            default: Serial.read(); break;
        }
    }
}

float anti_cog_voltage(float motor_angle) {
    float anticog_angle = fmod(motor_angle * anticog_multiplier, 2.0f * PI);
    indexa = int(((-1*anticog_angle / (2.0f * PI)) * float(lut_size-1)));  
    return lut[indexa];
}



void doA() { sensor.handleA(); }
void doB() { sensor.handleB(); }
void doC() { sensor.handleC(); }



void setup() {
    sensor.pullup = Pullup::USE_INTERN;
    smooth.phase_correction = -_PI_6;
    Serial.begin(1000000);
    SimpleFOCDebug::enable(&Serial);
    Serial.setTimeout(1);
    sensor.init();
    sensor.enableInterrupts(doA, doB, doC);
    motor.linkSensor(&smooth);
    driver.voltage_power_supply = 24;
    driver.init();
    motor.linkDriver(&driver);
    motor.voltage_sensor_align = 3;
    motor.foc_modulation = FOCModulationType::SinePWM;
    motor.controller = MotionControlType::torque;
    motor.init();
    motor.initFOC();
    initialize_lut();
    for (float i = 0.0f; i < 600000.0f; i++) {
        motor.loopFOC();
        motor.move(-1 * i * target_voltage / 600000);
    }
}

void loop() {
    SerialComm();
    //Serial.println("seems to be working");
    //Serial.println((-1 * target_voltage)+(anti_cog_voltage(motor.shaft_angle)));
    //Serial.println(float(indexa));
    for (int j = 0; j < 30; j++) {
        for (int i = 0; i < 300; i++) {
            motor.loopFOC();
            motor.move((-1 * target_voltage*voltage_override)+(anti_cog_voltage(motor.shaft_angle)));
        }
         if (motor.shaftVelocity() > -20){ // it's in rads/s
            voltage_override = 0.0f;
            target_voltage = 0.0f;

 }  
    }
}