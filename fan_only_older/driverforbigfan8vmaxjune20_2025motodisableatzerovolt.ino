
 #define polepairs 11

 #include <Arduino.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include <encoders/smoothing/SmoothingSensor.h>
BLDCMotor motor = BLDCMotor(polepairs);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003, -64.0/7.0, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);
// hall sensor instance
HallSensor sensor = HallSensor(PB6,PB7,PB8, polepairs);
SmoothingSensor smooth(sensor, motor);

// Interrupt routine intialisation
// channel A and B callbacks
void doA(){sensor.handleA();}
void doB(){sensor.handleB();}
void doC(){sensor.handleC();}


// voltage set point variable
float target_voltage = 0;
bool voltage_override = 0;
float maybe_v = 1;
float voltage_max = 8;
float goal_v = voltage_max;
float accel = 1;// in volts per second
void SerialComm(){ 
  if (Serial.available() > 0){
  switch(Serial.peek()){
      case 'c': Serial.read(); Serial.print("c"); Serial.println(accel); break;
      case 'o': Serial.read(); Serial.print("o"); Serial.println(voltage_override); break;
      case 'v': Serial.read(); Serial.print("v"); Serial.println(motor.target); break;
      case 's': Serial.read(); Serial.print("s"); Serial.println(motor.shaftVelocity()); break;
      
      //case 'g': Serial.read(); Serial.print("g"); Serial.println(currentSense.getDCCurrent(), 5); break;
     // case 'i': Serial.read(); Serial.print("i"); Serial.println(get_mA(), 4); break;
      //case 'y': Serial.read(); Serial.print("y"); Serial.println(_readADCVoltageInline(A_POTENTIOMETER, currentSense.params)*0.305810398); break;
  case 'V': break;
  default: Serial.read(); break; //if anything we don't recognize got in the buffer, clear it out or it will mess things up.
       
  }
}
  if (Serial.available() >= 9){//if there are 9 or more characters in there
  switch(Serial.read())
  { 
  case 'V': 
    maybe_v = Serial.parseFloat(); // just in case the wrong data gets in somehow we don't want the voltage going crazy
    if (maybe_v < 0){

      maybe_v = 0;
      } 
    if (maybe_v >= voltage_max){ 
    
      maybe_v = voltage_max;
    }
    goal_v = maybe_v;
    break;
  }
  }
}
void setup() { 
  pinMode(PB4, INPUT_PULLUP);
  sensor.pullup = Pullup::USE_INTERN;
  smooth.phase_correction = -_PI_6;
  Serial.begin(115200);
  Serial.println("test serial2");
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

  // initialize encoder sensor hardware
  sensor.init();
  sensor.enableInterrupts(doA, doB, doC); 
  // link the motor to the sensor
  motor.linkSensor(&smooth);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 24;
  driver.init();
  // link driver
  motor.linkDriver(&driver);

  // aligning voltage
  motor.voltage_sensor_align = 1.1;
  
  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SinePWM;

  // set motion control loop to be used
  motor.controller = MotionControlType::torque;
  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

}


float looptime_seconds = 0;
bool underspeed_start = 0;
void loop() {
  static unsigned long underspeed_detect_start;
  static unsigned long lastTime = millis();
  unsigned long now = millis();
    unsigned long looptime_ms = now - lastTime;
  lastTime = now;
looptime_seconds = float(looptime_ms)/1000;
if (goal_v > (target_voltage+0.1)) {
  target_voltage = target_voltage + (looptime_seconds*accel);
}
if (goal_v < (target_voltage-0.1)) {
  target_voltage = target_voltage - (looptime_seconds*accel);
}
 for (int i = 0; i < 1000; i++){
  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz 
  motor.loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(-1*target_voltage);
 }
 if ((target_voltage >=1) && !underspeed_start && motor.shaftVelocity() > -5){ // it's in rads/s negative is the direction we are going. this is the stall detection
 underspeed_detect_start = millis();
 underspeed_start = true;
 }  
 if ( underspeed_start && motor.shaftVelocity() > -5){ // it's in rads/s negative is the direction we are going. this is the stall detection
 if ((millis() - underspeed_detect_start) > 5000){
 target_voltage = 0;
 goal_v = 0;
underspeed_start = false;

 }
 }
 if (motor.shaftVelocity() < -5){ // it's in rads/s negative is the direction we are going. this is the stall detection
  underspeed_start = false;
 }



 SerialComm();
}