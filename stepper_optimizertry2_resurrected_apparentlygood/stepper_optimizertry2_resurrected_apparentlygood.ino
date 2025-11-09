#include <Arduino.h>
#include <SimpleFOC.h>
#include <math.h>

// ---------- USER PINS (Raspberry Pi Pico 2W) ----------
static const int PIN_PWM_AH = 4;
static const int PIN_PWM_AL = 5;
static const int PIN_PWM_BH = 6;
static const int PIN_PWM_BL = 7;
static const int En_a       = 8;
static const int En_b       = 9;
static const int POT_PIN    = 26;  // ADC0
static const int CURR_PIN   = 28;  // ADC2

// ---------- MOTOR / DRIVER ----------
StepperMotor motor = StepperMotor(50);
StepperDriver4PWM driver = StepperDriver4PWM(PIN_PWM_AH, PIN_PWM_AL, PIN_PWM_BH, PIN_PWM_BL, En_a, En_b);

// ---------- SPEED / POT ----------
volatile float RPS       = 0.0f;
volatile float RPS_accel = 0.3f;    // rps^2 
volatile float rps_max   = 7.5f;
const float    HYST_RPS  = 0.5f;    // rps hysteresis band
unsigned long  lastMicros = 0;

// ---------- CURRENT READ ----------
float read_current_units() {
  // raw ADC voltage (0..3.3 V) proportional to current; absolute amps not required for minimization
  int raw = analogRead(CURR_PIN);
  return 3.3f * (float)raw / 4095.0f;
}

// ---------- OPTIMIZER (ΔV trim around base voltage) ----------
struct MinCurrentTrimOpt {
  // Voltage bounds (motor command)
  float Vmin = 3.0f;
  float Vmax = 24.0f;

  // Trim state (what we adjust): Vcmd = clamp(Vbase + trim)
  float trim = 0.0f;

  // Step logic
  float step     = 0.10f;   // initial trim step (V)
  float step_max = 0.30f;
  float step_min = 0.02f;
  int   dir      = +1;      // +1 / -1

  // Filtering & decision
  float Ibar   = 0.0f;      // EMA of current
  float alpha  = 0.15f;     // EMA coeff (0..1)
  float Iref   = INFINITY;  // drifting reference (best so far with drift)
  float eps    = 0.02f;     // significant improvement threshold 
  float drift  = 0.003f;    // upward drift rate for Iref (keeps optimizer adaptive)

  // Timing
  uint32_t lastTickUs = 0;
  uint32_t periodUs   = 10000;  // 10 ms tick
  int      settleTicks = 8;     // ~80 ms settle after each voltage change
  int      settleLeft  = 0;

  // simple rate-limited debugging
  uint16_t printEvery = 25;
  uint16_t printCnt   = 0;

  // helpers
  inline float clampV(float v) const { return constrain(v, Vmin, Vmax); }

  static inline float ema(float prev, float sample, float a) {
    return prev + a * (sample - prev);
  }

  void init() {
    trim = 0.0f;
    step = 0.10f;
    dir  = +1;
    Ibar = 0.0f;
    Iref = INFINITY;
    settleLeft  = settleTicks;
    lastTickUs  = micros();
    printCnt    = 0;
  }

  void applyCommand(float Vbase) {
    float Vcmd = clampV(Vbase + trim);
    motor.voltage_limit = Vcmd;
  }

  void tick(float Iraw, float Vbase) {
    uint32_t now = micros();
    if (now - lastTickUs < periodUs) {
      return; // wait until next tick
    }
    lastTickUs = now;

    // Update filtered current
    Ibar = ema(Ibar, Iraw, alpha);
    if (isinf(Iref)) Iref = Ibar;            // first sample
    // Let the "best" baseline drift upward to follow changing conditions
    if (Ibar > Iref) Iref += drift * (Ibar - Iref);
    else             Iref  = Ibar;           // if we see lower current, follow it immediately

    // Settling time after a voltage change
    if (settleLeft > 0) {
      settleLeft--;
      applyCommand(Vbase);
      return;
    }

    bool improved = (Ibar + eps < Iref);

    if (improved) {
      // Accept direction; slightly grow step
      step = min(step * 1.2f, step_max);
      // Advance trim in same direction
      trim += dir * step;
      // Keep command within bounds (relative to Vbase)
      float Vcmd = clampV(Vbase + trim);
      // If clamped, flip and shrink
      if (Vcmd == Vmin || Vcmd == Vmax) {
        trim = Vcmd - Vbase;
        dir  = -dir;
        step = max(step * 0.5f, step_min);
      }
      settleLeft = settleTicks;
      Iref = Ibar; // new local best
      applyCommand(Vbase);
    } else {
      // Backtrack: flip direction & shrink step
      dir  = -dir;
      step = max(step * 0.5f, step_min);
      trim += dir * step;
      // Bound again
      float Vcmd = clampV(Vbase + trim);
      if (Vcmd == Vmin || Vcmd == Vmax) {
        trim = Vcmd - Vbase;
      }
      settleLeft = settleTicks;
      applyCommand(Vbase);
      Serial.println(motor.voltage_limit);
    }

    // (Optional) light debug printing, rate-limited
   // if (++printCnt >= printEvery) {
   //   printCnt = 0;
  //    Serial.print("Ib"); Serial.print(Ibar, 3);
  //    Serial.print(" I"); Serial.print(Iref, 3);
 //     Serial.print(" V"); Serial.print(Vbase, 3);
 //     Serial.print(" t");  Serial.print(trim, 3);
 //     Serial.print(" s");  Serial.print(step, 3);
  //    Serial.print(" d");   Serial.println(dir);
  //  }
  }
} opt;

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  // while (!Serial) {}

  // GPIO modes (SimpleFOC may reconfigure as needed)
  pinMode(PIN_PWM_AH, OUTPUT_12MA);
  pinMode(PIN_PWM_AL, OUTPUT_12MA);
  pinMode(PIN_PWM_BH, OUTPUT_12MA);
  pinMode(PIN_PWM_BL, OUTPUT_12MA);
  pinMode(LED_BUILTIN, OUTPUT);

  // ADC
  analogReadResolution(12);

  // Driver
  driver.voltage_power_supply = 24.0f;
  driver.voltage_limit        = 24.0f;   // driver ceiling
  driver.pwm_frequency        = 30000;   // 30 kHz PWM
  if (!driver.init()) {
    Serial.println("Driver init failed!");
    while (1) { delay(1000); }
  }
  driver.enable();

  // Motor
  motor.linkDriver(&driver);
  motor.controller         = MotionControlType::velocity_openloop;
  motor.velocity_limit     = 50.0f;           // rad/s ceiling
  motor.foc_modulation     = FOCModulationType::SinePWM;
  motor.modulation_centered = false;
  motor.init();

  opt.init();               // initialize optimizer state (after motor.init)
  lastMicros = micros();

  Serial.println("Stepper: pot-controlled RPM + online ΔV trim for min current");
}

// ---------- LOOP ----------
void loop() {
  // --- Pot to target RPS ---
  int raw = analogRead(POT_PIN);
  float norm   = (float)raw / 4095.0f;      // 0..1
  float target = norm * rps_max;            // RPS target

  // --- Time step for RPS ramp ---
  unsigned long now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  lastMicros = now;

  // --- RPS ramp with hysteresis & accel limit ---
  float err = target - RPS;
  if (fabsf(err) > HYST_RPS) {
    float maxStep = RPS_accel * dt;
    if (err > 0.0f) RPS += min(err,  maxStep);
    else            RPS += max(err, -maxStep);
  }
  RPS = constrain(RPS, 0.0f, rps_max);

  // --- Base voltage from RPM (kept simple) ---
  float Vbase = (19.0f * (RPS / rps_max)) + 3.0f;   // maps 0..rps_max -> 3..16 V
  Vbase = constrain(Vbase, opt.Vmin, opt.Vmax);     // ensure within bounds

  // --- Read current & optimize trim at ~100 Hz ---
  float Iraw = read_current_units();
  opt.tick(Iraw, Vbase);    // opt writes motor.voltage_limit each decision tick

  // --- Command motor speed (rad/s) ---
  float cmd_rad_per_sec = RPS * 2.0f * PI;
  motor.move(cmd_rad_per_sec);  // one call per loop keeps things responsive

  // (Optional) You can call motor.loopFOC() here if using closed-loop in the future
  // motor.loopFOC();
}
