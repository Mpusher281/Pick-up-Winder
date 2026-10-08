#include <TMCStepper.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

// ============================================================
//  TWO-STEPPER COIL WINDER (TMC2209 UART + CORRECTIONS)
// ============================================================

#define RX1_PIN 2
#define TX1_PIN 3
#define STEP1_PIN 4
#define DIR1_PIN 5
#define DIAG1_PIN 6

#define RX2_PIN 10
#define TX2_PIN 11
#define STEP2_PIN 12
#define DIR2_PIN 13
#define DIAG2_PIN 9

#define HOME_SWITCH_PIN 7

// ------------------------------------------------------------
// SWITCH LOGIC CONFIGURATION
// Set to 'true' if switch reads LOW when unpressed (NC wiring)
// Set to 'false' if switch reads HIGH when unpressed (NO wiring)
// ------------------------------------------------------------
#define INVERT_SWITCH_LOGIC true

bool isSwitchTriggered() {
  bool pinState = digitalRead(HOME_SWITCH_PIN);
  return INVERT_SWITCH_LOGIC ? (pinState == HIGH) : (pinState == LOW);
}

#define HOME_SWITCH_TRIGGERED isSwitchTriggered()

#define R_SENSE 0.11f
#define DRIVER_ADDRESS 0b00
#define DRIVER2_ADDRESS 0b01

SoftwareSerial serialDriver1(RX1_PIN, TX1_PIN);
SoftwareSerial serialDriver2(RX2_PIN, TX2_PIN);

TMC2209Stepper driver1(&serialDriver1, R_SENSE, DRIVER_ADDRESS);
TMC2209Stepper driver2(&serialDriver2, R_SENSE, DRIVER2_ADDRESS);

const uint16_t MOTOR_FULL_STEPS = 200;
const float LEAD_SCREW_PITCH_MM = 8.0;
const float HOMING_MAX_TRAVEL_MM = 279.4;

// --- PHYSICAL MOTOR DIRECTIONS ---
#define DIR_TOWARD_SWITCH HIGH
#define DIR_AWAY_FROM_SWITCH LOW

#define CONFIG_MAGIC 0xC01B

struct Config {
  uint16_t magic;
  float bobbinWidthMM;
  float bobbinCircMM;
  float wireGaugeAWG;
  float wireDiaMM;
  float targetLengthM;
  uint32_t targetTurns;
  float homeOffsetMM;
  uint16_t spindleRPM;
  uint16_t microsteps;
  uint16_t currentSpindleMA;
  uint16_t currentTraverseMA;
  uint8_t  sgthrsSpindle;
  uint8_t  sgthrsTraverse;
};

Config cfg;

float awgToDiaMM(float awg);
void recomputeDerived();
void printStatus();
void printHelp();
void handleCommand(String cmd);
void homeTraverse();
void saveConfig();
void loadConfig();

void loadDefaults() {
  cfg.magic = CONFIG_MAGIC;
  cfg.bobbinWidthMM = 11.0;
  cfg.bobbinCircMM  = 45.0;
  cfg.wireGaugeAWG  = 42.0;
  cfg.wireDiaMM     = awgToDiaMM(cfg.wireGaugeAWG);
  cfg.targetLengthM = 0;
  cfg.targetTurns   = 5000;
  cfg.homeOffsetMM  = 0.7;
  cfg.spindleRPM    = 300;
  cfg.microsteps    = 16;
  cfg.currentSpindleMA  = 1200;
  cfg.currentTraverseMA = 800;
  cfg.sgthrsSpindle    = 5;
  cfg.sgthrsTraverse   = 5;
}

uint16_t stepsPerRev = 3200;
float traverseStepsPerSpindleStep = 0.0;
uint32_t maxTraverseStepsPerSweep = 0;
unsigned long spindleStepIntervalMicros = 250;

uint32_t totalSpindleSteps = 0;
uint32_t currentTurnCount = 0;
bool isWinding = false;
bool isHomed = false;

unsigned long lastSpindleStepMicros = 0;
float traverseAccumulator = 0.0;
uint8_t traverseDir = DIR_TOWARD_SWITCH;
uint32_t traverseStepsInCurrentSweep = 0;

float awgToDiaMM(float awg) {
  float bareMM = 0.127 * pow(92.0, (36.0 - awg) / 39.0);
  float insulationAllowanceMM = 0.038;
  return bareMM + insulationAllowanceMM;
}

void recomputeDerived() {
  stepsPerRev = MOTOR_FULL_STEPS * cfg.microsteps;
  traverseStepsPerSpindleStep = cfg.wireDiaMM / LEAD_SCREW_PITCH_MM;
  maxTraverseStepsPerSweep = (uint32_t)((cfg.bobbinWidthMM / LEAD_SCREW_PITCH_MM) * stepsPerRev);

  float stepsPerSec = ((float)stepsPerRev * cfg.spindleRPM) / 60.0;
  if (stepsPerSec < 1.0) stepsPerSec = 1.0;
  spindleStepIntervalMicros = (unsigned long)(1000000.0 / stepsPerSec);

  if (cfg.targetLengthM > 0 && cfg.bobbinCircMM > 0) {
    float lengthMM = cfg.targetLengthM * 1000.0;
    cfg.targetTurns = (uint32_t)ceil(lengthMM / cfg.bobbinCircMM);
  }
}

void saveConfig() {
  EEPROM.put(0, cfg);
  Serial.println(F(">>> Config saved to EEPROM. <<<"));
}

void loadConfig() {
  Config temp;
  EEPROM.get(0, temp);
  if (temp.magic == CONFIG_MAGIC) {
    cfg = temp;
    Serial.println(F(">>> Config loaded from EEPROM. <<<"));
  } else {
    loadDefaults();
    Serial.println(F(">>> No valid saved config found, using defaults. <<<"));
  }
  recomputeDerived();
}

void stepTraverse(uint16_t delayUs) {
  digitalWrite(STEP2_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(STEP2_PIN, LOW);
  delayMicroseconds(delayUs);
}

void homeTraverse() {
  Serial.println(F(">>> Homing traverse... <<<"));

  // 1. If switch is ALREADY closed, move AWAY from it first
  if (HOME_SWITCH_TRIGGERED) {
    Serial.println(F("  [debug] Switch detected TRIGGERED on start. Backing off AWAY..."));
    digitalWrite(DIR2_PIN, DIR_AWAY_FROM_SWITCH);
    traverseDir = DIR_AWAY_FROM_SWITCH;

    uint32_t initialBackoffSteps = 0;
    uint32_t initialBackoffLimit = (uint32_t)((HOMING_MAX_TRAVEL_MM / LEAD_SCREW_PITCH_MM) * stepsPerRev);

    while (HOME_SWITCH_TRIGGERED && initialBackoffSteps < initialBackoffLimit) {
      stepTraverse(1200);
      initialBackoffSteps++;
    }

    if (initialBackoffSteps >= initialBackoffLimit) {
      Serial.println(F("!!! HOMING FAILED - switch stuck triggered. Check INVERT_SWITCH_LOGIC or wiring !!!"));
      isHomed = false;
      return;
    }
    delay(100);
  }

  // 2. Fast approach TOWARD the switch
  Serial.println(F("  [debug] Fast approach TOWARD switch..."));
  digitalWrite(DIR2_PIN, DIR_TOWARD_SWITCH);
  traverseDir = DIR_TOWARD_SWITCH;

  uint32_t safetyStepLimit = (uint32_t)((HOMING_MAX_TRAVEL_MM / LEAD_SCREW_PITCH_MM) * stepsPerRev);
  uint32_t stepsTaken = 0;
  bool homed = false;

  while (!homed && stepsTaken < safetyStepLimit) {
    stepTraverse(2000);
    stepsTaken++;
    if (HOME_SWITCH_TRIGGERED) {
      homed = true;
    }
  }

  if (!homed) {
    Serial.println(F("!!! HOMING FAILED - switch never triggered !!!"));
    isHomed = false;
    return;
  }

  // 3. Clear switch contacts by moving AWAY
  Serial.println(F("  [debug] Backing off AWAY from switch..."));
  digitalWrite(DIR2_PIN, DIR_AWAY_FROM_SWITCH);
  traverseDir = DIR_AWAY_FROM_SWITCH;
  for (int i = 0; i < 300; i++) {
    stepTraverse(1000);
  }
  delay(100);

  // 4. Slow precision approach TOWARD switch
  Serial.println(F("  [debug] Slow re-approach TOWARD switch..."));
  digitalWrite(DIR2_PIN, DIR_TOWARD_SWITCH);
  traverseDir = DIR_TOWARD_SWITCH;

  bool homedSlow = false;
  stepsTaken = 0;
  while (!homedSlow && stepsTaken < 1500) {
    stepTraverse(2500);
    stepsTaken++;
    if (HOME_SWITCH_TRIGGERED) homedSlow = true;
  }

  if (!homedSlow) {
    Serial.println(F("!!! HOMING FAILED - slow approach missed switch !!!"));
    isHomed = false;
    return;
  }

  Serial.println(F(">>> Home switch triggered. <<<"));

  // 5. Move to home offset (AWAY from switch to flatwork edge)
  if (cfg.homeOffsetMM > 0) {
    uint32_t offsetSteps = (uint32_t)((cfg.homeOffsetMM / LEAD_SCREW_PITCH_MM) * stepsPerRev);
    Serial.println(F("  [debug] Applying offset, moving AWAY from switch..."));
    digitalWrite(DIR2_PIN, DIR_AWAY_FROM_SWITCH);
    traverseDir = DIR_AWAY_FROM_SWITCH;
    for (uint32_t i = 0; i < offsetSteps; i++) {
      stepTraverse(800);
    }
  }

  traverseStepsInCurrentSweep = 0;
  isHomed = true;
  Serial.println(F(">>> Homing complete. Traverse zeroed. <<<"));
}

void setup() {
  Serial.begin(115200);

  pinMode(STEP1_PIN, OUTPUT);
  pinMode(DIR1_PIN, OUTPUT);
  pinMode(STEP2_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);

  pinMode(DIAG1_PIN, INPUT);
  pinMode(DIAG2_PIN, INPUT);
  pinMode(HOME_SWITCH_PIN, INPUT_PULLUP);

  loadConfig();

  traverseDir = DIR_TOWARD_SWITCH;
  digitalWrite(DIR1_PIN, HIGH);
  digitalWrite(DIR2_PIN, traverseDir);

  // Initialize Driver 1 via UART
  serialDriver1.begin(9600);
  serialDriver1.listen();
  driver1.begin();
  driver1.toff(5);
  driver1.rms_current(cfg.currentTraverseMA);
  driver1.microsteps(cfg.microsteps);
  driver1.pwm_autoscale(true);
  driver1.en_spreadCycle(false);
  driver1.TCOOLTHRS(0xFFFFF);
  driver1.SGTHRS(cfg.sgthrsTraverse);

  delay(10);
  uint8_t version1 = driver1.version();
  Serial.print(F("Driver 1 UART: "));
  Serial.println(version1 == 0x21 ? "Connected!" : "FAILED");

  // Initialize Driver 2 via UART
  serialDriver2.begin(9600);
  serialDriver2.listen();
  driver2.begin();
  driver2.toff(5);
  driver2.rms_current(cfg.currentTraverseMA);
  driver2.microsteps(cfg.microsteps);
  driver2.pwm_autoscale(true);
  driver2.en_spreadCycle(false);
  driver2.TCOOLTHRS(0xFFFFF);
  driver2.SGTHRS(cfg.sgthrsTraverse);

  delay(10);
  uint8_t version2 = driver2.version();
  Serial.print(F("Driver 2 UART: "));
  Serial.println(version2 == 0x21 ? "Connected!" : "FAILED");

  // --- STARTUP SWITCH DIAGNOSTIC ---
  Serial.println(F("\n--- Home Switch Diagnostic ---"));
  Serial.print(F("Raw Pin 7 State: "));
  Serial.println(digitalRead(HOME_SWITCH_PIN) == HIGH ? "HIGH (5V)" : "LOW (GND)");
  Serial.print(F("Evaluated Triggered State: "));
  Serial.println(HOME_SWITCH_TRIGGERED ? "TRIGGERED" : "OPEN");
  Serial.println(F("------------------------------\n"));

  homeTraverse();

  Serial.println(F("\n=========================================="));
  Serial.println(F("          COIL WINDER READY               "));
  Serial.println(F("=========================================="));
  printStatus();
}

void printHelp() {
  Serial.println(F("--- Commands ---"));
  Serial.println(F("START, STOP, HOME, STATUS, SAVE, LOAD, DEFAULTS"));
}

void printStatus() {
  Serial.println(F("--- Status ---"));
  Serial.print(F("Homed: ")); Serial.println(isHomed ? "yes" : "no");
  Serial.print(F("Turns: ")); Serial.print(currentTurnCount); Serial.print(F(" / ")); Serial.println(cfg.targetTurns);
  Serial.print(F("AWG: ")); Serial.println(cfg.wireGaugeAWG);
  Serial.print(F("Bobbin Width: ")); Serial.print(cfg.bobbinWidthMM); Serial.println(F(" mm"));
  Serial.print(F("Spindle Speed: ")); Serial.print(cfg.spindleRPM); Serial.println(F(" RPM"));
}

void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  String upper = cmd;
  upper.toUpperCase();

  if (upper.equals("START")) {
    if (!isHomed) {
      Serial.println(F("!!! Not homed - run HOME first. !!!"));
      return;
    }
    currentTurnCount = 0;
    totalSpindleSteps = 0;
    traverseStepsInCurrentSweep = 0;
    traverseAccumulator = 0.0;

    traverseDir = DIR_AWAY_FROM_SWITCH;
    digitalWrite(DIR2_PIN, traverseDir);

    isWinding = true;
    Serial.println(F(">>> Winding Started! <<<"));
    return;
  }
  if (upper.equals("STOP")) {
    isWinding = false;
    Serial.println(F(">>> Winding Stopped <<<"));
    return;
  }
  if (upper.equals("HOME")) {
    isWinding = false;
    homeTraverse();
    return;
  }
  if (upper.equals("STATUS")) {
    printStatus();
    return;
  }
  if (upper.equals("SAVE")) {
    saveConfig();
    return;
  }
  if (upper.equals("LOAD")) {
    loadConfig();
    return;
  }
  if (upper.equals("DEFAULTS")) {
    loadDefaults();
    recomputeDerived();
    Serial.println(F(">>> Loaded default settings. <<<"));
    return;
  }
  if (upper.equals("HELP")) {
    printHelp();
    return;
  }
}

void loop() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    handleCommand(cmd);
  }

  if (isWinding) {
    if (digitalRead(DIAG1_PIN) == HIGH || digitalRead(DIAG2_PIN) == HIGH) {
      isWinding = false;
      Serial.println(F("!!! STALL DETECTED - EMERGENCY STOP !!!"));
    }

    if (HOME_SWITCH_TRIGGERED) {
      isWinding = false;
      Serial.println(F("!!! HOME SWITCH HIT DURING WINDING - STOP !!!"));
    }

    if (!isWinding) return;

    unsigned long currentMicros = micros();

    if (currentMicros - lastSpindleStepMicros >= spindleStepIntervalMicros) {
      lastSpindleStepMicros = currentMicros;

      digitalWrite(STEP1_PIN, HIGH);
      delayMicroseconds(2);
      digitalWrite(STEP1_PIN, LOW);

      totalSpindleSteps++;

      traverseAccumulator += traverseStepsPerSpindleStep;
      if (traverseAccumulator >= 1.0) {
        traverseAccumulator -= 1.0;

        digitalWrite(STEP2_PIN, HIGH);
        delayMicroseconds(2);
        digitalWrite(STEP2_PIN, LOW);

        traverseStepsInCurrentSweep++;

        if (traverseStepsInCurrentSweep >= maxTraverseStepsPerSweep) {
          traverseStepsInCurrentSweep = 0;
          traverseDir = (traverseDir == DIR_AWAY_FROM_SWITCH) ? DIR_TOWARD_SWITCH : DIR_AWAY_FROM_SWITCH;
          digitalWrite(DIR2_PIN, traverseDir);
        }
      }

      if (totalSpindleSteps % stepsPerRev == 0) {
        currentTurnCount++;
        if (currentTurnCount >= cfg.targetTurns) {
          isWinding = false;
          Serial.println(F(">>> TARGET TURNS REACHED! <<<"));
        }
      }
    }
  }
}
