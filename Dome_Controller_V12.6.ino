// Dome Controller V12.6
// Author: Clarke Yeager
// Date: 4/3/2026
//
// Observatory dome azimuth controller - Merged V1 + V11 features
// Controls DC motor via H-bridge for CW/CCW rotation
// Position tracking via encoder pulses stored in external EEPROM
//
// Features 
// - Shutter control (open/close with configurable times)
// - Remote power toggle
// - Status display
// - Tracking controlled by external bridge script (theskyx_dome_bridge.py)
// - Park offset positions dome opening for wind protection
// - Telescope offset compensation for off-center telescope mount (controled in the Bridge)
// - Meridian flip tracking (handled in the Bridge)
// - Configurable slow-down PWM and ramp speed
// - a move of less than 10 counts the rotator is moved at slow speed to facilitate good tracking 
// - Interactive configuration menu
// - 2-second home switch hold requirement
//
// Serial Commands:
//   0-359    Move dome to angle (degrees)
//   O        Open shutter (toggle remote, open, toggle remote off)
//   C        Close shutter (toggle remote, close, toggle remote off)
//   G        Go home (park)
//   S        Status display (position, shutter, telescope settings)
//   AO<val>  Set Shutter Open time (1-120 seconds)
//   AC<val>  Set Shutter Close time (1-120 seconds)
//   AT<val>  Set Shutter Tension relief time (1-10 seconds)
//   P<val>   Set Park offset (0-359 degrees)
//   W<val>   Set sloW-down PWM (50-150)
//   D<val>   Set ramp Delay/speed (1-20 ms)
//   Q        Query all configuration
//   I        Interactive configuration menu
//   ?        Help
//
// Manual Controls:
//   CW button    - Rotate clockwise while held
//   CCW button   - Rotate counter-clockwise while held
//   Home button  - Hold 2 seconds to park at home position
//
// NOTE: Set Serial Monitor to "No Line Ending" when connected via USB

#include <Wire.h>
#include <I2C_eeprom.h>

// ==================== Version Info ====================

const char* CodeVersion = "V12.6";
const char* CodeDate = "4/3/2026";

// ==================== Configuration Constants ====================

// Dome geometry parameters
const float DomeRingHoles = 175.0;  // Holes in dome ring
const float GearCogs = 14.0;        // Cogs on drive gear
const float GearTachs = 24.0;       // Encoder pulses per motor revolution

// Calculated constants
const float TachsPerDomeRev = (DomeRingHoles / GearCogs) * GearTachs;  // 300 pulses per 360 deg
const float DegreesPerTach = 360.0 / TachsPerDomeRev;                   // 1.2 deg per pulse

// Motor control parameters (fixed)
const int MaxPower = 255;   // Maximum PWM
const unsigned long HomeHoldTime = 2000;  // Required hold time for home switch (ms)
const unsigned long MotorTimeoutMs = 30000;  // Motor timeout if no encoder pulses (30 sec)
const unsigned long HomingTimeoutMs = 60000;  // Homing timeout per phase (60 sec)
const int OvershootCompensation = 0;  // Pulses to subtract for motor/inertia overshoot
const int BacklashDegrees = 10;       // Degrees to overshoot for backlash compensation

// ==================== EEPROM Configuration ====================

const int EepromAddress = 0x57;

// EEPROM Address Allocation (Merged)
const int PositionAddress = 12;          // Position count (2 bytes)
// Addresses 14-17 reserved (were tracking move time and rate in V12.3)
const int OpenTimeAddress = 18;          // Shutter open time in seconds (2 bytes)
const int CloseTimeAddress = 20;         // Shutter close time in seconds (2 bytes)
const int ParkOffsetAddress = 22;        // Park offset in degrees (2 bytes)
const int SlowPWMAddress = 24;           // Slow-down PWM (1 byte)
const int RampSpeedAddress = 25;         // Ramp speed in ms (1 byte)
// Addresses 26-27 reserved (were telescope offset and scope side in V12.3)
const int ShutterStateAddress = 28;     // Shutter state: 0=unknown, 1=open, 2=closed (1 byte)
const int TensionReliefAddress = 29;     // Tension relief time in seconds (1 byte)
const int ConfigValidAddress = 30;       // Config valid marker 0xAA (1 byte)
// Addresses 31-38 reserved (were dead zone angles in V12.3)

// ==================== Default Configuration Values ====================

const int DefaultOpenTime = 30;          // Default shutter open time (seconds)
const int DefaultCloseTime = 30;         // Default shutter close time (seconds)
const int DefaultTensionRelief = 1;      // Default tension relief time (seconds)
const int DefaultParkOffset = 140;       // Default park offset (degrees)
const int DefaultSlowPWM = 75;           // Default slow-down PWM value
const int DefaultRampSpeed = 3;          // Default ramp speed (ms)

// ==================== Pin Assignments ====================

// H-bridge motor driver pins
const int TopLeftFET = 3;      // PWM - CW drive
const int TopRightFET = 5;     // PWM - CCW drive
const int BottomLeftFET = 4;   // Digital - CCW ground path
const int BottomRightFET = 2;  // Digital - CW ground path

// Control input pins
const int CWSwitch = 6;        // Manual CW button (INPUT_PULLUP)
const int CCWSwitch = 7;       // Manual CCW button (INPUT_PULLUP)
const int HomeSwitch = 8;      // Home button (INPUT_PULLUP)
const int StopHallPin = 9;     // Home stop sensor (interrupt)
const int SlowHallPin = 10;    // Home slow-down sensor (interrupt)
const int EncoderPin = 11;     // Encoder input (interrupt)

// Shutter control pins
const int PowerLEDPin = A0;    // Input: reads if remote controller is on/off (LOW = off)
const int OpenPin = A1;        // Output: activates shutter open
const int PWSwitchPin = A2;    // Output: pulled HIGH to toggle remote power
const int ClosePin = A3;       // Output: activates shutter close

// ==================== Global Objects ====================

I2C_eeprom eeprom(EepromAddress, I2C_DEVICESIZE_24LC256);

// ==================== Configurable Parameters ====================

// Shutter parameters (from V1)
int openTime = DefaultOpenTime;                  // Shutter open time in seconds
int closeTime = DefaultCloseTime;                // Shutter close time in seconds
int tensionRelief = DefaultTensionRelief;        // Tension relief time in seconds

// Dome parameters (from V11)
int parkOffset = DefaultParkOffset;    // Park offset in degrees
int slowPWM = DefaultSlowPWM;          // Slow-down PWM value
int rampSpeed = DefaultRampSpeed;      // Ramp speed in ms

// Shutter state tracking
// 0 = unknown, 1 = open, 2 = closed
int shutterState = 0;

// ==================== State Variables ====================

bool eepromAvailable = true;  // Track if EEPROM communication is working
volatile int encoderCount = 0;
volatile bool countCW = false;
volatile bool countCCW = false;
volatile bool encoderEnabled = true;
volatile bool slowDownTriggered = false;
volatile bool homeStopTriggered = false;
volatile bool positionChanged = false;  // Flag to defer EEPROM writes outside ISR
bool remoteError = false;               // Suppress display after remote power failure

int currentPower = DefaultSlowPWM;

// ==================== Forward Declarations ====================

void moveToAngle(int targetDegrees);
void configurationMenu();
void printHelp();
void displayStatus();

// ==================== EEPROM Functions ====================

bool checkEepromConnection() {
  // Check if EEPROM responds on I2C bus
  if (!eeprom.isConnected()) {
    if (eepromAvailable) {  // Only print once on first failure
      Serial.println("ERROR: EEPROM not responding on I2C bus!");
      Serial.println("Check wiring to 24LC256 at address 0x57.");
    }
    eepromAvailable = false;
    return false;
  }
  eepromAvailable = true;
  return true;
}

void savePosition(int count) {
  if (!eepromAvailable) return;  // Skip if EEPROM unavailable

  int result1 = eeprom.writeByte(PositionAddress, (count >> 8) & 0xFF);
  int result2 = eeprom.writeByte(PositionAddress + 1, count & 0xFF);

  if (result1 != 0 || result2 != 0) {
    Serial.println("WARNING: Failed to save position to EEPROM.");
    checkEepromConnection();  // Verify connection status
  }
}

int loadPosition() {
  if (!eepromAvailable) return 0;  // Return 0 if EEPROM unavailable

  int highByte = eeprom.readByte(PositionAddress);
  int lowByte = eeprom.readByte(PositionAddress + 1);
  int value = (highByte << 8) | lowByte;

  // Sanity check: position should be 0 to TachsPerDomeRev-1
  if (value < 0 || value >= (int)TachsPerDomeRev) {
    Serial.print("WARNING: Invalid position read from EEPROM: ");
    Serial.println(value);
    Serial.println("Defaulting to 0. Home the dome to calibrate.");
    return 0;
  }
  return value;
}

void write2BytesToEEPROM(int address, int value) {
  if (!eepromAvailable) return;

  int result1 = eeprom.writeByte(address, (value >> 8) & 0xFF);
  int result2 = eeprom.writeByte(address + 1, value & 0xFF);

  if (result1 != 0 || result2 != 0) {
    Serial.println("WARNING: EEPROM write failed.");
    checkEepromConnection();
  }
}

int read2BytesFromEEPROM(int address) {
  if (!eepromAvailable) return 0;

  int highByte = eeprom.readByte(address);
  int lowByte = eeprom.readByte(address + 1);
  return (highByte << 8) | lowByte;
}

// ==================== Configuration Functions ====================

void saveConfiguration() {
  if (!eepromAvailable) {
    Serial.println("WARNING: Cannot save - EEPROM unavailable.");
    return;
  }

  // Save parameters (2 bytes each)
  write2BytesToEEPROM(OpenTimeAddress, openTime);
  write2BytesToEEPROM(CloseTimeAddress, closeTime);
  write2BytesToEEPROM(ParkOffsetAddress, parkOffset);

  // Save single-byte parameters
  eeprom.writeByte(SlowPWMAddress, slowPWM & 0xFF);
  eeprom.writeByte(RampSpeedAddress, rampSpeed & 0xFF);
  eeprom.writeByte(ShutterStateAddress, shutterState & 0xFF);
  eeprom.writeByte(TensionReliefAddress, tensionRelief & 0xFF);

  // Write marker to indicate valid config
  eeprom.writeByte(ConfigValidAddress, 0xAA);

  delay(10);
}

void loadConfiguration() {
  // If EEPROM unavailable, use defaults
  if (!eepromAvailable) {
    Serial.println("EEPROM unavailable, using default configuration.");
    openTime = DefaultOpenTime;
    closeTime = DefaultCloseTime;
    parkOffset = DefaultParkOffset;
    slowPWM = DefaultSlowPWM;
    rampSpeed = DefaultRampSpeed;
    shutterState = 0;
    tensionRelief = DefaultTensionRelief;
    return;
  }

  // Check for valid config marker
  if (eeprom.readByte(ConfigValidAddress) != 0xAA) {
    Serial.println("No saved configuration found, using defaults.");
    openTime = DefaultOpenTime;
    closeTime = DefaultCloseTime;
    parkOffset = DefaultParkOffset;
    slowPWM = DefaultSlowPWM;
    rampSpeed = DefaultRampSpeed;
    shutterState = 0;  // Unknown
    tensionRelief = DefaultTensionRelief;
    return;
  }

  // Load 2-byte parameters
  openTime = read2BytesFromEEPROM(OpenTimeAddress);
  closeTime = read2BytesFromEEPROM(CloseTimeAddress);
  parkOffset = read2BytesFromEEPROM(ParkOffsetAddress);

  // Load single-byte parameters
  slowPWM = eeprom.readByte(SlowPWMAddress);
  rampSpeed = eeprom.readByte(RampSpeedAddress);
  shutterState = eeprom.readByte(ShutterStateAddress);
  tensionRelief = eeprom.readByte(TensionReliefAddress);

  // Validate ranges and apply defaults if invalid
  if (shutterState > 2) shutterState = 0;  // Unknown if invalid
  if (tensionRelief < 1 || tensionRelief > 10) tensionRelief = DefaultTensionRelief;
  if (openTime < 1 || openTime > 120) openTime = DefaultOpenTime;
  if (closeTime < 1 || closeTime > 120) closeTime = DefaultCloseTime;
  if (parkOffset < 0 || parkOffset > 359) parkOffset = DefaultParkOffset;
  if (slowPWM < 50 || slowPWM > 150) slowPWM = DefaultSlowPWM;
  if (rampSpeed < 1 || rampSpeed > 20) rampSpeed = DefaultRampSpeed;

  Serial.println("Configuration loaded from EEPROM.");
}

void displayConfiguration() {
  Serial.println();
  Serial.println("======= CURRENT CONFIGURATION =======");
  Serial.print("Shutter open time: ");
  Serial.print(openTime);
  Serial.println(" sec (AO)");
  Serial.print("Shutter close time: ");
  Serial.print(closeTime);
  Serial.println(" sec (AC)");
  Serial.print("Tension relief time: ");
  Serial.print(tensionRelief);
  Serial.println(" sec (AT)");
  Serial.print("Park offset: ");
  Serial.print(parkOffset);
  Serial.println(" deg (P)");
  Serial.print("Slow-down PWM: ");
  Serial.print(slowPWM);
  Serial.println(" (W)");
  Serial.print("Ramp speed: ");
  Serial.print(rampSpeed);
  Serial.println(" ms (D)");
  Serial.println("=====================================");
}

int getSerialInt() {
  while (Serial.available() == 0) {
    // Wait for input
  }
  return Serial.parseInt();
}

void configurationMenu() {
  Serial.println();
  Serial.println("******* CONFIGURATION MENU *******");

  bool configChanged = false;

  while (true) {
    Serial.println();
    Serial.println("1. Shutter open time (sec)");
    Serial.println("2. Shutter close time (sec)");
    Serial.println("3. Tension relief time (sec)");
    Serial.println("4. Park offset (deg)");
    Serial.println("5. Slow-down PWM");
    Serial.println("6. Ramp speed (ms)");
    displayConfiguration();
    Serial.println();
    Serial.println("Enter option (1-6), S to save, X to exit:");

    while (Serial.available() == 0) {
      // Wait for input
    }

    char input = Serial.read();
    // Clear any remaining characters
    while (Serial.available() > 0) Serial.read();

    switch (input) {
      case '1':
        Serial.print("Enter new shutter open time (1-120 sec): ");
        {
          int val = getSerialInt();
          if (val >= 1 && val <= 120) {
            openTime = val;
            configChanged = true;
            Serial.print("Shutter open time set to ");
            Serial.print(openTime);
            Serial.println(" sec");
          } else {
            Serial.println("Invalid value. Must be 1-120.");
          }
        }
        break;

      case '2':
        Serial.print("Enter new shutter close time (1-120 sec): ");
        {
          int val = getSerialInt();
          if (val >= 1 && val <= 120) {
            closeTime = val;
            configChanged = true;
            Serial.print("Shutter close time set to ");
            Serial.print(closeTime);
            Serial.println(" sec");
          } else {
            Serial.println("Invalid value. Must be 1-120.");
          }
        }
        break;

      case '3':
        Serial.print("Enter new tension relief time (1-10 sec): ");
        {
          int val = getSerialInt();
          if (val >= 1 && val <= 10) {
            tensionRelief = val;
            configChanged = true;
            Serial.print("Tension relief time set to ");
            Serial.print(tensionRelief);
            Serial.println(" sec");
          } else {
            Serial.println("Invalid value. Must be 1-10.");
          }
        }
        break;

      case '4':
        Serial.print("Enter new park offset (0-359 deg): ");
        {
          int val = getSerialInt();
          if (val >= 0 && val <= 359) {
            parkOffset = val;
            configChanged = true;
            Serial.print("Park offset set to ");
            Serial.print(parkOffset);
            Serial.println(" deg");
          } else {
            Serial.println("Invalid value. Must be 0-359.");
          }
        }
        break;

      case '5':
        Serial.print("Enter new slow-down PWM (50-150): ");
        {
          int val = getSerialInt();
          if (val >= 50 && val <= 150) {
            slowPWM = val;
            configChanged = true;
            Serial.print("Slow-down PWM set to ");
            Serial.println(slowPWM);
          } else {
            Serial.println("Invalid value. Must be 50-150.");
          }
        }
        break;

      case '6':
        Serial.print("Enter new ramp speed (1-20 ms): ");
        {
          int val = getSerialInt();
          if (val >= 1 && val <= 20) {
            rampSpeed = val;
            configChanged = true;
            Serial.print("Ramp speed set to ");
            Serial.print(rampSpeed);
            Serial.println(" ms");
          } else {
            Serial.println("Invalid value. Must be 1-20.");
          }
        }
        break;

      case 'S':
      case 's':
        saveConfiguration();
        configChanged = false;
        Serial.println("Configuration saved to EEPROM.");
        break;

      case 'X':
      case 'x':
        if (configChanged) {
          Serial.println("Warning: Unsaved changes will be lost.");
          Serial.println("Press Y to exit anyway, any other key to cancel:");
          while (Serial.available() == 0) {}
          char confirm = Serial.read();
          while (Serial.available() > 0) Serial.read();
          if (confirm != 'Y' && confirm != 'y') {
            continue;
          }
        }
        Serial.println("Exiting configuration menu.");
        return;

      default:
        Serial.println("Invalid option.");
        break;
    }
  }
}

// ==================== Position Conversion ====================

int tachsToDegrees(int tachs) {
  return (int)(tachs * DegreesPerTach);
}

int degreesToTachs(int degrees) {
  return (int)(degrees / DegreesPerTach);
}

// ==================== Safe Volatile Access ====================

// Safely read encoderCount from main code (prevents race condition on 2-byte read)
int getEncoderCount() {
  noInterrupts();
  int count = encoderCount;
  interrupts();
  return count;
}

// Safely write encoderCount from main code
void setEncoderCount(int value) {
  noInterrupts();
  encoderCount = value;
  interrupts();
}

// ==================== Position Save Helper ====================

void savePositionIfChanged() {
  // Safely save position to EEPROM if it changed (called from main code, not ISR)
  if (positionChanged) {
    noInterrupts();
    int countToSave = encoderCount;
    positionChanged = false;
    interrupts();
    savePosition(countToSave);
  }
}

// ==================== Motor Control ====================

void stopMotor() {
  analogWrite(TopLeftFET, 0);
  analogWrite(TopRightFET, 0);
  digitalWrite(BottomLeftFET, LOW);
  digitalWrite(BottomRightFET, LOW);
  delay(5);
}

void rampDown(int pwmPin, int &power) {
  while (power > slowPWM) {
    power--;
    analogWrite(pwmPin, power);
    delay(rampSpeed);
  }
  analogWrite(pwmPin, 0);
}

void runCW(int pwmPin) {
  if (currentPower < MaxPower) {
    currentPower++;
    analogWrite(pwmPin, currentPower);
    delay(rampSpeed);
  }
}

void runCCW(int pwmPin) {
  if (currentPower < MaxPower) {
    currentPower++;
    analogWrite(pwmPin, currentPower);
    delay(rampSpeed);
  }
}

// ==================== Interrupt Handlers ====================

void encoderISR() {
  if (!encoderEnabled) return;

  static unsigned long lastPulse = 0;
  unsigned long now = micros();
  if (now - lastPulse < 50000UL) return;  // Debounce: ignore pulses within 50ms
  lastPulse = now;

  if (countCW) {
    encoderCount++;
    if (encoderCount >= (int)TachsPerDomeRev) {  // Wrap 300 -> 0
      encoderCount = 0;
    }
    positionChanged = true;  // Defer EEPROM write to main code
  }
  else if (countCCW) {
    encoderCount--;
    if (encoderCount < 0) {  // Wrap -1 -> 299
      encoderCount = (int)TachsPerDomeRev - 1;
    }
    positionChanged = true;  // Defer EEPROM write to main code
  }
}

void slowDownISR() {
  slowDownTriggered = true;  // No delay in ISR - debounce in main code if needed
}

void homeStopISR() {
  homeStopTriggered = true;  // No delay in ISR - debounce in main code if needed
}

// ==================== Movement Functions ====================

void moveCW(int pulsesToMove, bool slowOnly = false) {
  // pulsesToMove: 0 = manual mode, >0 = move this many pulses
  // slowOnly: true = run at slowPWM only (no ramp up), for short moves
  countCW = true;          // Set direction before enabling ISR
  countCCW = false;
  encoderEnabled = true;   // Enable ISR last — direction flags already set
  currentPower = slowPWM;

  int pulsesMoved = 0;
  int lastCount = getEncoderCount();
  unsigned long lastPulseTime = millis();  // Timeout tracking
  int slowZonePulses = degreesToTachs(BacklashDegrees);  // Slow down within 10 deg of target

  digitalWrite(BottomRightFET, HIGH);
  delay(5);

  while (true) {
    bool nearTarget = (pulsesToMove > 0) && ((pulsesToMove - pulsesMoved) <= slowZonePulses);
    if (slowOnly || nearTarget) {
      currentPower = slowPWM;  // Keep in sync so rampDown starts from slow speed
      analogWrite(TopLeftFET, slowPWM);
    } else {
      runCW(TopLeftFET);
    }

    // Count pulses moved using delta to avoid missing pulses during delay()
    int currentCount = getEncoderCount();
    if (currentCount != lastCount) {
      int delta = currentCount - lastCount;
      if (delta < -150) delta += 300;  // Handle wrap 299 -> 0
      if (delta > 150)  delta -= 300;
      pulsesMoved += abs(delta);
      lastCount = currentCount;
      lastPulseTime = millis();  // Reset timeout on pulse
    }

    // Check for motor timeout (no encoder pulses)
    if (pulsesToMove > 0 && (millis() - lastPulseTime > MotorTimeoutMs)) {
      Serial.println("ERROR: Motor timeout - no encoder pulses detected.");
      Serial.println("Check encoder connection or motor drive.");
      break;
    }

    // Manual mode: exit when switch released
    if (pulsesToMove == 0) {
      if (digitalRead(CWSwitch) == HIGH) break;
    }
    // Programmatic mode: exit when pulses reached
    else {
      if (pulsesMoved >= pulsesToMove) break;
    }
  }

  rampDown(TopLeftFET, currentPower);
  digitalWrite(BottomRightFET, LOW);
  delay(5);

  encoderEnabled = false;  // Disable ISR before clearing direction flags
  countCW = false;
  savePositionIfChanged();  // Save position after movement completes
}

void moveCCW(int pulsesToMove, bool slowOnly = false) {
  // pulsesToMove: 0 = manual mode, >0 = move this many pulses
  // slowOnly: true = run at slowPWM only (no ramp up), for short moves
  countCW = false;         // Set direction before enabling ISR
  countCCW = true;
  encoderEnabled = true;   // Enable ISR last — direction flags already set
  currentPower = slowPWM;

  int pulsesMoved = 0;
  int lastCount = getEncoderCount();
  unsigned long lastPulseTime = millis();  // Timeout tracking

  digitalWrite(BottomLeftFET, HIGH);
  delay(5);

  while (true) {
    if (slowOnly) {
      analogWrite(TopRightFET, slowPWM);
    } else {
      runCCW(TopRightFET);
    }

    // Count pulses moved using delta to avoid missing pulses during delay()
    int currentCount = getEncoderCount();
    if (currentCount != lastCount) {
      int delta = currentCount - lastCount;
      if (delta < -150) delta += 300;  // Handle wrap 0 -> 299
      if (delta > 150)  delta -= 300;
      pulsesMoved += abs(delta);
      lastCount = currentCount;
      lastPulseTime = millis();  // Reset timeout on pulse
    }

    // Check for motor timeout (no encoder pulses)
    if (pulsesToMove > 0 && (millis() - lastPulseTime > MotorTimeoutMs)) {
      Serial.println("ERROR: Motor timeout - no encoder pulses detected.");
      Serial.println("Check encoder connection or motor drive.");
      break;
    }

    // Manual mode: exit when switch released
    if (pulsesToMove == 0) {
      if (digitalRead(CCWSwitch) == HIGH) break;
    }
    // Programmatic mode: exit when pulses reached
    else {
      if (pulsesMoved >= pulsesToMove) break;
    }
  }

  rampDown(TopRightFET, currentPower);
  digitalWrite(BottomLeftFET, LOW);
  delay(5);

  encoderEnabled = false;  // Disable ISR before clearing direction flags
  countCCW = false;
  savePositionIfChanged();  // Save position after movement completes
}

bool checkHomeSwitch() {
  // Home switch must be held for 2 seconds to activate
  if (digitalRead(HomeSwitch) == LOW) {
    unsigned long pressStart = millis();
    Serial.println("Home switch pressed - hold for 2 seconds to activate...");

    while (digitalRead(HomeSwitch) == LOW) {
      if (millis() - pressStart >= HomeHoldTime) {
        Serial.println("Home activated!");
        return true;
      }
    }
    Serial.println("Home switch released too early - cancelled.");
  }
  return false;
}

void goHome() {
  Serial.println("Homing to north (0 degrees)...");

  encoderEnabled = false;
  slowDownTriggered = false;
  homeStopTriggered = false;
  currentPower = slowPWM;

  int currentDegrees = tachsToDegrees(getEncoderCount());

  // Determine shortest path to north (0 degrees)
  // If position > 180, go CW (through 360/0)
  // If position <= 180, go CCW (toward 0)
  bool goClockwise = (currentDegrees > 180);
  bool homingFailed = false;
  unsigned long phaseStart;

  Serial.print("Current position: ");
  Serial.print(currentDegrees);
  Serial.println(" degrees");

  if (goClockwise) {
    Serial.println("Rotating CW to north...");
    digitalWrite(BottomRightFET, HIGH);
    delay(5);

    // Phase 1: Ramp up and run until slow-down sensor (with timeout)
    phaseStart = millis();
    while (!slowDownTriggered && !homingFailed) {
      if (currentPower < MaxPower) {
        currentPower++;
        analogWrite(TopLeftFET, currentPower);
        delay(rampSpeed);
      }
      if (millis() - phaseStart > HomingTimeoutMs) {
        Serial.println("ERROR: Homing timeout waiting for slow-down sensor.");
        homingFailed = true;
      }
    }

    // Ramp down to minimum
    rampDown(TopLeftFET, currentPower);

    if (!homingFailed) {
      // Phase 2: Continue at minimum until stop sensor detects north (with timeout)
      phaseStart = millis();
      while (!homeStopTriggered && !homingFailed) {
        analogWrite(TopLeftFET, slowPWM);
        if (millis() - phaseStart > HomingTimeoutMs) {
          Serial.println("ERROR: Homing timeout waiting for home stop sensor.");
          homingFailed = true;
        }
      }
    }

    analogWrite(TopLeftFET, 0);
    digitalWrite(BottomRightFET, LOW);
  }
  else {
    Serial.println("Rotating CCW to north...");
    digitalWrite(BottomLeftFET, HIGH);
    delay(5);

    // Phase 1: Ramp up and run until slow-down sensor (with timeout)
    phaseStart = millis();
    while (!slowDownTriggered && !homingFailed) {
      if (currentPower < MaxPower) {
        currentPower++;
        analogWrite(TopRightFET, currentPower);
        delay(rampSpeed);
      }
      if (millis() - phaseStart > HomingTimeoutMs) {
        Serial.println("ERROR: Homing timeout waiting for slow-down sensor.");
        homingFailed = true;
      }
    }

    // Ramp down to minimum
    rampDown(TopRightFET, currentPower);

    if (!homingFailed) {
      // Phase 2: Continue at minimum until stop sensor detects north (with timeout)
      phaseStart = millis();
      while (!homeStopTriggered && !homingFailed) {
        analogWrite(TopRightFET, slowPWM);
        if (millis() - phaseStart > HomingTimeoutMs) {
          Serial.println("ERROR: Homing timeout waiting for home stop sensor.");
          homingFailed = true;
        }
      }
    }

    analogWrite(TopRightFET, 0);
    digitalWrite(BottomLeftFET, LOW);
  }

  delay(5);

  slowDownTriggered = false;
  homeStopTriggered = false;
  encoderEnabled = true;

  if (homingFailed) {
    Serial.println("Homing FAILED. Check Hall sensors and connections.");
    Serial.println("Position is now UNKNOWN - manual calibration required.");
    return;
  }

  // At north sensor - position is 0 degrees
  savePosition(0);
  setEncoderCount(0);

  Serial.println("North (0 degrees) reached.");

  // Now move to park position (absolute, no telescope offset)
  if (parkOffset > 0) {
    Serial.print("Moving to park position: ");
    Serial.print(parkOffset);
    Serial.println(" degrees");

    delay(500);
    // Move directly without telescope offset - park is absolute dome position
    if (parkOffset <= 180) {
      int pulsesToMove = degreesToTachs(parkOffset);
      if (pulsesToMove > OvershootCompensation) pulsesToMove -= OvershootCompensation;
      moveCW(pulsesToMove);
    } else {
      int pulsesToMove = degreesToTachs(360 - parkOffset);
      if (pulsesToMove > OvershootCompensation) pulsesToMove -= OvershootCompensation;
      moveCCW(pulsesToMove);
    }

    Serial.println("Park complete.");
  }
}

void moveToAngle(int targetDegrees) {
  int currentTachs = getEncoderCount();
  int currentDegrees = tachsToDegrees(currentTachs);

  // Dome azimuth is calculated by the external bridge (theskyx_dome_bridge.py)
  // and sent directly — no telescope offset applied here.

  Serial.print("Dome target: ");
  Serial.print(targetDegrees);
  Serial.println(" deg");

  if (currentDegrees == targetDegrees) {
    Serial.println("Already at target position.");
    return;
  }

  // Calculate both possible distances in degrees
  int cwDistance;   // Distance going clockwise (increasing degrees)
  int ccwDistance;  // Distance going counter-clockwise (decreasing degrees)

  if (targetDegrees >= currentDegrees) {
    cwDistance = targetDegrees - currentDegrees;
    ccwDistance = 360 - cwDistance;
  } else {
    ccwDistance = currentDegrees - targetDegrees;
    cwDistance = 360 - ccwDistance;
  }

  Serial.print("CW distance: ");
  Serial.print(cwDistance);
  Serial.print(" deg, CCW distance: ");
  Serial.print(ccwDistance);
  Serial.println(" deg");

  // Choose shortest path distance
  int shortestDistance = (cwDistance <= ccwDistance) ? cwDistance : ccwDistance;

  if (shortestDistance < BacklashDegrees) {
    // Short move: go directly to target at slow speed, no backlash
    int pulsesToMove = degreesToTachs(shortestDistance);
    if (cwDistance <= ccwDistance) {
      Serial.println("Moving CW (short move - slow speed)...");
      moveCW(pulsesToMove, true);
    } else {
      Serial.println("Moving CCW (short move - slow speed)...");
      moveCCW(pulsesToMove, true);
    }
  } else if (cwDistance <= ccwDistance) {
    // CW move: go directly to target — final approach is already CW, no backlash needed
    int pulsesToMove = degreesToTachs(cwDistance);
    if (pulsesToMove > OvershootCompensation) pulsesToMove -= OvershootCompensation;
    Serial.println("Moving CW (shortest path)...");
    moveCW(pulsesToMove);
  } else {
    // CCW move: overshoot CCW past target, then return CW at slow speed
    // Final approach is always CW to eliminate backlash error
    int pulsesToMove = degreesToTachs(ccwDistance + BacklashDegrees);
    if (pulsesToMove > OvershootCompensation) pulsesToMove -= OvershootCompensation;
    Serial.println("Moving CCW (with backlash overshoot)...");
    moveCCW(pulsesToMove);
    // Read actual position after coast, compute exact CW return to target
    int actualTachs = getEncoderCount();
    int targetTachs = (int)(targetDegrees / DegreesPerTach);
    int returnPulses = targetTachs - actualTachs;  // positive = CW return needed
    if (returnPulses < 0) returnPulses += (int)TachsPerDomeRev;  // handle 0/300 wrap
    Serial.println("Backlash return: moving CW to target at slow speed...");
    moveCW(returnPulses, true);
  }
}

// ==================== Shutter Control Functions ====================

bool turnRemoteOn() {
  // Hold PW switch HIGH until power LED lights up (max 6 seconds)
  // Returns true if LED confirmed on, false if timeout
  Serial.println("Turning remote ON...");
  digitalWrite(PWSwitchPin, HIGH);

  unsigned long startTime = millis();
  while (millis() - startTime < 6000) {
    if (digitalRead(PowerLEDPin) == HIGH) {
      digitalWrite(PWSwitchPin, LOW);
      delay(100);  // Pause before applying open/close command
      Serial.println("Remote power ON.");
      return true;
    }
    delay(50);
  }

  // Timeout - LED never lit within 6 seconds
  digitalWrite(PWSwitchPin, LOW);
  Serial.println("ERR: Remote power failure.");
  Serial.println("Check Remote battery.");
  remoteError = true;
  return false;
}

bool turnRemoteOff() {
  // Hold PW switch HIGH until power LED goes off (max 6 seconds)
  // Returns true if LED confirmed off, false if timeout
  Serial.println("Turning remote OFF...");
  digitalWrite(PWSwitchPin, HIGH);

  unsigned long startTime = millis();
  while (millis() - startTime < 6000) {
    if (digitalRead(PowerLEDPin) == LOW) {
      digitalWrite(PWSwitchPin, LOW);
      delay(100);
      Serial.println("Remote power OFF.");
      return true;
    }
    delay(50);
  }

  // Timeout - LED never went off within 6 seconds
  digitalWrite(PWSwitchPin, LOW);
  Serial.println("ERR: Remote power off failed.");
  Serial.println("Check Remote battery.");
  remoteError = true;
  return false;
}

void openShutter() {
  // Check if already open
  if (shutterState == 1) {
    Serial.println("Shutter is already OPEN. No action taken.");
    return;
  }

  Serial.println("Starting shutter OPEN sequence...");

  // Ensure remote is on before opening
  if (digitalRead(PowerLEDPin) == LOW) {
    if (!turnRemoteOn()) return;
  } else {
    Serial.println("Remote is already ON.");
  }

  // Activate open pin for configured time (can be cancelled)
  Serial.print("Opening shutter for ");
  Serial.print(openTime);
  Serial.println(" seconds...");
  Serial.print("Terminate opening? (Y): ");
  digitalWrite(OpenPin, HIGH);

  unsigned long openStart = millis();
  bool openCancelled = false;
  while (millis() - openStart < (unsigned long)openTime * 1000UL) {
    if (Serial.available() > 0) {
      char response = Serial.read();
      if (response == 'Y' || response == 'y') {
        openCancelled = true;
        break;
      }
    }
    if (digitalRead(CWSwitch) == LOW || digitalRead(CCWSwitch) == LOW || digitalRead(HomeSwitch) == LOW) {
      openCancelled = true;
      break;
    }
    delay(50);
  }

  delay(10);  // Prevent race condition after timeout
  digitalWrite(OpenPin, LOW);
  // Carriage return overwrites the "Terminate?" prompt
  Serial.print("\r");
  if (openCancelled) {
    Serial.println("OPEN TERMINATED                 ");
    // Multiple reads to reliably detect remote power state
    bool remoteOn = false;
    for (int i = 0; i < 5; i++) {
      if (digitalRead(PowerLEDPin) == HIGH) {
        remoteOn = true;
        break;
      }
      delay(20);
    }
    if (remoteOn) {
      if (!turnRemoteOff()) return;
    } else {
      Serial.println("Remote already OFF.");
    }
    shutterState = 0;  // Unknown
    eeprom.writeByte(ShutterStateAddress, shutterState);
    return;
  }
  Serial.println("OPEN COMPLETE                   ");

  // Update and save shutter state before tension relief
  shutterState = 1;  // Open
  eeprom.writeByte(ShutterStateAddress, shutterState);

  delay(10);  // Prevent race condition before tension relief
  // Tension relief - briefly activate close to reduce tension
  Serial.print("Tension relief (");
  Serial.print(tensionRelief);
  Serial.println(" sec)...");
  digitalWrite(ClosePin, HIGH);
  delay((unsigned long)tensionRelief * 1000UL);
  digitalWrite(ClosePin, LOW);
  delay(2000);  // Wait for remote to settle after tension relief

  // Turn off the remote
  if (digitalRead(PowerLEDPin) == HIGH) {
    if (!turnRemoteOff()) return;
  } else {
    Serial.println("Remote already OFF.");
  }

  Serial.println("Shutter OPEN sequence complete.");
}

void closeShutter() {
  // Check if already closed
  if (shutterState == 2) {
    Serial.println("Shutter is already CLOSED. No action taken.");
    return;
  }

  Serial.println("Starting shutter CLOSE sequence...");

  // Ensure remote is on before closing
  if (digitalRead(PowerLEDPin) == LOW) {
    if (!turnRemoteOn()) return;
  } else {
    Serial.println("Remote is already ON.");
  }

  // Activate close pin for configured time (can be cancelled)
  Serial.print("Closing shutter for ");
  Serial.print(closeTime);
  Serial.println(" seconds...");
  Serial.print("Terminate closing? (Y): ");
  digitalWrite(ClosePin, HIGH);

  unsigned long closeStart = millis();
  bool closeCancelled = false;
  while (millis() - closeStart < (unsigned long)closeTime * 1000UL) {
    if (Serial.available() > 0) {
      char response = Serial.read();
      if (response == 'Y' || response == 'y') {
        closeCancelled = true;
        break;
      }
    }
    if (digitalRead(CWSwitch) == LOW || digitalRead(CCWSwitch) == LOW || digitalRead(HomeSwitch) == LOW) {
      closeCancelled = true;
      break;
    }
    delay(50);
  }

  delay(10);  // Prevent race condition after timeout
  digitalWrite(ClosePin, LOW);
  // Carriage return overwrites the "Terminate?" prompt
  Serial.print("\r");
  if (closeCancelled) {
    Serial.println("CLOSE TERMINATED                ");
    // Multiple reads to reliably detect remote power state
    bool remoteOn = false;
    for (int i = 0; i < 5; i++) {
      if (digitalRead(PowerLEDPin) == HIGH) {
        remoteOn = true;
        break;
      }
      delay(20);
    }
    if (remoteOn) {
      if (!turnRemoteOff()) return;
    } else {
      Serial.println("Remote already OFF.");
    }
    shutterState = 0;  // Unknown
    eeprom.writeByte(ShutterStateAddress, shutterState);
    return;
  }
  Serial.println("CLOSE COMPLETE                  ");

  // Update and save shutter state before tension relief
  shutterState = 2;  // Closed
  eeprom.writeByte(ShutterStateAddress, shutterState);

  delay(10);  // Prevent race condition before tension relief
  // Tension relief - briefly activate open to reduce tension
  Serial.print("Tension relief (");
  Serial.print(tensionRelief);
  Serial.println(" sec)...");
  digitalWrite(OpenPin, HIGH);
  delay((unsigned long)tensionRelief * 1000UL);
  digitalWrite(OpenPin, LOW);
  delay(2000);  // Wait for remote to settle after tension relief

  // Turn off the remote
  if (digitalRead(PowerLEDPin) == HIGH) {
    if (!turnRemoteOff()) return;
  } else {
    Serial.println("Remote already OFF.");
  }

  Serial.println("Shutter CLOSE sequence complete.");
}

// ==================== Tracking (V12.4) ====================
// Tracking is handled by the external bridge script (theskyx_dome_bridge.py)

// NOTE: The following tracking functions were removed:
// - isAngleInRange(), getTrackingDirection(), runTracking()
// The bridge script polls the telescope position and sends angles via serial.

// ==================== User Interface ====================

void displayCurrentPosition() {
  int tachs = getEncoderCount();
  int degrees = tachsToDegrees(tachs);

  Serial.println();
  Serial.print("Current dome position: ");
  Serial.print(degrees);
  Serial.println(" degrees");
}

void displayStatus() {
  Serial.println();
  Serial.println("============ STATUS ============");
  int currentCount = getEncoderCount();
  int currentDegrees = tachsToDegrees(currentCount);
  Serial.print("Dome position: ");
  Serial.print(currentDegrees);
  Serial.println(" degrees");

  // Tracking info
  Serial.println("Tracking: controlled by external bridge script");

  // Shutter info
  Serial.print("Shutter state: ");
  if (shutterState == 1) {
    Serial.println("OPEN");
  } else if (shutterState == 2) {
    Serial.println("CLOSED");
  } else {
    Serial.println("UNKNOWN");
  }
  Serial.print("Shutter open time: ");
  Serial.print(openTime);
  Serial.print(" sec, close time: ");
  Serial.print(closeTime);
  Serial.println(" sec");
  Serial.print("Tension relief time: ");
  Serial.print(tensionRelief);
  Serial.println(" sec");
  Serial.print("Remote power: ");
  Serial.println(digitalRead(PowerLEDPin) == HIGH ? "ON" : "OFF");

  // Park offset
  Serial.print("Park offset: ");
  Serial.print(parkOffset);
  Serial.println(" deg");

  Serial.println("================================");
}

void printHelp() {
  Serial.println();
  Serial.println("============ COMMANDS ============");
  Serial.println("0-359    Move to angle (degrees)");
  Serial.println("O        Open shutter");
  Serial.println("C        Close shutter");
  Serial.println("G        Go home (park)");
  Serial.println("S        Status display");
  Serial.println("AO<val>  Shutter Open time (1-120 sec)");
  Serial.println("AC<val>  Shutter Close time (1-120 sec)");
  Serial.println("AT<val>  Shutter Tension relief (1-10 sec)");
  Serial.println("P<val>   Park offset (0-359 deg)");
  Serial.println("W<val>   sloW-down PWM (50-150)");
  Serial.println("D<val>   ramp Delay/speed (1-20 ms)");
  Serial.println("Q        Query all configuration");
  Serial.println("I        Interactive config menu");
  Serial.println("?        Show this help");
  Serial.println("==================================");
  Serial.println("Manual: CW/CCW buttons");
  Serial.println("Home: Hold home switch 2 sec");
  Serial.println();
}

void processSerialCommand() {
  char cmd = Serial.read();
  Serial.print("The entered command = ");
  Serial.println(cmd);

  // Handle two-character commands first (AO, AC, AT)
  if (cmd == 'A' || cmd == 'a') {
    if (Serial.available() > 0) {
      char subCmd = Serial.read();
      int value = Serial.parseInt();

      if (subCmd == 'O' || subCmd == 'o') {
        // Shutter Open time
        if (value >= 1 && value <= 120) {
          openTime = value;
          saveConfiguration();
          Serial.print("Shutter open time set to ");
          Serial.print(openTime);
          Serial.println(" sec (saved)");
        } else {
          Serial.println("Error: Shutter open time must be 1-120 sec");
        }
      } else if (subCmd == 'C' || subCmd == 'c') {
        // Shutter Close time
        if (value >= 1 && value <= 120) {
          closeTime = value;
          saveConfiguration();
          Serial.print("Shutter close time set to ");
          Serial.print(closeTime);
          Serial.println(" sec (saved)");
        } else {
          Serial.println("Error: Shutter close time must be 1-120 sec");
        }
      } else if (subCmd == 'T' || subCmd == 't') {
        // Shutter Tension relief time
        if (value >= 1 && value <= 10) {
          tensionRelief = value;
          saveConfiguration();
          Serial.print("Tension relief time set to ");
          Serial.print(tensionRelief);
          Serial.println(" sec (saved)");
        } else {
          Serial.println("Error: Tension relief time must be 1-10 sec");
        }
      } else {
        Serial.println("Use AO# open, AC# close, AT# tension relief");
      }
    } else {
      Serial.println("Use AO# open, AC# close, AT# tension relief");
    }
    while (Serial.available() > 0) Serial.read();
    return;
  }

  int value = Serial.parseInt();

  switch (cmd) {
    case 'O':  // Open shutter
    case 'o':
      openShutter();
      break;

    case 'C':  // Close shutter
    case 'c':
      closeShutter();
      break;

    case 'G':  // Go home (park)
    case 'g':
      goHome();
      break;

    case 'S':  // Status display
    case 's':
      displayStatus();
      break;

    case 'P':  // Park offset
    case 'p':
      if (value >= 0 && value <= 359) {
        parkOffset = value;
        saveConfiguration();
        Serial.print("Park offset set to ");
        Serial.print(parkOffset);
        Serial.println(" deg (saved)");
      } else {
        Serial.println("Error: Park offset must be 0-359 deg");
      }
      break;

    case 'W':  // Slow-down PWM
    case 'w':
      if (value >= 50 && value <= 150) {
        slowPWM = value;
        saveConfiguration();
        Serial.print("Slow-down PWM set to ");
        Serial.print(slowPWM);
        Serial.println(" (saved)");
      } else {
        Serial.println("Error: Slow-down PWM must be 50-150");
      }
      break;

    case 'D':  // Ramp Delay/speed
    case 'd':
      if (value >= 1 && value <= 20) {
        rampSpeed = value;
        saveConfiguration();
        Serial.print("Ramp speed set to ");
        Serial.print(rampSpeed);
        Serial.println(" ms (saved)");
      } else {
        Serial.println("Error: Ramp speed must be 1-20 ms");
      }
      break;

    case 'Q':  // Query configuration
    case 'q':
      displayConfiguration();
      break;

    case 'I':  // Interactive config menu
    case 'i':
      configurationMenu();
      break;

    case 'H':  // Help
    case 'h':
    case '?':
      printHelp();
      break;

    default:
      Serial.println("Unknown command. Enter ? for help.");
      break;
  }

  // Clear any remaining characters
  while (Serial.available() > 0) Serial.read();
}

int getTargetAngle() {
  Serial.println();
  Serial.println("***************************************");
  Serial.print("Dome Controller ");
  Serial.print(CodeVersion);
  Serial.print(" (");
  Serial.print(CodeDate);
  Serial.println(")");
  Serial.println("Enter angle (0-359), command, or ? for help:");
  Serial.println();

  // Wait for serial input or button press
  while (Serial.available() == 0) {
    if (digitalRead(CWSwitch) == LOW) return -1;   // Manual CW
    if (digitalRead(CCWSwitch) == LOW) return -2;  // Manual CCW
    if (checkHomeSwitch()) return -3;              // Home (requires 2-second hold)
  }

  // Check first character for command
  char peek = Serial.peek();

  // If it's a letter or ?, process as command
  if ((peek >= 'A' && peek <= 'Z') || (peek >= 'a' && peek <= 'z') || peek == '?') {
    processSerialCommand();
    return -6;  // Command processed, no movement
  }

  // Otherwise parse as angle
  int angle = Serial.parseInt();

  // Clear buffer
  while (Serial.available() > 0) Serial.read();

  Serial.print("The entered command = ");
  Serial.println(angle);

  if (angle < 0 || angle > 359) {
    Serial.println("Invalid angle. Enter 0-359 or ? for help.");
    return -4;  // Invalid
  }

  Serial.print("Target angle: ");
  Serial.print(angle);
  Serial.println(" degrees");

  return angle;
}

// ==================== Setup ====================

void setup() {
  Serial.begin(9600);
  delay(2000);

  // Initialize EEPROM and check connection
  eeprom.begin();
  delay(10);

  if (!checkEepromConnection()) {
    Serial.println("*********************************************");
    Serial.println("*** WARNING: EEPROM NOT DETECTED!         ***");
    Serial.println("*** Position will NOT be saved.           ***");
    Serial.println("*** Configuration will use defaults.      ***");
    Serial.println("*** Check I2C wiring before operation.    ***");
    Serial.println("*********************************************");
    Serial.println();
  }

  // Configure motor output pins
  pinMode(TopLeftFET, OUTPUT);
  pinMode(TopRightFET, OUTPUT);
  pinMode(BottomLeftFET, OUTPUT);
  pinMode(BottomRightFET, OUTPUT);

  // Configure control input pins
  pinMode(CWSwitch, INPUT_PULLUP);
  pinMode(CCWSwitch, INPUT_PULLUP);
  pinMode(HomeSwitch, INPUT_PULLUP);
  pinMode(SlowHallPin, INPUT);
  pinMode(StopHallPin, INPUT);

  // Configure shutter control pins
  pinMode(PowerLEDPin, INPUT);       // Read remote power status
  pinMode(OpenPin, OUTPUT);          // Shutter open signal
  pinMode(PWSwitchPin, OUTPUT);      // Remote power toggle
  pinMode(ClosePin, OUTPUT);         // Shutter close signal
  digitalWrite(OpenPin, LOW);        // Initialize outputs LOW
  digitalWrite(PWSwitchPin, LOW);    // PW switch is active HIGH, so start LOW
  digitalWrite(ClosePin, LOW);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(EncoderPin), encoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(SlowHallPin), slowDownISR, RISING);
  attachInterrupt(digitalPinToInterrupt(StopHallPin), homeStopISR, RISING);

  // Ensure motor is stopped
  stopMotor();

  // Load saved position
  setEncoderCount(loadPosition());

  // Load configuration from EEPROM
  loadConfiguration();
  currentPower = slowPWM;

  Serial.println();
  Serial.print("Dome Controller ");
  Serial.print(CodeVersion);
  Serial.print(" (");
  Serial.print(CodeDate);
  Serial.println(")");
  Serial.println("NOTE: Set Serial Monitor to 'No Line Ending'");
  Serial.println("Enter ? for help");
  displayConfiguration();
}

// ==================== Main Loop ====================

void loop() {
  // Periodically check EEPROM connection (allows recovery if reconnected)
  static unsigned long lastEepromCheck = 0;
  if (!eepromAvailable && (millis() - lastEepromCheck > 10000)) {
    lastEepromCheck = millis();
    if (checkEepromConnection()) {
      Serial.println("EEPROM connection restored!");
    }
  }

  // Periodically save position if changed (safety net for any missed saves)
  savePositionIfChanged();

  if (remoteError) {
    remoteError = false;
  } else {
    displayCurrentPosition();
  }

  int target = getTargetAngle();

  switch (target) {
    case -1:  // Manual CW
      moveCW(0);
      break;

    case -2:  // Manual CCW
      moveCCW(0);
      break;

    case -3:  // Home
      goHome();
      break;

    case -4:  // Invalid input
      break;

    case -6:  // Command processed
      break;

    default:  // Valid angle (0-359)
      if (target >= 0 && target <= 359) {
        moveToAngle(target);
      }
      break;
  }

  delay(100);
}
