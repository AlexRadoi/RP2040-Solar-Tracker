#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"
#include <SolarPosition.h>
#include <AccelStepper.h>

// --- Hardware Pin Allocations ---
const int chipSelect = 1;      // MicroSD Card CS pin
const int pinPWM = 8;          // PWM output for SEPIC MOSFET gate driver (GP8)
const int pinPot = 29;         // ADC input for Target Voltage reference potentiometer
const int pinFeedback = 28;    // ADC input for SEPIC Output Voltage feedback
const int pinVin = 27;         // ADC input for Solar Panel Input Voltage monitoring

// Stepper Motor Driver Interfaces (A4988)
const int step1 = 6;  const int dir1 = 7;   // Primary Axis Stepper (Motor 1)
const int step2 = 11; const int dir2 = 12;  // Secondary Axis Mirror Stepper (Motor 2)

// --- Physical and Geometric Constants ---
const float LATITUDE = 44.4268;         // Bucharest coordinates
const float LONGITUDE = 26.1025;
const float stepsPerDegree = 8.88888;  // Mechanical resolution (Motor steps per kinematic degree)
const float referenceVoltage = 3.3;    // RP2040 ADC reference voltage (VREF)

// Hardware travel boundaries (-45° East to +45° West)
const long stepsMin = (long)(-45.0 * stepsPerDegree); // East Park Position (~ -400 steps)
const long stepsMax = (long)(45.0 * stepsPerDegree);  // West Park Position (~ +400 steps)

// --- Shared Inter-Core Volatile Variables (IPC) ---
volatile long globalTargetSteps = 0;   // Calculated target position from Core 1 to Core 0
volatile float sharedInV = 0;          // Solar voltage acquired by Core 0, logged by Core 1
volatile float sharedOutV = 0;         // SEPIC voltage acquired by Core 0, logged by Core 1
volatile float sharedAzimuth = 0;      // Astronomical Azimuth for logging and telemetry
volatile float sharedElevation = 0;     // Astronomical Elevation for logging and telemetry
volatile float pwmVal = 0;             // Current Duty Cycle value for the SEPIC loop

// --- Chronological Tracking Profile Variables ---
int dynamicNoonMinute = 615;           // Fallback Solar Noon value (10:15 UTC in minutes)
int lastScanDay = -1;                  // Tracks calendar day changes to trigger the daily solar scan

// Object Instantiations
RTC_DS3231 rtc;
SolarPosition bucharestSolar(LATITUDE, LONGITUDE);
AccelStepper stepper1(AccelStepper::DRIVER, step1, dir1);
AccelStepper stepper2(AccelStepper::DRIVER, step2, dir2);

// Automatic Daylight Saving Time (DST) Logic for Romania
int getUtcOffset(DateTime dt) {
  int m = dt.month(); int d = dt.day(); int dow = dt.dayOfTheWeek();
  if (m < 3 || m > 10) return 2;  // Winter Time: UTC+2
  if (m > 3 && m < 10) return 3;  // Summer Time (DST): UTC+3
  if (m == 3) return (d - dow >= 25) ? 3 : 2;   // Last Sunday of March transition
  if (m == 10) return (d - dow >= 25) ? 2 : 3;  // Last Sunday of October transition
  return 2;
}

// ============================================================================
// CORE 0: REAL-TIME EXECUTION (SEPIC CONVERTER LOOP & STEPPER ACCELERATION)
// ============================================================================
void setup() {
  analogReadResolution(12);       // Configure ADC to 12-bit resolution (0 - 4095)
  pinMode(pinPWM, OUTPUT);
  analogWriteFreq(200000);        // 200kHz switching frequency for SEPIC inductor efficiency
  analogWriteRange(1000);         // 0 - 1000 dynamic range for fine duty cycle adjustments

  // Configure movement profiles for both stepper actuators (adjusted for high microstepping)
  stepper1.setMaxSpeed(4000);      
  stepper1.setAcceleration(1000);
  stepper2.setMaxSpeed(4000);
  stepper2.setAcceleration(1000);

  // Perform initial 360-degree rotation at boot
  long fullSweepSteps = (long)(360.0 * stepsPerDegree);
  //stepper1.move(fullSweepSteps);
  //stepper2.move(fullSweepSteps);
  
  // Block and execute the movement until both steppers finish their 360-degree rotation
  while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0) {
    stepper1.run();
    stepper2.run();
  }
  
  // Reset current positions to 0 to establish the new origin point for tracking
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
}

void loop() {
  // 1. SEPIC PWM Feedback Loop (Priority 1 - Runs strictly every 5ms)
  static unsigned long lastSepicUpdate = 0;
  if (millis() - lastSepicUpdate >= 5) { 
    lastSepicUpdate = millis();
    int targetV = analogRead(pinPot);      // Desired output voltage reference
    int actualV = analogRead(pinFeedback); // Real-time feedback from output rail
    int inVRaw = analogRead(pinVin);       // Input voltage from solar panel
    
    // Safety check: shut down PWM if there is no input voltage
    // Prevents duty cycle from ramping to 100% and burning the converter
    if(actualV  > 3800){ 
        pwmVal = 0;
      }
    else if (inVRaw < 50) { // ~40mV threshold on the ADC pin
      pwmVal = 0;
    } else {
      // Incremental integral control action for voltage stabilization
      if (abs(targetV - actualV) > 5) {
        if (targetV > actualV) pwmVal += 1.0; 
        else pwmVal -= 1.0;
      }
    }
    pwmVal = constrain(pwmVal, 0, 1000);    // Safety clamp
    analogWrite(pinPWM, (int)pwmVal);
  }

  // 2. Stepper Motion Execution (Priority 2 - Non-blocking tracking update)
  stepper1.moveTo((long)(globalTargetSteps));
  stepper2.moveTo((long)(globalTargetSteps));
  stepper1.run();
  stepper2.run();

  // Detect state changes to print movement events
  static bool wasMoving = false;
  bool isMoving = (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0);
  if (isMoving != wasMoving) {
    if (isMoving) {
      Serial.println("\n>> ACTUATORS: Motion started towards target...");
    } else {
      Serial.println(">> ACTUATORS: Target position reached.\n");
    }
    wasMoving = isMoving;
  }
  

  // 3. Sensor Scaling (Convert raw ADC counts to physical voltages)
  sharedInV = (analogRead(pinVin) / 4095.0) * referenceVoltage;
  sharedOutV = (analogRead(pinFeedback) / 4095.0) * referenceVoltage;
}

// ============================================================================
// CORE 1: RESOURCE MANAGEMENT (ASTRONOMY, TIME MAPPING & DATA SD LOGGING)
// ============================================================================
void setup1() {
  Serial.begin(115200);
  delay(2000); // Guard window allowing Core 0 loops and power rails to stabilize
  
  Wire.setSDA(4); Wire.setSCL(5); Wire.begin(); // Map I2C lines for RTC communication
  
  // Explicit Hardware SPI0 Mapping for RP2040-Zero Form Factor
  SPI.setRX(0);   // MISO = GP0
  SPI.setTX(3);   // MOSI = GP3
  SPI.setSCK(2);  // SCK  = GP2
  
  if (!rtc.begin()) {
    Serial.println("Error: RTC hardware module not found!");
  } else {
    Serial.println("RTC module initialized successfully.");
    
    // Check if the RTC lost power and needs to be set, or force update it
    if (rtc.lostPower()) {
      Serial.println("RTC lost power, setting time to compile time!");
      // This sets the RTC to the date & time this sketch was compiled
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    // UNCOMMENT THE LINE BELOW TO FORCE UPDATE THE TIME ON EVERY REBOOT:
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  
  // SD Card File System Initialization
  if (SD.begin(chipSelect)) {
    Serial.println("SD card initialization successful.");
    if (!SD.exists("log.csv")) {
      File f = SD.open("log.csv", FILE_WRITE);
      if (f) {
        // Updated structure containing full solar telemetry vectors
        f.println("Time,SolarIn,SepicOut,Azimuth,Elevation,Steps");
        f.close();
        Serial.println("Created log.csv file header.");
      }
    }
  } else {
    Serial.println("Critical Error: SD card initialization failed!");
  }
}

void loop1() {
  DateTime now = rtc.now(); // Poll current local time from the hardware clock

  // 1. DAILY SOLAR SCAN (Executes once per 24 hours at midnight)
  // Computes the exact UTC minute where the sun crosses true South (Azimuth = 180 degrees)
  if (now.day() != lastScanDay) {
    float minAzDiff = 360.0;
    int foundNoonMinute = 615; // Reset to default safe fallback

    // Virtual high-speed tracking loop covering 08:00 UTC (480m) to 14:00 UTC (840m)
    for (int minutes = 480; minutes <= 840; minutes++) {
      DateTime testTime(now.year(), now.month(), now.day(), 0, 0, 0);
      long testUnix = testTime.unixtime() + (minutes * 60);
      SolarPosition_t solTest = bucharestSolar.getSolarPosition(testUnix);
      
      float azDiff = abs(solTest.azimuth - 180.0);
      if (azDiff < minAzDiff) {
        minAzDiff = azDiff;
        foundNoonMinute = minutes; // Store computed astronomical solar noon minute
      }
    }
    dynamicNoonMinute = foundNoonMinute;
    lastScanDay = now.day();
    
    Serial.print("Daily profile updated. Dynamic Solar Noon calculated at UTC minute: ");
    Serial.println(dynamicNoonMinute);
  }

  // 2. REAL-TIME SOLAR DATA ACQUISITION
  // Translate local RTC time into absolute UTC minutes elapsed from midnight
  long utcUnix = now.unixtime() - (getUtcOffset(now) * 3600);
  SolarPosition_t currentSol = bucharestSolar.getSolarPosition(utcUnix);
  sharedAzimuth = currentSol.azimuth;
  sharedElevation = currentSol.elevation;

  // 3. ASTRONOMICAL MAPPING & NIGHTTIME REVERSAL LOGIC
  // The sun crosses True South at exactly 180.0 degrees Azimuth.
  // Hardware limits are -45° East (135° Azimuth) to +45° West (225° Azimuth).
  if (sharedElevation > 0) { 
    // Daytime Operation: Sun is above the horizon
    if (sharedAzimuth < 135.0) {
      // Early Morning: Sun is further East than the hardware limits. Park at East limit.
      globalTargetSteps = stepsMin;
    } 
    else if (sharedAzimuth > 225.0) {
      // Late Afternoon: Sun is further West than the hardware limits. Park at West limit.
      globalTargetSteps = stepsMax;
    }  
    else {
      // Active Tracking Window: Map astronomical Azimuth directly to physical microsteps.
      // This provides perfect non-linear tracking compensating for the sun's actual arc.
      globalTargetSteps = map(sharedAzimuth, 135.0, 225.0, stepsMin, stepsMax);
    }
  } else {
    // Night Routine / Reversal Phase: Sun is below the horizon
    // Automatically returns the array back to the maximum East position (-45°)
    // This pre-positions the mechanics safely for the next day's dawn cycle.
    globalTargetSteps = stepsMin; 
  }

  // 5. DATA LOGGING ROUTINE (SD Card Flash Write)
  File dataFile = SD.open("log.csv", FILE_WRITE);
  
  if (!dataFile) {
    // Automated hot-plugging recovery routine if SD card drops offline
    SD.begin(chipSelect);
    dataFile = SD.open("log.csv", FILE_WRITE);
  }
  
  if (dataFile) {
    char buf[] = "hh:mm:ss";
    dataFile.print(now.toString(buf)); dataFile.print(",");
    dataFile.print(sharedInV, 2); dataFile.print(",");
    dataFile.print(sharedOutV, 2); dataFile.print(",");
    dataFile.print(sharedAzimuth, 2); dataFile.print(",");     // Log absolute real-time Azimuth
    dataFile.print(sharedElevation, 2); dataFile.print(",");   // Log absolute real-time Elevation
    dataFile.println(globalTargetSteps);                      // Log actual ordered stepper position
    dataFile.close();
    Serial.println("Telemetry batch logged to SD card.");
  } else {
    Serial.println("Error: Failed to write data to log.csv");
  }

  // 6. SERIAL TELEMETRY MONITORING (For terminal interface during bench testing)
  Serial.println("\n=== DUAL-CORE SYSTEM TELEMETRY ===");
  char timeBuf[] = "YYYY-MM-DD hh:mm:ss";
  Serial.print("Local System Time: "); Serial.println(now.toString(timeBuf));
  Serial.print("Solar Array Voltage (PV In): "); Serial.print(sharedInV, 2); Serial.println(" V");
  Serial.print("SEPIC Rail Output (DC Out): "); Serial.print(sharedOutV, 2); Serial.println(" V");
  Serial.print("Astronomical Vector -> Azimuth: "); Serial.print(sharedAzimuth, 2);
  Serial.print(" deg | Elevation: "); Serial.print(sharedElevation, 2); Serial.println(" deg");
  Serial.print("Actuator Control   -> Target Steps: "); Serial.println(globalTargetSteps);
  Serial.println("==================================");

  delay(1000); // Fixed management loop interval (1-second updates)
}
