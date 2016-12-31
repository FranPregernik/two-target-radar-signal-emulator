#include "Arduino.h"
#include "PP.h"

//#define DEBUG
//#define SIM

PP myPP; /* Instance our PPM class library	*/

uint8_t pinLed = 13;

uint8_t pinNorth = 12;

uint8_t pinTargetOneTrigger = 2;
uint8_t pinTargetTwoTrigger = 4;

uint8_t pinProgramSelectOne = 6;
uint8_t pinProgramSelectTwo = 8;

uint8_t pingTargetOneOutput = 22;
uint8_t pingTargetTwoOutput = 3;

uint8_t programSelect = 0;

/**
 * Calibration vars
 */
elapsedMicros calibrationTimeUs;
elapsedMillis calibrationTimeMs;

// flag to indicate measuring is in progress
volatile bool measuring = false;

// used for calculating averages
volatile long calTotal = 0;
volatile long calSampleCount = 0;

// Measured time between radar pulses.
// Efectively max distance the radar can measure in [us]
volatile long maxDistanceTimeUs = 0;

// Measured time between North signal readings
volatile long northRotTimeMs = 0;

/**
 * An ISR that fires on Rp1 and measures the time between signals
 */
void calibrateMaxTargetDistanceIsr() {
  if (measuring) {
    calSampleCount++;
    calTotal += calibrationTimeUs;
  } else {
    measuring = true;
  }
  calibrationTimeUs = 0;
}

/**
 * An ISR that fires on north trigger and measures the time between signals
 */
void calibrateNorthIsr() {
  if (measuring) {
    calSampleCount++;
    calTotal += calibrationTimeMs;
  } else {
    measuring = true;
  }
  calibrationTimeMs = 0;
}

// Flag to indicate if the scenario calculation has complated
// Calculation is started at North signal (ISR)
volatile boolean calculationComplete = false;

// flags to indicate that the target ISRs are attached
volatile boolean t0_Attached = false;
volatile boolean t1_Attached = false;

// Offset from North to center of target
// relative to rotation speed
volatile int32_t t0_angleTimeMs = 0;
volatile int32_t t1_angleTimeMs = 0;

// indicates how long the targets appear active (azimuth)
// in [ms] relative to rotation speed and North signal
volatile uint32_t t0_angleWidthTimeMs = 0;
volatile uint32_t t1_angleWidthTimeMs = 0;

// Calculated start of target (azimuth) based on
// offset (angleTimeMs) and target width (angleWidthTimeMs)
volatile uint32_t t0_startMs;
volatile uint32_t t1_startMs;

// Calculated end of target (azimuth) based on
// offset (angleTimeMs) and target width (angleWidthTimeMs)
volatile uint32_t t0_endMs;
volatile uint32_t t1_endMs;

// calculated based on target speed for each scenario
// how much the target moved with each radar rotation
volatile uint32_t t0_radialStepUs = 0;
volatile uint32_t t1_radialStepUs = 0;

// calculated based on target speed (tx_radialStepUs), rotation count
// bounded by minimum and maximum distance.
// Represents the time from the start of radar signal ping to when the radar
// receives the response
volatile uint32_t t0_timeToReceiverUs = 1;
volatile uint32_t t1_timeToReceiverUs = 1;

// scenario state - how many rotations the radar has made
volatile uint32_t fullRotationCount = 0;

elapsedMillis elpsedSinceNorthMs;
elapsedMicros elpsedSinceTargetUs;

/**
 * Moving targets spread across all azimuths - DEMO
 * target one departing at the speed of 250 [m/s]
 * target two aproaching at the speed of 250 [m/s]
 */
void north0Isr() {
  calculationComplete = false;
  fullRotationCount++;
  elpsedSinceNorthMs = 0;

  // just increase (departing) one step at a time and modulo it with the max distance
  t0_timeToReceiverUs += t0_radialStepUs;
  t0_timeToReceiverUs %= maxDistanceTimeUs;
  t0_timeToReceiverUs = max(5, t0_timeToReceiverUs);

  // decrease with wrap around
  if (t1_radialStepUs > t1_timeToReceiverUs) {
    t1_timeToReceiverUs = maxDistanceTimeUs - (t1_radialStepUs - t1_timeToReceiverUs);
  } else {
    t1_timeToReceiverUs -= t1_radialStepUs;
  }
  t1_timeToReceiverUs = max(5, t1_timeToReceiverUs);

#ifdef DEBUG
  Serial.print("t0_timeToReceiverUs [us]: ");
  Serial.println(t0_timeToReceiverUs);
  Serial.print("t1_timeToReceiverUs [us]: ");
  Serial.println(t1_timeToReceiverUs);
#endif

  // signal software north (calculation completed)
  digitalWriteFast(pinLed, !digitalReadFast(pinLed));

  calculationComplete = true;
}

/**
 * Moving targets on same azimuth
 * target one (cloud) departing at the speed of 20 [m/s]
 * target two (helicopter) aproaching at the speed of 70 [m/s]
 */
void north1Isr() {
  calculationComplete = false;
  fullRotationCount++;
  elpsedSinceNorthMs = 0;

  // just increase (departing) one step at a time and modulo it with the max distance
  t0_timeToReceiverUs += t0_radialStepUs;
  t0_timeToReceiverUs %= maxDistanceTimeUs;
  t0_timeToReceiverUs = max(5, t0_timeToReceiverUs);

  // decrease with wrap around
  if (t1_radialStepUs > t1_timeToReceiverUs) {
    t1_timeToReceiverUs = maxDistanceTimeUs - (t1_radialStepUs - t1_timeToReceiverUs);
  } else {
    t1_timeToReceiverUs -= t1_radialStepUs;
  }
  t1_timeToReceiverUs = max(5, t1_timeToReceiverUs);

#ifdef DEBUG
  Serial.print("t0_timeToReceiverUs [us]: ");
  Serial.println(t0_timeToReceiverUs);
  Serial.print("t1_timeToReceiverUs [us]: ");
  Serial.println(t1_timeToReceiverUs);
#endif

  // signal software north (calculation completed)
  digitalWriteFast(pinLed, !digitalReadFast(pinLed));

  calculationComplete = true;
}

/**
 * target one at azimuth A departing at the speed of 100 [m/s]
 * target two at azimuth B approacing at the speed of 200 [m/s]
 * |A - B| = 10 Deg
 */
void north2Isr() {
  calculationComplete = false;
  fullRotationCount++;
  elpsedSinceNorthMs = 0;

  // just increase (departing) one step at a time and modulo it with the max distance
  t0_timeToReceiverUs += t0_radialStepUs;
  t0_timeToReceiverUs %= maxDistanceTimeUs;
  t0_timeToReceiverUs = max(5, t0_timeToReceiverUs);

  // decrease with wrap around
  if (t1_radialStepUs > t1_timeToReceiverUs) {
    t1_timeToReceiverUs = maxDistanceTimeUs - (t1_radialStepUs - t1_timeToReceiverUs);
  } else {
    t1_timeToReceiverUs -= t1_radialStepUs;
  }
  t1_timeToReceiverUs = max(5, t1_timeToReceiverUs);

#ifdef DEBUG
  Serial.print("t0_timeToReceiverUs [us]: ");
  Serial.println(t0_timeToReceiverUs);
  Serial.print("t1_timeToReceiverUs [us]: ");
  Serial.println(t1_timeToReceiverUs);
#endif

  // signal software north (calculation completed)
  digitalWriteFast(pinLed, !digitalReadFast(pinLed));

  calculationComplete = true;
}

/**
 * target one at azimuth A departing at the speed of 200 [m/s]
 * target two at azimuth B departing at the speed of 200 [m/s]
 * |A - B| = 5 Deg
 */
void north3Isr() {
  calculationComplete = false;
  fullRotationCount++;
  elpsedSinceNorthMs = 0;

  // just increase (departing) one step at a time and modulo it with the max distance
  t0_timeToReceiverUs += t0_radialStepUs;
  t0_timeToReceiverUs %= maxDistanceTimeUs;
  t0_timeToReceiverUs = max(5, t0_timeToReceiverUs);

  // decrease with wrap around
  t1_timeToReceiverUs += t1_radialStepUs;
  t1_timeToReceiverUs %= maxDistanceTimeUs;
  t1_timeToReceiverUs = max(5, t1_timeToReceiverUs);

#ifdef DEBUG
  Serial.print("t0_timeToReceiverUs [us]: ");
  Serial.println(t0_timeToReceiverUs);
  Serial.print("t1_timeToReceiverUs [us]: ");
  Serial.println(t1_timeToReceiverUs);
#endif

  // signal software north (calculation completed)
  digitalWriteFast(pinLed, !digitalReadFast(pinLed));

  calculationComplete = true;
}

/**
 * Fires a pulse at the specified time - target 1
 */
void targetOneIsr() { myPP.pulse(22, t0_timeToReceiverUs); }

/**
 * Fires a pulse at the specified time - target 2
 */
void targetTwoIsr() { myPP.pulse(3, t1_timeToReceiverUs); }

/**
 * Setup procedure that performs nort and max distance measurement
 * and reads the selected program
 */
void setup() {
#ifdef DEBUG
  // init serial
  Serial.begin(9600);  // USB is always 12 Mbit/sec
  delay(4000);
  Serial.println("Startup");
#endif

  // setup pins
  pinMode(pinLed, OUTPUT); /* LED for North */
  pinMode(pinProgramSelectOne, INPUT);
  pinMode(pinProgramSelectTwo, INPUT);
  pinMode(pinTargetOneTrigger, INPUT);
  pinMode(pinTargetTwoTrigger, INPUT);
  pinMode(pinNorth, INPUT);
  pinMode(pingTargetOneOutput, OUTPUT);
  pinMode(pingTargetTwoOutput, OUTPUT);

  /* Instance pin 22 as FALLING edge output pin		*/
  myPP.Output(2, 22, 3);

  // Find the average max full rotation time
  // i.e. time between subsequent north signals
  calTotal = 0;
  calSampleCount = 0;
  measuring = false;
#ifndef SIM
#ifdef DEBUG
  Serial.println("Measuring avg TTnorth ...");
#endif
  attachInterrupt(pinNorth, calibrateNorthIsr, FALLING);
  while (calSampleCount < 2) {
    // noop
  }
  detachInterrupt(pinNorth);
  northRotTimeMs = calTotal / calSampleCount;
#else
  northRotTimeMs = 12480;
#endif

#ifdef DEBUG
  Serial.print("Time between north [ms]: ");
  Serial.println(northRotTimeMs);
#endif

  // Find the average max distance time
  // i.e. time between subsequent Rp1
  calTotal = 0;
  calSampleCount = 0;
  measuring = false;
#ifndef SIM
#ifdef DEBUG
  Serial.println("Measuring avg TTtarget ...");
#endif
  attachInterrupt(pinTargetOneTrigger, calibrateMaxTargetDistanceIsr, FALLING);
  while (calSampleCount < 10) {
  }
  detachInterrupt(pinTargetOneTrigger);
  maxDistanceTimeUs = calTotal / calSampleCount;
#else
  maxDistanceTimeUs = 3540;
#endif

#ifdef DEBUG
  Serial.print("Time between Rpx [us]: ");
  Serial.println(maxDistanceTimeUs);
#endif

  // fetch program choice
  uint8_t ps1 = digitalRead(pinProgramSelectOne);
  uint8_t ps2 = digitalRead(pinProgramSelectTwo);
  programSelect = 2 * ps1 + ps2;
#ifdef DEBUG
  Serial.print("Program selected: ");
  Serial.println(programSelect);
#endif

  // setup north pin interrupt
  switch (programSelect) {
    case 0:
      /**
       * Moving targets spread across all azimuths - DEMO
       * target one departing at the speed of 250 [m/s]
       * target two aproaching at the speed of 250 [m/s]
       */
      t0_timeToReceiverUs = 5;
      t0_angleTimeMs = 40;
      t0_angleWidthTimeMs = 75;
      // t0_radialStepUs = (2 * 250.0 /* m/s */ * (northRotTimeMs / 1000.0) / (300000.0 /* m/s */) * 1e6);
      t0_radialStepUs = (250.0 * northRotTimeMs) / (150.0 * 1000);
      t1_timeToReceiverUs = maxDistanceTimeUs;
      t1_angleTimeMs = 40;
      t1_angleWidthTimeMs = 75;
      // t1_radialStepUs = (2 * 250.0 /* m/s */ * (northRotTimeMs / 1000.0) / (300000.0 /* m/s */) * 1e6);
      t1_radialStepUs = (250.0 * northRotTimeMs) / (150.0 * 1000);
      attachInterrupt(pinNorth, north0Isr, FALLING);
      break;
    case 1:
      /**
       * Moving targets on same azimuth
       * target one (cloud) departing at the speed of 20 [m/s]
       * target two (helicopter) aproaching at the speed of 120 [m/s]
       */
      t0_timeToReceiverUs = 5;
      t0_angleTimeMs = 40;
      t0_angleWidthTimeMs = 15 * (northRotTimeMs / 360);  // the cloud is 15deg wide
      // t0_radialStepUs = (2 * 20.0 /* m/s */ * (northRotTimeMs / 1000.0) / (300000.0 /* m/s */) * 1e6);
      t0_radialStepUs = (20.0 * northRotTimeMs) / (150.0 * 1000);
      t1_timeToReceiverUs = maxDistanceTimeUs;
      t1_angleTimeMs = 40;
      t1_angleWidthTimeMs = 75;
      // t1_radialStepUs = (2 * 70.0 /* m/s */ * (northRotTimeMs / 1000.0) / (300000.0 /* m/s */) * 1e6);
      t1_radialStepUs = (120.0 * northRotTimeMs) / (150.0 * 1000);

      // calculate real start and end of targets
      t0_startMs = t0_angleTimeMs;
      t0_endMs = t0_startMs + t0_angleWidthTimeMs;
      t1_startMs = (t0_startMs + t0_endMs) / 2 - t1_angleWidthTimeMs / 2;
      t1_endMs = t1_startMs + t1_angleWidthTimeMs;

      attachInterrupt(pinNorth, north1Isr, FALLING);
      break;
    case 2:
      /**
       * target one at azimuth A departing at the speed of 100 [m/s]
       * target two at azimuth B aproaching at the speed of 200 [m/s]
       * |A - B| = 10 Deg
       */
      t0_timeToReceiverUs = 5;
      t0_angleTimeMs = 40;
      t0_angleWidthTimeMs = 75;
      // t0_radialStepUs = (2 * 100.0 /* m/s */ * (northRotTimeMs / 1000.0) / (300000.0 /* m/s */) * 1e6);
      t0_radialStepUs = (100.0 * northRotTimeMs) / (150.0 * 1000);
      t1_timeToReceiverUs = maxDistanceTimeUs;
      t1_angleTimeMs = 40 + 10 * (northRotTimeMs / 360);
      t1_angleWidthTimeMs = 75;
      // t1_radialStepUs = (2 * 200.0 /* m/s */ * (northRotTimeMs / 1000.0) / (300000.0 /* m/s */) * 1e6);
      t1_radialStepUs = (200.0 * northRotTimeMs) / (150.0 * 1000);

      // calculate real start and end of targets
      t0_startMs = t0_angleTimeMs;
      t0_endMs = t0_startMs + t0_angleWidthTimeMs;
      t1_startMs = t1_angleTimeMs;
      t1_endMs = t1_startMs + t1_angleWidthTimeMs;

      attachInterrupt(pinNorth, north2Isr, FALLING);
      break;
    case 3:
      /**
       * target one at azimuth A departing at the speed of 250 [m/s]
       * target two at azimuth B aproaching at the speed of 250 [m/s]
       * |A - B| = 5 Deg
       */
      t0_timeToReceiverUs = 5;
      t0_angleTimeMs = 40;
      t0_angleWidthTimeMs = 75;
      // t0_radialStepUs = (2 * 200.0 /* m/s */ * (northRotTimeMs / 1000.0) / (300000.0 /* m/s */) * 1e6);
      t0_radialStepUs = (200.0 * northRotTimeMs) / (150.0 * 1000);
      t1_timeToReceiverUs = maxDistanceTimeUs;
      t1_angleTimeMs = 40 + 5 * (northRotTimeMs / 360);
      t1_angleWidthTimeMs = 75;
      // t1_radialStepUs = (2 * 200.0 /* m/s */ * (northRotTimeMs / 1000.0) / (300000.0 /* m/s */) * 1e6);
      t1_radialStepUs = (200.0 * northRotTimeMs) / (150.0 * 1000);

      // calculate real start and end of targets
      t0_startMs = t0_angleTimeMs;
      t0_endMs = t0_startMs + t0_angleWidthTimeMs;
      t1_startMs = t1_angleTimeMs;
      t1_endMs = t1_startMs + t1_angleWidthTimeMs;

      attachInterrupt(pinNorth, north3Isr, FALLING);
      break;
  }

  if (programSelect != 0) {
  }

  if (programSelect == 0) {
    /**
     * fixed targets on all azimuth (full circle) - DEMO
     * target one departing at the speed of 250 [m/s]
     * target two aproaching at the speed of 250 [m/s]
     */
    // All programs exept "0" are for specific azimuth
    t0_Attached = true;
    attachInterrupt(pinTargetOneTrigger, targetOneIsr, FALLING);
    t1_Attached = true;
    attachInterrupt(pinTargetTwoTrigger, targetTwoIsr, FALLING);
  }

#ifdef DEBUG
  Serial.print("t0_timeToReceiverUs [us]: ");
  Serial.println(t0_timeToReceiverUs);
  Serial.print("t0_angleTimeMs [ms]: ");
  Serial.println(t0_angleTimeMs);
  Serial.print("t0_angleWidthTimeMs [ms]: ");
  Serial.println(t0_angleWidthTimeMs);
  Serial.print("t0_radialStepUs [us]: ");
  Serial.println(t0_radialStepUs);
  Serial.print("t0_startMs [ms]: ");
  Serial.println(t0_startMs);
  Serial.print("t0_endMs [ms]: ");
  Serial.println(t0_endMs);
  Serial.print("t1_angleTimeMs [ms]: ");
  Serial.println(t1_angleTimeMs);
  Serial.print("t1_radialStepUs [us]: ");
  Serial.println(t1_radialStepUs);
  Serial.print("t1_startMs [ms]: ");
  Serial.println(t1_startMs);
  Serial.print("t1_endMs [ms]: ");
  Serial.println(t1_endMs);
#endif
}

boolean led;
void loop() {
  // early exit while north calculation is not complete
  if (!calculationComplete) {
    return;
  }

  if (programSelect != 0) {
    // All programs exept "0" are for specific azimuth
    if (!t0_Attached && elpsedSinceNorthMs > t0_startMs && elpsedSinceNorthMs < t0_endMs) {
      // setup target one interrupt
      t0_Attached = true;
      attachInterrupt(pinTargetOneTrigger, targetOneIsr, FALLING);
#ifdef DEBUG
      Serial.print("ATTACH T1 ");
      Serial.println(elpsedSinceNorthMs);
#endif
    }
    if (t0_Attached && elpsedSinceNorthMs > t0_endMs) {
      // setup target one interrupt
      t0_Attached = false;
      detachInterrupt(pinTargetOneTrigger);
#ifdef DEBUG
      Serial.print("DETACH T1 ");
      Serial.println(elpsedSinceNorthMs);
#endif
    }

    if (!t1_Attached && elpsedSinceNorthMs > t1_startMs && elpsedSinceNorthMs < t1_endMs) {
      // setup target two interrupt
      t1_Attached = true;
      attachInterrupt(pinTargetTwoTrigger, targetTwoIsr, FALLING);
#ifdef DEBUG
      Serial.print("ATTACH T2 ");
      Serial.println(elpsedSinceNorthMs);
#endif
    }
    if (t1_Attached && elpsedSinceNorthMs > t1_endMs) {
      // setup target one interrupt
      t1_Attached = false;
      detachInterrupt(pinTargetTwoTrigger);
#ifdef DEBUG
      Serial.print("DETACH T2 ");
      Serial.println(elpsedSinceNorthMs);
#endif
    }
  }
}
