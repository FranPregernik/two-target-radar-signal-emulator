/*
* PulsePosition library for Teensy 3.1
* Not related in functionality with the original PPM.
* Used only for it's pulse position functionality and modified
* for my needs.
* Copyright (c) 2016, Fran Pregernik fran.pregernik@gmail.com
*
* Inspired by PulsePosition Library for Teensy 3.1
* Copyright (c) 2014, Robert Collins http://www.rcollins.org
*
* Inspired by original PulsePosition library:
* High resolution input and output of PP encoded signals
* http://www.pjrc.com/teensy/td_libs_PulsePosition.html
* Copyright (c) 2014, Paul Stoffregen, paul@pjrc.com
*
* This library borrowed significant portions of the code from the original library.
* What portions weren't borrowed were entirely rewritten.
*
* The new library will support multiple hardware (FTM)
* timers.  This version of Pulse Position library is entirely table driven:
* all of the support for input and output is defined by the PPPins data
* registration table.  Should future versions of Teensy be created that
* add or delete other PP-capable pins, then the only modifications necessary
* will be the addition or deletion of table entries in PPPins.
*
* Boiler Plate BS:
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice, development funding notice, and this permission
* notice shall be included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*/

#define DEBUG

/*---------------------------------------------------------------------
 * Overview
 * PP -- Pulse Position Modulation library.  Reads and writes PP
 * 			stream data.  PP signals are often used by remote control
 * 			transmitters and receivers to packetize multiple servo
 * 			control streams into a single packet of data.
 *
 * 			This library will simultaneously read and write up to
 * 			twelve PP streams.  Only a single instance of the library
 * 			is required to support simultaneous PP input and output
 * 			streams.
 *
 * 			The PP library is driven by a 48 MHz clock.  Each 'tick'
 * 			is equivalent to 0.020833 uS (20.833 nano-seconds).
 *
 * 			More information about PP may be found here:
 * 			http://www.omegaco.demon.co.uk/mectnpdf/mectn004.pdf
 *
 * Features:
 * 	- Single instance to support all in/out PP streams
 *  - Read/Write up to 12 simultaneous input/output PP streams
 * 	- Each in/out stream is polarity selectable
 * 	- 48 Mhz to 96 Mhz clock resolution
 *
 * HOWTO:
 * Usage: Sending and Receiving PP Encoded Signals
 *
 * PP myPP;
 * Create a PP object. Create only one instance of the PP library.
 * The PP library is designed to handle multiple simultaneous inputs and
 * outputs.  Therefore, create only one instance of the PP library.
 *
 * Public Methods:  (Each described below)
 * 		PP(void);
 *		void 	Output(int polarity, int numIOs, ...);
 *		bool 	RegisterOutput(int polarity, uint8_t framePin, int txPin);
 *		bool 	pulse(int txPin, float microseconds);
 * 		char	*getVersion(void);
 *		friend 	void ftm0_isr(void);
 *		friend 	void ftm1_isr(void);
 *		friend 	void ftm2_isr(void);
 *
 * Public Method:  myPP.Output(POLARITY, numIOs, txPin [, txPin, txPin, ...])
 * 		Valid output pins:  [3, 4, 5, 6, 9, 10, 20, 21, 22, 23, 25, 32]
 * 		Initializes a PP output stream with one or more output pins.  Configures
 * 		each txPin with appropriate polarity, timer channel, and interrupts.
 * 		Each txPin will be ready to send PP pulses.  More than
 * 		one output pin may be specified.  Output may also be called more than
 * 		once, should some pins have different polarity than others.
 * 		Example(s):
 * 			myPP.Output(RISING, 2, 4, 6); - Rising edge, two output pins [4, 6]
 * 			myPP.Output(FALLING, 1, 22); - Falling edge, one output pin [22]
 *
 * Public Method:   bool RegisterOutput(int polarity, int txPin);
 * 		This is the "worker" function called by Output().  For each pin in the
 * 		argument list of Output, RegisterOutput is called.  This is the function
 * 		that actually sets up the appropriate polarity, timer channel, and interrupts.
 * 		Calling this method directly is allowed.
 * 		Example(s):
 * 			myPP.RegisterOutput(RISING, 4); - Rising edge, output pin [4].
 * 			myPP.RegisterOutput(FALLING, 9); - Falling edge, output pin [9].
 *
 * Public Method:  bool pulse(int txPin, float microseconds);
 *		This function populates the output stream of txPin array with channel
 * 		and timing data.  This function will overwrite any previous channel
 * 		data and replace with new data.  The next time an output frame is
 * 		generated, the data in the output channel array will be converted
 * 		to the appropriate timing register data to create a pulse wave of
 * 		the appropriate polarity and duration.
 * 		Example(s):
 *			float val = myPP.dataRead(PP_INPUT_03, i); - Reads from input stream
 *			myPP.pulse(PP_OUTPUT_32, (i % 16) + 1, val); - Writes it to a different output stream
 *
 * Public Method:  char *getVersion(void);
 * 		This function returns the PP library version number string.
 * 		Example(s):
 * 			Serial.print("TeensyPP Library Version Number:  ");
 *			Serial.println(myPP.getVersion());
 *
 * Public Method:  friend void ftm0_isr(void);
 * Public Method:  friend void ftm1_isr(void);
 * Public Method:  friend void ftm2_isr(void);
 * 		Interrupt Service Routines for the FlexTimer Modules.
 * ------------------------------------------------------------------*/

#include "PP.h"

/*---------------------------------------------------------------------
 * Class:		<Static Data>
 * Data:		Version
 *
 * Synopsis:    PP Version Number
 *
 * Defined in: 	PP.h
 *
 * ------------------------------------------------------------------*/
char *Version = PP_VERSION;

/*---------------------------------------------------------------------
 * Class:		<Static Data>
 * Data:		PPPins	: PP PIN registration table
 *
 * Synopsis:    The following table contains all of the valid PP
 * 				Input and Output capable pins.  The table contains a
 * 				pointer to its hardware registers, and localized data.
 * 				The localized data within this structure allows the
 * 				interrupt service routine to function without calling
 * 				a C++ compatible object.
 *
 * Defined in: 	PP.h
 *
 * ------------------------------------------------------------------*/
// clang-format off
PPPinStruct PPPins[] = {
    /*- Flex Timer Base Registers				       Flex Timer Channel Registers					        Pin#	Pin	 FTM			 Channel */
    /*- Flex Timer Base Registers				       Flex Timer Channel Registers					        Pin#	MUX	 IRQ			 Enabled */
    //{(struct FlexTimerBase_Struct *)&FTM0_SC, (struct FlexTimerChannel_Struct *)&FTM0_C1SC, 23,   4,   IRQ_FTM0, false},
    //{(struct FlexTimerBase_Struct *)&FTM0_SC, (struct FlexTimerChannel_Struct *)&FTM0_C2SC, 9,    4,   IRQ_FTM0, false},
    // {(struct FlexTimerBase_Struct *)&FTM0_SC, (struct FlexTimerChannel_Struct *)&FTM0_C3SC, 10,   4,   IRQ_FTM0, false},
    // {(struct FlexTimerBase_Struct *)&FTM0_SC, (struct FlexTimerChannel_Struct *)&FTM0_C4SC, 6,    4,   IRQ_FTM0, false},
    // {(struct FlexTimerBase_Struct *)&FTM0_SC, (struct FlexTimerChannel_Struct *)&FTM0_C5SC, 20,   4,   IRQ_FTM0, false},
    // {(struct FlexTimerBase_Struct *)&FTM0_SC, (struct FlexTimerChannel_Struct *)&FTM0_C6SC, 21,   4,   IRQ_FTM0, false},
    // {(struct FlexTimerBase_Struct *)&FTM0_SC, (struct FlexTimerChannel_Struct *)&FTM0_C7SC, 5,    4,   IRQ_FTM0, false},
    // {(struct FlexTimerBase_Struct *)&FTM1_SC, (struct FlexTimerChannel_Struct *)&FTM1_C1SC, 4,    3,   IRQ_FTM1, false},
    // {(struct FlexTimerBase_Struct *)&FTM2_SC, (struct FlexTimerChannel_Struct *)&FTM2_C0SC, 32,   3,   IRQ_FTM2, false},
    // {(struct FlexTimerBase_Struct *)&FTM2_SC, (struct FlexTimerChannel_Struct *)&FTM2_C1SC, 25,   3,   IRQ_FTM2, false},
    {(struct FlexTimerBase_Struct *)&FTM0_SC, (struct FlexTimerChannel_Struct *)&FTM0_C0SC, 22,   4,   IRQ_FTM0, false},
    {(struct FlexTimerBase_Struct *)&FTM1_SC, (struct FlexTimerChannel_Struct *)&FTM1_C0SC, 3,    3,   IRQ_FTM1, false}
  };
// clang-format on

uint8_t nPins = sizeof(PPPins) / sizeof(PPPins[0]);

PP *PP::thisPtr; /* Currently unused			*/

/*---------------------------------------------------------------------
 * Class:		PP
 * Function:	PP : Class constructor
 *
 * Syntax:		PP myPP;
 *
 * Synopsis:    Instances the PP library.
 *
 * Arguments:
 * 		None
 *
 * Returns:
 * 		None
 * ------------------------------------------------------------------*/
PP::PP(void) {}

/*---------------------------------------------------------------------
 * Class:		PP
 * Function:	Output : Initialize PP Output
 *
 * Syntax:		Output(polarity, numIOs, rxPin, rxPin, rxPin, ...)
 *
 * Valid output pins:  [3, 4, 5, 6, 9, 10, 20, 21, 22, 23, 25, 32]
 *
 * Synopsis:    Initializes PP Output.  This function accepts a variable
 * 				length of arguments that represent the number of PP
 * 				output channels.
 *
 * Arguments:
 * 		polarity	Polarity of the PP output stream (RISING, FALLING).
 * 		numIOs		Number of PP Output channels.  This is a mandatory
 * 					input to tell the Output function how many input
 * 					channels to receive.
 * 		...			Variable number of arguments, each representing a
 * 					differnt PP Output pin.
 *
 * Returns:
 * 		None
 * ------------------------------------------------------------------*/
void PP::Output(int numIOs, ...) {
  int x, txPin;  //, numIOs;
  bool res;
  va_list args;
  va_start(args, numIOs);
  for (x = 0; x < numIOs; x++) {
    txPin = va_arg(args, int);
    res = RegisterOutput(txPin);
  }
  va_end(args);
  thisPtr = this;
}

/*---------------------------------------------------------------------
 * Class:		PP
 * Function:	RegisterOutput : Registers a PIN as a PP Output device
 *
 * Syntax:		RegisterOutput(txPin, polarity)
 *
 * Synopsis:    Registers a PIN as a PP Output device:
 * 				1) Verifies the PIN in the PPPins registration table
 * 				2) Programs the appropriate FTM hardware registers
 * 				3) Sets the PINMUX function to use Flex Timer Module (FTM)
 * 				4) Prioritizes and enables the FTM interrupt
 *
 * Arguments:
 * 		txPin		Teensy 3.1 compatible PIN assignment
 * 		polarity	Polarity of the PP input stream (RISING, FALLING).
 *
 * Returns:
 * 		true	: PP Output successfully registered
 * 		false	: Could not find the PIN number in the PPPins
 * 					registration table
 * ------------------------------------------------------------------*/
bool PP::RegisterOutput(int txPin) {
  int x, y;
  bool results = false;

  for (x = 0; x < nPins; x++) {
    if (txPin == PPPins[x].rtxPin) {
      /* Channel enabled				*/
      PPPins[x].enabled = true;

      /*---------------------------------------------------------------------
       * Initialize the output buffer
       * ------------------------------------------------------------------*/
      PPPins[x].posClkRemaining = 0;

      // /*---------------------------------------------------------------------
      //  * Only program the FTM base registers if necessary.  This is a slight
      //  * optimization.
      //  * ------------------------------------------------------------------*/
      // if (PPPins[x].FTMBase->MOD != 0xFFFF || (PPPins[x].FTMBase->SC & !FTM_SC_TOF_VAL(FTM_SC_TOF_INT)) != FTM_SC_REG_INIT) {
      //   /* Clear interrupt, enable register writes, diesable clocks	*/
      //   PPPins[x].FTMBase->SC = 0;
      //   /* Reset COUNT register	*/
      //   PPPins[x].FTMBase->CNT = 0;
      //   /* Trigger overflow interrupts every 0xFFFF clocks */
      //   PPPins[x].FTMBase->MOD = 0xFFFF;
      //
      //   /*---------------------------------------------------------------------
      //    * Set the FTM (Master) Status and Control as follows
      //    * 		(See PP.h for 'FTM_SC_REG_INIT definition)
      //    * 	TOIE	: Enable overflow interrupt
      //    * 	CLKS	: Use System Clock
      //    * 	PS		: CLK / 1
      //    * ------------------------------------------------------------------*/
      //   PPPins[x].FTMBase->SC = FTM_SC_REG_INIT;
      //   /* Disable register writes */
      //   PPPins[x].FTMBase->MODE = 0;
      // }
      /* Clear interrupt, enable register writes, diesable clocks	*/
      PPPins[x].FTMBase->SC = 0;

      // init and enable the CH0 initial value
      PPPins[x].FTMBase->POL = 1;

      /*---------------------------------------------------------------------
       * Set the FTM Channel Status and Control as follows:
       * 		(See PP.h for 'FTM_SnSC_OUTPUT_REG_INIT definition)
       * 	CHIE	: Interrupt Enable
       * 	MSx		: Output mode
       * 	ELSx	: Rising/Falling polarity
       * 	DMA		: Disabled
       * ------------------------------------------------------------------*/
      PPPins[x].pinSet = FTM_CnSC_OUTPUT_REG_SET;
      PPPins[x].pinClear = FTM_CnSC_OUTPUT_REG_CLEAR;

      /*---------------------------------------------------------------------
       * Set Pin Control Register as follows (Register: PORTx_PCRn):
       * 	MUX		: Pin Mux Control, sets pin definition according to table
       * 				in K20 Manual, Section 10.3.1,
       * 				"K20 Signal Multiplexing and Pin Assignments"
       * 	DSE		: Drive Strength Enable (HIGH driver strength)
       * 	SRE		: Slew Rate Control (SLOW slew rate for digital output)
       * ------------------------------------------------------------------*/
      *portConfigRegister(txPin) = PORT_PCR_MUX(PPPins[x].pinMux) | PORT_PCR_DSE | PORT_PCR_SRE;

      /*---------------------------------------------------------------------
       * Set Nested Vector Interrupt Control
       * ------------------------------------------------------------------*/
      NVIC_SET_PRIORITY(PPPins[x].ftmIRQ, 32);
      NVIC_ENABLE_IRQ(PPPins[x].ftmIRQ);

      /*---------------------------------------------------------------------
       * Returns SUCCESS from function
       * ------------------------------------------------------------------*/
      results = true;
    }
  }
  return results;
}

/*---------------------------------------------------------------------
 * Class:		PP
 * Function:	pulse : Write PP output stream data by PIN/Channel.
 *
 * Syntax:		dataRead(txPin, Channel#, MicroSeconds)
 *
 * Synopsis:    Returns an individual PP input channel item.
 *
 * Arguments:
 * 		txPin			Teensy 3.1 compatible PIN assignment
 * 		Channel#		PP channel #
 * 		MicroSeconds	# Microseconds between rising/falling edges.
 *
 * Returns:
 * 		true			PP Output Success
 * 		false			PP Output Failure
 * ------------------------------------------------------------------*/
bool PP::pulse(int txPin, float microseconds) {
  int x;
  uint32_t i, clocks;

  for (x = 0; x < nPins; x++) {
    if (PPPins[x].rtxPin == txPin) {
      clocks = microseconds * CLOCKS_PER_MICROSECOND;

      __disable_irq();

      /* Clear interrupt, enable register writes, diesable clocks	*/
      PPPins[x].FTMBase->SC = 0;
      // init and enable the CH0 polarity
      PPPins[x].FTMBase->POL = 1;
      /* Reset COUNT register	*/
      PPPins[x].FTMBase->CNT = 0;
      /* Trigger overflow interrupts every 0xFFFF clocks */
      PPPins[x].FTMBase->MOD = 0xFFFF;

      /*---------------------------------------------------------------------
       * Set timer initial value
       * ------------------------------------------------------------------*/
      // take a small chunk of total time to trigger the ISR
      uint16_t maxW = clocks <= 1000 ? clocks / 2 : 1000;
      PPPins[x].posClkRemaining = clocks - maxW;
      PPPins[x].FTMChannel->CnV = maxW;
      PPPins[x].FTMChannel->CnSC = PPPins[x].pinClear;

      /*---------------------------------------------------------------------
       * Set the FTM (Master) Status and Control as follows
       * 		(See PP.h for 'FTM_SC_REG_INIT definition)
       * 	TOIE	: Enable overflow interrupt
       * 	CLKS	: Use System Clock
       * 	PS		: CLK / 1
       * ------------------------------------------------------------------*/
      PPPins[x].FTMBase->SC = FTM_SC_REG_INIT;
      /* Disable register writes */
      PPPins[x].FTMBase->MODE = 0;

      __enable_irq();

      return true;
    }
  }
  return false;
}

/*---------------------------------------------------------------------
 * Class:		PP
 * Function:	getVersion : Read PP library version number
 *
 * Syntax:		char * getVersion()
 *
 * Synopsis:    Returns an individual PP input channel item.
 *
 * Arguments:
 * 		rxPin		Teensy 3.1 compatible PIN assignment
 * 		Channel#	PP channel #
 *
 * Returns:
 * 		<data>	PP Input channel data
 * 		-1		No data available.
 * ------------------------------------------------------------------*/
char *PP::getVersion(void) { return Version; }

/*---------------------------------------------------------------------
 * Class:		<Owned by Hardware>
 * Function:	fmt0_isr : FlexTimer Module-0 Interrupt Service Routine
 *
 * Syntax:		<Called by hardware, no software entry points.>
 *
 * Synopsis:	Services FTM0 interrupts.
 *
 * Arguments:
 *		None
 *
 * Returns:
 * 		Nothing
 * ------------------------------------------------------------------*/
void ftm0_isr(void) {
  int x;
  uint32_t val;

  if (FTM0_SC & FTM_SC_TOF_VAL(FTM_SC_TOF_INT)) {
    FTM0_SC ^= FTM_SC_TOF_VAL(FTM_SC_TOF_INT); /* Clear overflow interrupt		*/
  }

  for (x = 0; x < nPins; x++) {
    /*---------------------------------------------------------------------
     * Service the interrupt IF all of the conditions are met:
     * 	1. Looking for IRQ on FTM0
     * 	2. This channel has been enabled
     * 	3. Interrupt flag is really set
     * ------------------------------------------------------------------*/
    val = PPPins[x].FTMChannel->CnSC;
    if ((PPPins[x].ftmIRQ == IRQ_FTM0) && PPPins[x].enabled && (val & FTM_CnSC_CHF_VAL(FTM_CnSC_CHF_INT))) {
      /*---------------------------------------------------------------------
       * Check for input or output interrupt by looking at the channel mode
       * 	select flags.
       * ------------------------------------------------------------------*/
      FTMx_OUTPUT_ISR(x);
    }
  }
}

/*---------------------------------------------------------------------
 * Class:		<Owned by Hardware>
 * Function:	fmt1_isr : FlexTimer Module-1 Interrupt Service Routine
 *
 * Syntax:		<Called by hardware, no software entry points.>
 *
 * Synopsis:	Services FTM1 interrupts.
 *
 * Arguments:
 *		None
 *
 * Returns:
 * 		Nothing
 * ------------------------------------------------------------------*/
void ftm1_isr(void) {
  int x;
  uint32_t val, count;

  if (FTM1_SC & FTM_SC_TOF_VAL(FTM_SC_TOF_INT)) {
    FTM1_SC ^= FTM_SC_TOF_VAL(FTM_SC_TOF_INT); /* Clear overflow interrupt		*/
  }

  for (x = 0; x < nPins; x++) {
    /*---------------------------------------------------------------------
     * Service the interrupt IF all of the conditions are met:
     * 	1. Looking for IRQ on FTM1
     * 	2. This channel has been enabled
     * 	3. Interrupt flag is really set
     * ------------------------------------------------------------------*/
    val = PPPins[x].FTMChannel->CnSC;
    if ((PPPins[x].ftmIRQ == IRQ_FTM1) && PPPins[x].enabled && (val & FTM_CnSC_CHF_VAL(FTM_CnSC_CHF_INT))) {
      /*---------------------------------------------------------------------
       * Check for input or output interrupt by looking at the channel mode
       * 	select flags.
       * ------------------------------------------------------------------*/
      FTMx_OUTPUT_ISR(x);
    }
  }
}

/*---------------------------------------------------------------------
 * Class:		<Owned by Hardware>
 * Function:	fmt2_isr : FlexTimer Module-2 Interrupt Service Routine
 *
 * Syntax:		<Called by hardware, no software entry points.>
 *
 * Synopsis:	Services FTM2 interrupts.
 *
 * Arguments:
 *		None
 *
 * Returns:
 * 		Nothing
 * ------------------------------------------------------------------*/
void ftm2_isr(void) {
  int x;
  uint32_t val, count;

  if (FTM2_SC & FTM_SC_TOF_VAL(FTM_SC_TOF_INT)) {
    FTM2_SC ^= FTM_SC_TOF_VAL(FTM_SC_TOF_INT); /* Clear overflow interrupt		*/
  }

  for (x = 0; x < nPins; x++) {
    /*---------------------------------------------------------------------
     * Service the interrupt IF all of the conditions are met:
     * 	1. Looking for IRQ on FTM2
     * 	2. This channel has been enabled
     * 	3. Interrupt flag is really set
     * ------------------------------------------------------------------*/
    val = PPPins[x].FTMChannel->CnSC;
    if ((PPPins[x].ftmIRQ == IRQ_FTM2) && PPPins[x].enabled && (val & FTM_CnSC_CHF_VAL(FTM_CnSC_CHF_INT))) {
      /*---------------------------------------------------------------------
       * Check for input or output interrupt by looking at the channel mode
       * 	select flags.
       * ------------------------------------------------------------------*/
      FTMx_OUTPUT_ISR(x);
    }
  }
}

/*---------------------------------------------------------------------
 * Class:		<Owned by Hardware>
 * Function:	FTMx_OUTPUT_ISR	: Common PP output ISR.
 *
 * Syntax:		FTMx_OUTPUT_ISR( x );
 *
 * Synopsis:	This is called  by all of the various FTMx output ISR
 * 				routines.  Each ISR passes the variables and pointers
 * 				necessary to service the interrupt in this common
 * 				subroutine.

 *
 * Arguments:
 *		x				:	Index into PPPins registration table.
 *
 * Returns:
 * 		Nothing
 * ------------------------------------------------------------------*/
void FTMx_OUTPUT_ISR(int x) {
  uint16_t maxW = 0;

  PPPins[x].FTMBase->MODE = 0;

  if (PPPins[x].posClkRemaining > 0) {
    // delay till pulse set
    maxW = PPPins[x].posClkRemaining <= 60000 ? PPPins[x].posClkRemaining : 60000;
    PPPins[x].posClkRemaining -= maxW;
  } else {
    PPPins[x].FTMBase->SC = 0;
  }

  PPPins[x].FTMChannel->CnV += maxW;
  if (PPPins[x].posClkRemaining > 0) {
    PPPins[x].FTMChannel->CnSC = PPPins[x].pinClear;
  } else {
    PPPins[x].FTMChannel->CnSC = PPPins[x].pinSet;
  }
}
