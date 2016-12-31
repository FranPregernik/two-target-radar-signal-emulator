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

#include "Arduino.h"
#include "FlexTimer.h"

#define PP_VERSION "1.0.0"

#define OUTPUT_COUNTER_INIT 0

#define FTM_SC_REG_INIT FTM_SC_TOIE_VAL(FTM_SC_TOIE_IE) | FTM_SC_CLKS_VAL(FTM_SC_CLKS_SYSTEM) | FTM_SC_PS_VAL(FTM_SC_PS_1)
#define FTM_CnSC_OUTPUT_REG_SET FTM_CnSC_CHIE_VAL(FTM_CnSC_CHIE_ENABLE) | FTM_CnSC_MSx_VAL(FTM_CnSC_MSx_OUTPUT) | FTM_CnSC_ELSx_VAL(FTM_CnSC_ELSx_OUTPUT_SET) | FTM_CnSC_DMA_VAL(FTM_DMA_DISA)
#define FTM_CnSC_OUTPUT_REG_CLEAR FTM_CnSC_CHIE_VAL(FTM_CnSC_CHIE_ENABLE) | FTM_CnSC_MSx_VAL(FTM_CnSC_MSx_OUTPUT) | FTM_CnSC_ELSx_VAL(FTM_CnSC_ELSx_OUTPUT_CLEAR) | FTM_CnSC_DMA_VAL(FTM_DMA_DISA)

#define CLOCKS_PER_MICROSECOND ((double)F_BUS / 1000000.0)

struct PPPinStruct {
  FlexTimerBase_Struct *FTMBase;
  FlexTimerChannel_Struct *FTMChannel;
  int rtxPin;
  uint8_t pinMux;                    /* PinMux configuration	*/
  uint8_t ftmIRQ;                    /* FlexTimer IRQ Enable	*/
  bool enabled;                      /* Channel enabled		*/
  uint8_t state;                     /* Output State Flag	*/
  uint32_t pinSet;                   /* Value programmed in FTMx_CnSC register to SET	*/
  uint32_t pinClear;                 /* Value programmed in FTMx_CnSC register to CLEAR	*/
  volatile uint32_t posClkRemaining; /* Remaining clock before pin set */
};

void FTMx_INPUT_ISR(int x, uint32_t *FMTx_OF_Cnt, bool *FMTx_OF_Inc);
void FTMx_OUTPUT_ISR(int x);

class PP {
 public:
  PP(void);
  void Output(int numIOs, ...);
  bool RegisterOutput(int txPin);
  bool pulse(int txPin, float microseconds);
  char *getVersion(void);
  friend void ftm0_isr(void);
  friend void ftm1_isr(void);
  friend void ftm2_isr(void);

 private:
  static PP *thisPtr;
};
