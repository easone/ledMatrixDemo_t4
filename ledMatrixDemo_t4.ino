// Proof of concept matrix test

#include <Arduino.h>
#include <FlexIO_t4.h> // requires FlexIO_t4 library from https://github.com/KurtE/FlexIO_t4
#include <DMAChannel.h>
#include "imageData.h"
#include "gammaLUT.h"

#define MATRIXWIDTH 64
#define MATRIXHEIGHT 32
#define ROWSPERFRAME 16
#define PIXELS_PER_LATCH 64

#define LATCH_TIMER_PULSE_WIDTH_NS  80  // 20 is minimum working value, don't exceed 160 to avoid interference between latch and data transfer
#define LATCH_TO_CLK_DELAY_NS       400  // max delay from rising edge of latch pulse to first pixel clock
#define PANEL_PIXELDATA_TRANSFER_MAXIMUM_NS  42  // time to transfer 1 pixel of data at FlexIO clock rate of 240 MHz (with CLOCK_DIVIDER=10)

#define LATCH_TIMER_PRESCALE  1
#define TIMER_FREQUENCY     (F_BUS_ACTUAL>>LATCH_TIMER_PRESCALE)
#define NS_TO_TICKS(X)      (uint32_t)(TIMER_FREQUENCY * ((X) / 1000000000.0) + 0.5)
#define LATCH_TIMER_PULSE_WIDTH_TICKS   NS_TO_TICKS(LATCH_TIMER_PULSE_WIDTH_NS)

uint8_t panelBrightness = 127; // range 0-255
const uint8_t latchesPerRow = 1; // controls the color depth per pixel; value from 1 to 16; 8 is 24bit truecolor
uint16_t refreshRate = 580; // frames per second. With 12bit color depth, works up to 580 FPS at 600 MHz (720 FPS at 816 MHz)
#define MIN_REFRESH_RATE    (((TIMER_FREQUENCY/65535)/ROWSPERFRAME/2) + 1) // cannot refresh slower than this due to PWM register overflow

typedef struct rgb24 {
    rgb24() : rgb24(0,0,0) {}
    rgb24(uint8_t r, uint8_t g, uint8_t b) {
        red = r; green = g; blue = b;
    }
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} rgb24;

typedef struct rgb48 {
    rgb48() : rgb48(0,0,0) {}
    rgb48(uint16_t r, uint16_t g, uint16_t b) {
        red = r; green = g; blue = b;
    }
    uint16_t red;
    uint16_t green;
    uint16_t blue;
} rgb48;

typedef struct timerpair {
    uint16_t timer_oe;
    uint16_t timer_period;
} timerpair;

FlexIOHandler *pFlex;
IMXRT_FLEXIO_t *flexIO;
DMAChannel dataTransferDMA, enablerDMA, timerUpdateDMA;
IMXRT_FLEXPWM_t *flexpwm = &IMXRT_FLEXPWM2;

// buffers used with DMA transfers
volatile uint32_t DMAMEM rowDataBuffer[latchesPerRow*PIXELS_PER_LATCH*sizeof(uint16_t)/sizeof(uint32_t)];
volatile uint8_t DMAMEM enablerSourceByte;
volatile timerpair DMAMEM timerLUT[latchesPerRow];
rgb24 DMAMEM matrixBuffer[MATRIXWIDTH*MATRIXHEIGHT];

extern const uint32_t PROGMEM testImage[MATRIXWIDTH*MATRIXHEIGHT]; // in imageData.h
extern const uint16_t PROGMEM gammaLUT8to16[256]; // in gammaLUT.h
unsigned int currentRow = 0;
const int dimmingMaximum = 255;
int dimmingFactor = dimmingMaximum - panelBrightness;

void setup() {
  Serial.begin(9600);

  /* Basic pin setup */
  pinMode(10, OUTPUT); // FlexIO2:0 = GPIO_B0_00 - BUFFER_CLK, wire to pin 14
  pinMode(12, OUTPUT); // FlexIO2:1 = GPIO_B0_01 - BUFFER_R1, wire to pin 2
  pinMode(11, OUTPUT); // FlexIO2:2 = GPIO_B0_02 - BUFFER_B2, wire to pin 20
  pinMode(6, OUTPUT); // FlexIO2:10 = GPIO_B0_10 - BUFFER_B1
  pinMode(9, OUTPUT); // FlexIO2:11 = GPIO_B0_11 - BUFFER_R2, wire to pin 21
  pinMode(32, OUTPUT); //FlexIO2:12 = GPIO_B0_12 - BUFFER_G1, wire to pin 5
  pinMode(8, OUTPUT); // FlexIO2:16 = GPIO_B1_00 - BUFFER_G2
  pinMode(4, OUTPUT); // FlexPWM2_0:A = EMC_06 - BUFFER_OE
  pinMode(33, OUTPUT); // FlexPWM2_0:B = EMC_07 - BUFFER_LATCH, wire to pin 3

  /* High speed and drive strength configuration */
  *(portControlRegister(10)) = 0xFF;
  *(portControlRegister(12)) = 0xFF;
  *(portControlRegister(11)) = 0xFF;
  *(portControlRegister(6)) = 0xFF;
  *(portControlRegister(9)) = 0xFF;
  *(portControlRegister(32)) = 0xFF;
  *(portControlRegister(8)) = 0xFF;
  *(portControlRegister(4)) = 0xFF;
  *(portControlRegister(33)) = 0xFF;
  
  loadTestImage();
  calculateTimerLut();
  flexIOSetup();
  dmaSetup();
  formatRowData(currentRow);
  setRowAddress(currentRow);
  flexPWMSetup();
}

void loop() {
  while(true);
}

void loadTestImage() {
  /* Test image (in imageData.h) was generated from a BMP file and is organized bottom-to-top.
     Need to flip the image top-to-bottom and also convert to bit-packed rgb24 format before storing into matrixBuffer. */
  for (int i=0; i<MATRIXHEIGHT; i++) {
    for (int j=0; j<MATRIXWIDTH; j++) {
      uint32_t c = testImage[(MATRIXHEIGHT-i-1)*MATRIXWIDTH+j];
      matrixBuffer[i*MATRIXWIDTH+j] = rgb24(c>>16,c>>8,c);
    }
  }
  /* flush DMAMEM cache */
  arm_dcache_flush_delete((void*)matrixBuffer, sizeof(matrixBuffer));
}

void calculateTimerLut(void) {
  /* adapted from smartMatrix library
   * This code creates a lookup table with a sequence of PWM period and duty cycles which are associated with the LSB through MSB of the image data.
   * The idea is to switch the LEDs on or off with each bitplane so that the total time is equal to the desired color intensity. The MSB accounts for
   * half the available time, and each additional bit is shorter by a factor of 2. This is achieved by changing the period of the PWM signal that
   * drives the panel according to the sequence in the timerLUT lookup table and repeating this process for the full number of bitplanes.
   * Additionally, we can independently control the duty cycle of the Output Enable pin. This can be used to dim any bit component of the
   * total color intensity, so we can make the smallest bits display for a shorter time. Additionally this is used to control the overall panel
   * brightness without affecting color depth.
   * Note: variable "ontime" is named confusingly; it's the duration of the OE pulse, but the panel is actually disabled when OE is HIGH, so the
   * brightness is controlled by the inverse pulse which has width "timer_period" minus "ontime." */
  
    #define TICKS_PER_ROW   (TIMER_FREQUENCY/refreshRate/ROWSPERFRAME)
    #define IDEAL_MSB_BLOCK_TICKS     (TICKS_PER_ROW/2)
    #define MIN_BLOCK_PERIOD_NS (LATCH_TO_CLK_DELAY_NS + (PANEL_PIXELDATA_TRANSFER_MAXIMUM_NS*PIXELS_PER_LATCH))
    #define MIN_BLOCK_PERIOD_TICKS NS_TO_TICKS(MIN_BLOCK_PERIOD_NS)
    #define MSB_BLOCK_TICKS_ADJUSTMENT_INCREMENT    10

    int i;
    uint32_t ticksUsed;
    uint16_t msbBlockTicks = IDEAL_MSB_BLOCK_TICKS + MSB_BLOCK_TICKS_ADJUSTMENT_INCREMENT;

    // start with ideal width of the MSB, and keep lowering until the width of all bits fits within TICKS_PER_ROW
    do {
        ticksUsed = 0;
        msbBlockTicks -= MSB_BLOCK_TICKS_ADJUSTMENT_INCREMENT;
        for (i = 0; i < latchesPerRow; i++) {
            uint16_t blockTicks = (msbBlockTicks >> (latchesPerRow - i - 1)) + LATCH_TIMER_PULSE_WIDTH_TICKS;
            if (blockTicks < MIN_BLOCK_PERIOD_TICKS)
                blockTicks = MIN_BLOCK_PERIOD_TICKS;
            ticksUsed += blockTicks;
        }
    } while (ticksUsed > TICKS_PER_ROW);

    for (i = 0; i < latchesPerRow; i++) {
        // set period and OE values for current block - going from smallest timer values to largest
        // order needs to be smallest to largest so the last update of the row has the largest time between
        // the falling edge of the latch and the rising edge of the latch on the next row - an ISR
        // updates the row in this time

        // period is max on time for this block, plus the dead time while the latch is high
        uint16_t period = (msbBlockTicks >> (latchesPerRow - i - 1)) + LATCH_TIMER_PULSE_WIDTH_TICKS;
        // on-time is the max on-time * dimming factor, plus the dead time while the latch is high
        uint16_t ontime = (((msbBlockTicks >> (latchesPerRow - i - 1)) * dimmingFactor) / dimmingMaximum) + LATCH_TIMER_PULSE_WIDTH_TICKS;

        if (period < MIN_BLOCK_PERIOD_TICKS) {
            uint16_t padding = (MIN_BLOCK_PERIOD_TICKS) - period; // padding is necessary to allow enough time for data to output to the display
            period += padding;
            ontime += padding; // by adding the same padding to the "ontime", the observed intensity is not affected and is still correct
        }

        timerLUT[i].timer_period = period;
        timerLUT[i].timer_oe = ontime;
    }
    /* flush DMAMEM cache */
    arm_dcache_flush_delete((void*)timerLUT, sizeof(timerLUT));

    /* print look-up table (for debugging) */
    for (i = 0; i < latchesPerRow; i++) {
      Serial.print("bitplane "); Serial.print(i);
      Serial.print(": period: "); Serial.print(timerLUT[i].timer_period);
      Serial.print(": ontime: "); Serial.println(timerLUT[i].timer_oe);
    }
}

void flexIOSetup() {
  /* Set up FlexIO peripheral for clocking out data to LED matrix panel. The FlexIO enables parallel output to the panel's RGB data inputs
   * and also generates the clock signal for the panel in hardware. For Teensy 3.x, SmartMatrix uses an 8-bit GPIO port to generate RGB and clock signals
   * by bitbanging, but this is not possible on Teensy 4.0 because the GPIO ports only have 32-bit access and do not have convenient pins.
   *
   * FlexIO provides four 32-bit shift registers which can shift 1, 2, 4, 8, 16, or 32 bits in parallel, which can be assigned to a group of contiguous
   * pins of our choice. We choose pins 1-16 of FlexIO2, of which 1, 2, 3, 10, 11, 12, and 16 are external pins on the Teensy 4.0 board.
   * There are also four timers which can output clock signals and control the shift registers. We assign our clock signal to pin 0.
   * 
   * We configure three of the shift registers:
   * Shifters 0 and 1 will be used for pixel RGB data, and Shifter 2 will be used for matrix row addressing.
   * Shifter 0 outputs to the external pins, Shifter 1 outputs to Shifter 0, and Shifter 2 outputs to Shifter 1.
   * 
   * We configure one of the FlexIO timers to function as the LED panel clock signal. This timer also controls the shifters. The
   * timer is triggered each time dataTransferDMA writes data into the SHIFTBUF[0] and SHIFTBUF[1] registers. The trigger loads the contents of
   * the registers into Shifters 0 and 1 and the full contents of the two shifters are clocked out as the timer decrements to zero.
   * When the data starts clocking out, a DMA trigger is generated which triggers dataTransferDMA to write more data into the registers.
   * After the data finishes clocking out, the new data is subsequently reloaded into the shifters. This process
   * generates uninterrupted clock and data signals until dataTransferDMA completes the row of pixels and disables. 
   * 
   * At this point, the final pixels are clocked out and then when Shifters 0 and 1 are completely empty, the data from Shifter 2 is output to the pins,
   * but the clock signal stops. These signals are not clocked into the LED panel but are instead used to control the row address latches on the SmartLED shield.
   * 
   * All the shifters output 16 bits in parallel. Each half-word contains a pixel of RGB data from the current row interleaved with a pixel 16 rows down. 
   * It would be possible to use 8 bit parallel output, except that there are not enough consecutive external pins on Teensy 4.0 for FlexIO1 and FlexIO2.
   * FlexIO3 does have enough pins but unfortunately cannot be accessed by the DMA peripheral. */

  uint32_t timerSelect, timerPolarity, pinConfig, pinSelect, pinPolarity, shifterMode, parallelWidth, inputSource, stopBit, startBit,
      triggerSelect, triggerPolarity, triggerSource, timerMode, timerOutput, timerDecrement, timerReset, timerDisable, timerEnable;
  
  Serial.println("Start FlexIO setup");

  pFlex = FlexIOHandler::flexIOHandler_list[1]; // get FlexIO2 handler
  flexIO = &pFlex->port(); // Pointer to the port structure in the FlexIO channel
 
  /* Set FlexIO clock, which is independent from CPU and bus clocks (not affected by CPU overclocking) */
  pFlex->setClockSettings(3, 0, 0); // 480 MHz clock

  /* Set up the pin mux for FlexIO */
  pFlex->setIOPinToFlexMode(10);
  pFlex->setIOPinToFlexMode(12);
  pFlex->setIOPinToFlexMode(11);
  pFlex->setIOPinToFlexMode(6);
  pFlex->setIOPinToFlexMode(9);
  pFlex->setIOPinToFlexMode(32);
  pFlex->setIOPinToFlexMode(8);

  /* Enable the clock */
  pFlex->hardware().clock_gate_register |= pFlex->hardware().clock_gate_mask;

  /* Enable the FlexIO with fast register access */
  flexIO->CTRL = FLEXIO_CTRL_FLEXEN | FLEXIO_CTRL_FASTACC;

  /* Shifter 0 registers */ 
  parallelWidth = FLEXIO_SHIFTCFG_PWIDTH(15); // 16-bit parallel shift width
  inputSource = FLEXIO_SHIFTCFG_INSRC*(1); // Input source from Shifter 1
  stopBit = FLEXIO_SHIFTCFG_SSTOP(0); // Stop bit disabled
  startBit = FLEXIO_SHIFTCFG_SSTART(0); // Start bit disabled, transmitter loads data on enable 
  timerSelect = FLEXIO_SHIFTCTL_TIMSEL(0); // Use timer 0
  timerPolarity = FLEXIO_SHIFTCTL_TIMPOL*(1); // Shift on negedge of clock 
  pinConfig = FLEXIO_SHIFTCTL_PINCFG(3); // Shifter pin output
  pinSelect = FLEXIO_SHIFTCTL_PINSEL(1); // Select pins FXIO_D1 through FXIO_D16
  pinPolarity = FLEXIO_SHIFTCTL_PINPOL*(0); // Shifter pin active high polarity
  shifterMode = FLEXIO_SHIFTCTL_SMOD(2); // Shifter transmit mode
  flexIO->SHIFTCFG[0] = parallelWidth | inputSource | stopBit | startBit;
  flexIO->SHIFTCTL[0] = timerSelect | timerPolarity | pinConfig | pinSelect | pinPolarity | shifterMode;

  /* Shifter 1 registers */ 
  parallelWidth = FLEXIO_SHIFTCFG_PWIDTH(15); // 16-bit parallel shift width
  inputSource = FLEXIO_SHIFTCFG_INSRC*(1); // Input source from Shifter 2
  stopBit = FLEXIO_SHIFTCFG_SSTOP(0); // Stop bit disabled
  startBit = FLEXIO_SHIFTCFG_SSTART(0); // Start bit disabled, transmitter loads data on enable 
  timerSelect = FLEXIO_SHIFTCTL_TIMSEL(0); // Use timer 0
  timerPolarity = FLEXIO_SHIFTCTL_TIMPOL*(1); // Shift on negedge of clock 
  pinConfig = FLEXIO_SHIFTCTL_PINCFG(0); // Shifter pin output disabled
  shifterMode = FLEXIO_SHIFTCTL_SMOD(2); // Shifter transmit mode
  flexIO->SHIFTCFG[1] = parallelWidth | inputSource | stopBit | startBit;
  flexIO->SHIFTCTL[1] = timerSelect | timerPolarity | pinConfig | shifterMode;

  /* Shifter 2 registers are configured the same as Shifter 1 */ 
  flexIO->SHIFTCFG[2] = parallelWidth | inputSource | stopBit | startBit;
  flexIO->SHIFTCTL[2] = timerSelect | timerPolarity | pinConfig | shifterMode;

  /* Timer 0 registers */ 
  timerOutput = FLEXIO_TIMCFG_TIMOUT(1); // Timer output is logic zero when enabled and is not affected by the Timer reset
  timerDecrement = FLEXIO_TIMCFG_TIMDEC(0); // Timer decrements on FlexIO clock, shift clock equals timer output
  timerReset = FLEXIO_TIMCFG_TIMRST(0); // Timer never reset
  timerDisable = FLEXIO_TIMCFG_TIMDIS(2); // Timer disabled on Timer compare
  timerEnable = FLEXIO_TIMCFG_TIMENA(2); // Timer enabled on Trigger assert
  stopBit = FLEXIO_TIMCFG_TSTOP(0); // Stop bit disabled
  startBit = FLEXIO_TIMCFG_TSTART*(0); // Start bit disabled
  triggerSelect = FLEXIO_TIMCTL_TRGSEL(1+4*(1)); // Trigger select Shifter 1 status flag
  triggerPolarity = FLEXIO_TIMCTL_TRGPOL*(1); // Trigger active low
  triggerSource = FLEXIO_TIMCTL_TRGSRC*(1); // Internal trigger selected
  pinConfig = FLEXIO_TIMCTL_PINCFG(3); // Timer pin output
  pinSelect = FLEXIO_TIMCTL_PINSEL(0); // Select pin FXIO_D0
  pinPolarity = FLEXIO_TIMCTL_PINPOL*(0); // Timer pin polarity active high
  timerMode = FLEXIO_TIMCTL_TIMOD(1); // Dual 8-bit counters baud mode
  flexIO->TIMCFG[0] = timerOutput | timerDecrement | timerReset | timerDisable | timerEnable | stopBit | startBit;
  flexIO->TIMCTL[0] = triggerSelect | triggerPolarity | triggerSource | pinConfig | pinSelect | pinPolarity | timerMode;
  
  #define FLEXIO_CLOCK_DIVIDER 20 // Output clock frequency is 20 times slower than FlexIO clock (41.7 ns period); minimum value is 18 due to panel hardware.
  #define SHIFTS_PER_TRANSFER 4 // Shift out 4 times with every transfer = two 32-bit words = contents of Shifter 0 and Shifter 1
  flexIO->TIMCMP[0] = ((SHIFTS_PER_TRANSFER*2-1)<<8) | ((FLEXIO_CLOCK_DIVIDER/2-1)<<0);

  /* Enable DMA trigger when data is loaded into Shifter 1 from its register */
  flexIO->SHIFTSDEN |= (1<<1);

  Serial.println("FlexIO setup complete");
}

void flexPWMSetup(void) {
  /* Configure the FlexPWM peripheral to generate a pair of synchronized PWM signals for the panel LATCH and OE inputs.
   * The period and duty cycles are updated dynamically by timerUpdateDMA with each cycle according to the timerLUT lookup table. */

  #define SUBMODULE 0
  Serial.println("Start PWM setup");

  /* set the pin mux for FlexPWM */
  *(portConfigRegister(4)) = 1; // IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_06 mux mode 1 = FLEXPWM2_PWMA00
  *(portConfigRegister(33)) = 1; // IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_07 mux mode 1 = FLEXPWM2_PWMB00

  /* PWM period and duty cycle */
  int mod = 65535; // period (placeholder)
  int compA = 32767; // OE duty cycle (placeholder)
  int compB = LATCH_TIMER_PULSE_WIDTH_TICKS; // Latch duty cycle (not a placeholder)

  /* set up PWM with initial settings */
  uint16_t bitmask = 1<<SUBMODULE;  
  flexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK(bitmask);
  flexpwm->SM[SUBMODULE].CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_PRSC(LATCH_TIMER_PRESCALE);
  flexpwm->SM[SUBMODULE].VAL1 = mod - 1;
  flexpwm->SM[SUBMODULE].VAL3 = compA;
  flexpwm->SM[SUBMODULE].VAL5 = compB;
  flexpwm->OUTEN |= FLEXPWM_OUTEN_PWMA_EN(bitmask);
  flexpwm->OUTEN |= FLEXPWM_OUTEN_PWMB_EN(bitmask);
  flexpwm->MCTRL |= FLEXPWM_MCTRL_LDOK(bitmask);
  
  /* Generate a DMA trigger at the beginning of each cycle (when the latch signal goes high). This is used to trigger
   * the timerUpdateDMA to reload a new period and duty cycle into the registers, which take effect on the next cycle.
   * As an interesting note, this DMA trigger has a unique feature that it allow us to bypass the FLEXPWM_MCTRL_LDOK register. Normally,
   * to update the FlexPWM registers it is necessary to first read, then write to the LDOK register, which would require two additional DMAs. */
  flexpwm->SM[SUBMODULE].DMAEN |= FLEXPWM_SMDMAEN_VALDE;

  Serial.println("PWM setup complete");
}

void dmaSetup() {
  unsigned int minorLoopBytes, minorLoopIterations, majorLoopBytes, majorLoopIterations, destinationAddressModulo;
  int destinationAddressOffset, destinationAddressLastOffset, sourceAddressOffset, sourceAddressLastOffset, minorLoopOffset;
  volatile uint32_t *destinationAddress1, *destinationAddress2, *sourceAddress;
    
  Serial.println("Start DMA setup");

  /* Disable DMA channels so they don't start transferring yet */
  timerUpdateDMA.disable();
  enablerDMA.disable();
  dataTransferDMA.disable();

  /* Enable minor loop mapping so that we can have a minor loop offset (necessary for timerUpdateDMA to write to two non-adjacent registers) */
  DMA_CR |= DMA_CR_EMLM;

  /* Set up timerUpdateDMA.
   * timerUpdateDMA transfers a new value for the OE duty cycle and OE/LATCH period from the lookup table (timerLUT) to the FlexPWM registers.
   * It is triggered by the FlexPWM cycle start (when the latch signal goes high) and subsequently links to the enablerDMA.
   * The destination address, offset, and minor loop offset are set so that each minor loop first writes to the duty cycle register (VAL3),
   * then the period register (VAL1), then resets the address for the next minor loop. */
  minorLoopBytes = sizeof(timerpair);
  majorLoopIterations = latchesPerRow;
  sourceAddress = (volatile uint32_t*) &timerLUT[0].timer_oe;
  sourceAddressOffset = sizeof(timerLUT[0].timer_oe);
  sourceAddressLastOffset = -majorLoopIterations*minorLoopBytes;
  destinationAddress1 = (volatile uint32_t*) &(flexpwm->SM[SUBMODULE].VAL3);
  destinationAddress2 = (volatile uint32_t*) &(flexpwm->SM[SUBMODULE].VAL1);
  destinationAddressOffset = (int)destinationAddress2 - (int)destinationAddress1;
  minorLoopOffset = -2*destinationAddressOffset;
  destinationAddressLastOffset = minorLoopOffset;
  timerUpdateDMA.TCD->SADDR = sourceAddress;
  timerUpdateDMA.TCD->SOFF = sourceAddressOffset;
  timerUpdateDMA.TCD->ATTR_SRC = DMA_TCD_ATTR_SIZE_16BIT;
  timerUpdateDMA.TCD->NBYTES_MLOFFYES = DMA_TCD_NBYTES_DMLOE | DMA_TCD_NBYTES_MLOFFYES_MLOFF(minorLoopOffset) | DMA_TCD_NBYTES_MLOFFYES_NBYTES(minorLoopBytes);
  timerUpdateDMA.TCD->SLAST = sourceAddressLastOffset;
  timerUpdateDMA.TCD->DADDR = destinationAddress1;
  timerUpdateDMA.TCD->ATTR_DST = DMA_TCD_ATTR_SIZE_16BIT;
  timerUpdateDMA.TCD->DOFF = destinationAddressOffset;
  timerUpdateDMA.TCD->DLASTSGA = destinationAddressLastOffset;
  timerUpdateDMA.TCD->BITER = majorLoopIterations;
  timerUpdateDMA.TCD->CITER = majorLoopIterations;
  timerUpdateDMA.triggerAtHardwareEvent(DMAMUX_SOURCE_FLEXPWM2_WRITE0);

  /* Set up enablerDMA.
   * enablerDMA simply writes a bit to the Set Enable register of dataTransferDMA to enable it so it can respond to its trigger.
   * dataTransferDMA is normally disabled because the trigger is active almost all the time and we don't want it to transfer data continuously.
   * We could link dataTransferDMA to enablerDMA but testing shows that this does not make it any faster. */
  enablerSourceByte = DMA_SERQ_SERQ(dataTransferDMA.channel);
  enablerDMA.source(enablerSourceByte);
  enablerDMA.destination(DMA_SERQ);
  enablerDMA.transferCount(1);
  enablerDMA.triggerAtTransfersOf(timerUpdateDMA); // Link enablerDMA to timerUpdateDMA
  enablerDMA.triggerAtCompletionOf(timerUpdateDMA);
  arm_dcache_flush((void*)&enablerSourceByte, sizeof(enablerSourceByte)); // Flush cache because enablerSourceByte is in DMAMEM
  
  /* Set up dataTransferDMA.
   * dataTransferDMA transfers 32-bit words from the rowDataBuffer to the FlexIO shifter registers. For improved speed, we configure it 
   * to transfer two words per trigger: one word to the Shifter 0 register, and the following word to the Shifter 1 register. These registers
   * are contiguous in memory space so we can write to them alternately by using the DMA modulo setting.
   * DMA is fast enough at 600 MHz to refill the shifter registers before they underrun with FlexIO running at FLEXIO_CLOCK_DIVIDER>=18. This ensures
   * that the shifters do not underrun until dataTransferDMA completes at the end of the row (allowing the Shifter 2 address data to output). */
  minorLoopIterations = 2;
  minorLoopBytes = minorLoopIterations*sizeof(uint32_t);
  majorLoopBytes = sizeof(uint16_t)*PIXELS_PER_LATCH;
  majorLoopIterations = majorLoopBytes/minorLoopBytes;
  sourceAddress = &(rowDataBuffer[0]);
  sourceAddressOffset = sizeof(rowDataBuffer[0]);
  sourceAddressLastOffset = 0; // don't reset to beginning of array at completion, move on to next bitplane
  destinationAddress1 = &(flexIO->SHIFTBUF[0]);
  destinationAddressOffset = sizeof(uint32_t);
  destinationAddressLastOffset = 0;
  destinationAddressModulo = 3; // keep the destination address fixed except for the 3 upper bits, effectively making an 8 byte circular buffer
  dataTransferDMA.TCD->SADDR = sourceAddress;
  dataTransferDMA.TCD->SOFF = sourceAddressOffset;
  dataTransferDMA.TCD->SLAST = sourceAddressLastOffset;
  dataTransferDMA.TCD->ATTR_SRC = DMA_TCD_ATTR_SIZE_32BIT;
  dataTransferDMA.TCD->DADDR = destinationAddress1;
  dataTransferDMA.TCD->DOFF = destinationAddressOffset;
  dataTransferDMA.TCD->ATTR_DST = DMA_TCD_ATTR_DMOD(destinationAddressModulo) | DMA_TCD_ATTR_SIZE_32BIT;
  dataTransferDMA.TCD->DLASTSGA = destinationAddressLastOffset;
  dataTransferDMA.TCD->NBYTES = minorLoopBytes;
  dataTransferDMA.TCD->BITER = majorLoopIterations;
  dataTransferDMA.TCD->CITER = majorLoopIterations;
  dataTransferDMA.disableOnCompletion(); // Set dataTransferDMA to automatically disable on completion (we have to enable it using enablerDMA)
  dataTransferDMA.triggerAtHardwareEvent(pFlex->hardware().shifters_dma_channel[1]); // Use FlexIO Shifter 1 trigger

  /* Enable interrupt on completion of DMA transfer. This interrupt is used to format the row buffer data for the next row. */
  dataTransferDMA.interruptAtCompletion();
  dataTransferDMA.attachInterrupt(rowUpdateISR);

  /* Enable DMA channels (except dataTransferDMA which stays disabled, or else it would trigger continuously) */
  enablerDMA.enable();
  timerUpdateDMA.enable();

  Serial.println("DMA setup complete");
}

void rowUpdateISR(void) {
  /* This interrupt runs at the completion of dataTransferDMA when a complete bitplane has been output to the panel.
   * If we have finished all the bitplanes and we have exhausted the row buffer, it's time to update the row buffer with the 
   * next row from the screen buffer. Otherwise, the interrupt does nothing if we are not done with all the bitplanes yet.
   * At this point in the cycle, the timerUpdateDMA transfer for this bitplane already happened and the CITER count was decremented.
   * If CITER = BITER that means that we just finished the last timer update transaction and the next one will be for a new row.
   * In this case it's time to increase the currentRow counter and format the row data for the next row. */
  dataTransferDMA.clearInterrupt();
  if ((timerUpdateDMA.TCD->CITER) == (timerUpdateDMA.TCD->BITER)) {
    currentRow++;
    if (currentRow >= ROWSPERFRAME) currentRow = 0;
    setRowAddress(currentRow);
    formatRowData(currentRow);
    dataTransferDMA.TCD->SADDR = &(rowDataBuffer[0]); // reset dataTransferDMA to point to the first bitplane in the rowDataBuffer
  }
}

inline void formatRowData(unsigned int row) {
  /* adapted from smartMatrix library with modifications
   * This code reads a new row of pixel data from the matrixBuffer, extracts the bit planes for each pixel, and reformats
   * the data into the format needed for the transfer to FlexIO, and stores that in the rowDataBuffer. 
   * In fact, each row is interleaved with another row, (MATRIXHEIGHT/2)=16 rows later.
   * The data structure in rowDataBuffer is organized as follows (different from smartMatrix implementation):
   * The overall buffer is divided into "latchesPerRow" sectors, each of which corresponds to a single bitplane. The first
   * sector is the LSB and the last sector is the MSB.
   * Each sector contains pixel data organized by position in the row. Stepping through the row two pixels at a time, we 
   * store the R, G, and B bits for two successive pixels (interleaved with the corresponding two pixels 16 rows later) into
   * the low and high halves of a 32-bit word.
   * This is also where we perform sRGB gamma decoding. The matrixBuffer is stored in gamma encoded format, but we need 
   * raw intensity values for output to the matrix. We do the decoding using a gamma decoding lookup table. */

    /* Temporary buffers to store gamma-decoded image data for reformatting; static to avoid putting large buffer on the stack.
     * Use 48-bit raw format for the gamma-decoded raw data because 24-bit raw causes data loss at low intensity. */
    static rgb48 tempRow1[PIXELS_PER_LATCH];
    static rgb48 tempRow2[PIXELS_PER_LATCH];

    // clear buffer to prevent garbage data showing
    memset(tempRow1, 0, sizeof(tempRow1));
    memset(tempRow2, 0, sizeof(tempRow2));

    /* Load the current row into tempRow1, and the interleaved row (16 rows down) into tempRow2, performing gamma decoding */
    for(int i=0; i<MATRIXWIDTH; i++) {
        rgb24 currentPixel = matrixBuffer[i+row*MATRIXWIDTH];
        tempRow1[i] = rgb48(gammaLUT8to16[currentPixel.red],
            gammaLUT8to16[currentPixel.green],
            gammaLUT8to16[currentPixel.blue]);
        currentPixel = matrixBuffer[i+(row+(MATRIXHEIGHT/2))*MATRIXWIDTH];
        tempRow2[i] = rgb48(gammaLUT8to16[currentPixel.red],
            gammaLUT8to16[currentPixel.green],
            gammaLUT8to16[currentPixel.blue]);
    }

    /* Step through the rows and extract 4 pixels (two successive pixels from both tempRow1 and tempRow2) */
    int x = 0;
    while(x < PIXELS_PER_LATCH) {
        uint32_t *ptr = (uint32_t*)rowDataBuffer + x*sizeof(uint16_t)/sizeof(uint32_t);
        
        uint16_t r1, g1, b1, r2, g2, b2, r3, g3, b3, r4, g4, b4;
        
        r1 = tempRow1[x].red;
        g1 = tempRow1[x].green;
        b1 = tempRow1[x].blue;
        r2 = tempRow2[x].red;
        g2 = tempRow2[x].green;
        b2 = tempRow2[x].blue;
        x++;
        r3 = tempRow1[x].red;
        g3 = tempRow1[x].green;
        b3 = tempRow1[x].blue;
        r4 = tempRow2[x].red;
        g4 = tempRow2[x].green;
        b4 = tempRow2[x].blue;
        x++;

        /* Union struct is a convenient way to reformat pixels by bitfields */
        union {
          uint32_t word;
          struct {
            // order of bits in word matches how FlexIO connects to the RGB signals
            uint32_t p0r1:1, p0b2:1, p0pad1:7, p0b1:1, p0r2:1, p0g1:1, p0pad2:3, p0g2:1,
            p1r1:1, p1b2:1, p1pad1:7, p1b1:1, p1r2:1, p1g1:1, p1pad2:3, p1g2:1;
          };
        } rgbData;
        rgbData.word = 0;

        /* For each bitplane (from LSB to MSB), extract the RGB bits for all 4 pixels and pack them into a word.
         * Store that word in the correct sector in the rowDataBuffer. */         
        for(int bitplane=16-latchesPerRow; bitplane<16; bitplane++) {
            rgbData.p0r1 = r1 >> bitplane;
            rgbData.p0g1 = g1 >> bitplane;
            rgbData.p0b1 = b1 >> bitplane;
            rgbData.p0r2 = r2 >> bitplane;
            rgbData.p0g2 = g2 >> bitplane;
            rgbData.p0b2 = b2 >> bitplane;
            rgbData.p1r1 = r3 >> bitplane;
            rgbData.p1g1 = g3 >> bitplane;
            rgbData.p1b1 = b3 >> bitplane;
            rgbData.p1r2 = r4 >> bitplane;
            rgbData.p1g2 = g4 >> bitplane;
            rgbData.p1b2 = b4 >> bitplane;
            *ptr = rgbData.word;
            ptr += PIXELS_PER_LATCH*sizeof(uint16_t)/sizeof(uint32_t); // move pointer to next sector
        }
    }

    /* Now we have refreshed the rowDataBuffer and we need to flush cache so that the changes are seen by DMA */
    arm_dcache_flush_delete((void*)rowDataBuffer, sizeof(rowDataBuffer));
}

inline void setRowAddress(unsigned int row) {
  /* Row addressing makes use of the same pins that output RGB color data. This is enabled by additional hardware on the SmartLED Shield.
     The row address signals are latched when the BUFFER_LATCH pin goes high. We need to output the address data without any clock pulses
     to avoid garbage pixel data. We can do this by putting the address data into a third shifter which outputs when the first two shifters
     are emptied (at the end of the row transfer after the DMA channel completes). Only the lower 16 bits will output. */
  union {
    uint32_t word;
    struct {
      // order of bits in word matches how FlexIO connects to the address signals
      uint32_t adx0:1, pad1:8, adx2:1, adx3:1, adx1:1, pad2:3, adx4:1, pad3:16;
    };
  } addressData;
  addressData.word = 0;

  addressData.adx0 = (row & 0x01) ? 1 : 0;
  addressData.adx1 = (row & 0x02) ? 1 : 0;
  addressData.adx2 = (row & 0x04) ? 1 : 0;
  addressData.adx3 = (row & 0x08) ? 1 : 0;
  addressData.adx4 = (row & 0x10) ? 1 : 0;
  flexIO->SHIFTBUF[2] = addressData.word;
}
