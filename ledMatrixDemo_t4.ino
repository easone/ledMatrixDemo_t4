/* Proof of concept demo display driver for LED Matrix using Teensy 4.0.
 * Uses Pixelmatix SmartLED Shield. Some jumper wires are necessary (see README). 
 * Portions of this code were adapted from the SmartMatrix library by Pixelmatix. */

#include <Arduino.h>
#include <FlexIO_t4.h> // requires FlexIO_t4 library from https://github.com/KurtE/FlexIO_t4
#include <DMAChannel.h>
#include "imageData.h"
#include "gammaLUT.h"

#define SMARTMATRIX_OPTIONS_NONE                    0
#define SMARTMATRIX_OPTIONS_C_SHAPE_STACKING        (1 << 0)
#define SMARTMATRIX_OPTIONS_BOTTOM_TO_TOP_STACKING  (1 << 1)

const int matrixWidth = 128; // height of the overall display
const int matrixHeight = 64; // width of the overall display
const int matrixPanelHeight = 32; // height of the individual panels making up the display
const unsigned char optionFlags = SMARTMATRIX_OPTIONS_C_SHAPE_STACKING;

uint8_t panelBrightness = 64; // range 0-255
const uint8_t latchesPerRow = 12; // controls the color depth per pixel; value from 1 to 16; 8 is 24bit truecolor
uint16_t refreshRate = 200; // frames per second. With 12bit color depth, works up to 580 FPS at 600 MHz (720 FPS at 816 MHz)
const uint8_t dmaBufferNumRows = 4; // number of rows of pixel data in rowDataBuffer

#define LATCH_TIMER_PULSE_WIDTH_NS  80  // 20 is minimum working value, don't exceed 160 to avoid interference between latch and data transfer
#define LATCH_TO_CLK_DELAY_NS       400  // max delay from rising edge of latch pulse to first pixel clock
#define PANEL_PIXELDATA_TRANSFER_MAXIMUM_NS  43  // time to transfer 1 pixel of data at FlexIO clock rate with FLEXIO_CLOCK_DIVIDER=20

#define LATCH_TIMER_PRESCALE  0
#define TIMER_FREQUENCY     (F_BUS_ACTUAL>>LATCH_TIMER_PRESCALE)
#define NS_TO_TICKS(X)      (uint32_t)(TIMER_FREQUENCY * ((X) / 1000000000.0) + 0.5)
#define LATCH_TIMER_PULSE_WIDTH_TICKS   NS_TO_TICKS(LATCH_TIMER_PULSE_WIDTH_NS)
#define MATRIX_STACK_HEIGHT  (matrixHeight / matrixPanelHeight)
#define PIXELS_PER_LATCH  (matrixWidth * MATRIX_STACK_HEIGHT)
#define INLINE __attribute__( ( always_inline ) ) inline

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

const uint16_t dmaBufferBytesPerRow = latchesPerRow*PIXELS_PER_LATCH*sizeof(uint16_t); // each pixel takes two bytes
const int matrixRowPairOffset = matrixPanelHeight / 2;
const int matrixRowsPerFrame = matrixPanelHeight / 2;

volatile uint32_t DMAMEM rowDataBuffer[dmaBufferBytesPerRow*dmaBufferNumRows/sizeof(uint32_t)];
volatile uint8_t DMAMEM enablerSourceByte;
volatile timerpair DMAMEM timerLUT[latchesPerRow];
unsigned int rowAddressBuffer[dmaBufferNumRows];
rgb24 matrixBuffer[matrixWidth*matrixHeight];

const uint16_t imageWidth = 64;
const uint16_t imageHeight = 32;
extern const uint32_t PROGMEM testImage[imageHeight*imageWidth]; // in imageData.h
extern const uint16_t PROGMEM gammaLUT8to16[256]; // in gammaLUT.h
const uint8_t dimmingMaximum = 255;
int dimmingFactor = dimmingMaximum - panelBrightness;
unsigned int rowBufferIndex = 0;
unsigned int rowBufferFillCount = 0;

FLASHMEM void setup() {
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
  
  loadTestImage(imageWidth, imageHeight);
  calculateTimerLut();
  flexIOSetup();
  dmaSetup();
  fillRowBuffer();
  setRowAddress(0);
  flexPWMSetup();
}

void loop() {
  while(true);
}

FLASHMEM void loadTestImage(uint16_t wd, uint16_t ht) {
  /* Test image (in imageData.h) was generated from a BMP file and is organized bottom-to-top.
     Need to flip the image top-to-bottom and also convert to bit-packed rgb24 format before storing into matrixBuffer.
     Tile the image to fill up the full display. */
  int widthReps = matrixWidth/wd;
  int heightReps = matrixHeight/ht;
  for (int y=0; y<ht; y++) {
    for (int x=0; x<wd; x++) {
      uint32_t c = testImage[(imageHeight-y-1)*imageWidth+x];
      for (int i=0; i<widthReps; i++) {
        for (int j=0; j<heightReps; j++) {
          matrixBuffer[(y+j*ht)*matrixWidth+(x+i*wd)] = rgb24(c>>16,c>>8,c);
        }
      }      
    }
  }
}

FLASHMEM void calculateTimerLut(void) {
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
  
    #define TICKS_PER_ROW   (TIMER_FREQUENCY/refreshRate/matrixRowsPerFrame)
    #define IDEAL_MSB_BLOCK_TICKS     (TICKS_PER_ROW/2) * (1<<latchesPerRow) / ((1<<latchesPerRow) - 1) 
    #define MIN_BLOCK_PERIOD_NS (LATCH_TO_CLK_DELAY_NS + (PANEL_PIXELDATA_TRANSFER_MAXIMUM_NS*PIXELS_PER_LATCH))
    #define MIN_BLOCK_PERIOD_TICKS NS_TO_TICKS(MIN_BLOCK_PERIOD_NS)
    #define MSB_BLOCK_TICKS_ADJUSTMENT_INCREMENT    10
    #define MIN_REFRESH_RATE    (((TIMER_FREQUENCY/65535)/matrixRowsPerFrame/2) + 1) // cannot refresh slower than this due to PWM register overflow

    int i;
    uint32_t ticksUsed;
    uint16_t msbBlockTicks = IDEAL_MSB_BLOCK_TICKS + MSB_BLOCK_TICKS_ADJUSTMENT_INCREMENT;

    if (MIN_BLOCK_PERIOD_TICKS * latchesPerRow >= TICKS_PER_ROW) {
      Serial.println("Error: refresh rate too high for this resolution and color depth");
      exit(1);
    }
    if (refreshRate < MIN_REFRESH_RATE) {
      Serial.println("Error: refresh rate too low - try increasing LATCH_TIMER_PRESCALE");
      exit(1);
    }

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
    Serial.print("Max brightness limited to "); Serial.print(msbBlockTicks*(200-(200>>latchesPerRow))/TICKS_PER_ROW); Serial.println("%");
    for (i = 0; i < latchesPerRow; i++) {
      Serial.print("bitplane "); Serial.print(i);
      Serial.print(": period: "); Serial.print(timerLUT[i].timer_period);
      Serial.print(": ontime: "); Serial.println(timerLUT[i].timer_oe);
    }
}

FLASHMEM void flexIOSetup() {
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
  
  #define FLEXIO_CLOCK_DIVIDER 20 // Output clock frequency is 20 times slower than FlexIO clock (41.7 ns period); limited by panel hardware.
  #define SHIFTS_PER_TRANSFER 4 // Shift out 4 times with every transfer = two 32-bit words = contents of Shifter 0 and Shifter 1
  flexIO->TIMCMP[0] = ((SHIFTS_PER_TRANSFER*2-1)<<8) | ((FLEXIO_CLOCK_DIVIDER/2-1)<<0);

  /* Enable DMA trigger when data is loaded into Shifter 1 from its register */
  flexIO->SHIFTSDEN |= (1<<1);

  Serial.println("FlexIO setup complete");
}

FLASHMEM void flexPWMSetup(void) {
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

FLASHMEM void dmaSetup() {
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

  /* Enable interrupt on completion of DMA transfer. This interrupt is used to update the DMA addresses to point to the next row. */
  dataTransferDMA.interruptAtCompletion();
  dataTransferDMA.attachInterrupt(rowUpdateISR);

  /* Borrow the interrupt allocated to timerUpdateDMA to use as a software interrupt. It is triggered manually inside rowUpdateISR (not by timerUpdateDMA). */
  #define ROW_CALCULATION_ISR_PRIORITY 240 // lowest priority
  NVIC_SET_PRIORITY(IRQ_DMA_CH0 + timerUpdateDMA.channel, ROW_CALCULATION_ISR_PRIORITY);
  timerUpdateDMA.attachInterrupt(rowCalculationISR);
  
  /* Enable DMA channels (except dataTransferDMA which stays disabled, or else it would trigger continuously) */
  enablerDMA.enable();
  timerUpdateDMA.enable();

  Serial.println("DMA setup complete");
}

FASTRUN void rowUpdateISR(void) {
  /* This interrupt runs at the completion of dataTransferDMA when a complete bitplane has been output to the panel.
   * If we have finished all the bitplanes for this row, it's time to update the source address of the dataTransferDMA to
   * point to the next row in the rowDataBuffer. Otherwise, the interrupt does nothing if we are not done with all the bitplanes yet.
   * To determine whether all the bitplanes are done, we look at the timerUpdateDMA which keeps an iteration count that 
   * corresponds to the bitplanes. In each cycle, timerUpdateDMA is serviced first, before rowUpdateISR.
   * If CITER = BITER that means that timerUpdateDMA finished the last timer update transaction and the next one will be for a new row.
   * In this case it's time to update dataTransferDMA to the next row.*/
   
  dataTransferDMA.clearInterrupt(); // if this line is at the end of the ISR then it does not clear fast enough and the ISR can be triggered twice
  
  if ((timerUpdateDMA.TCD->CITER) == (timerUpdateDMA.TCD->BITER)) {
    rowBufferIndex = (rowBufferIndex + 1) % dmaBufferNumRows; // increment the index pointing to the location of the next row in the rowDataBuffer
    --rowBufferFillCount; // indicates that the buffer needs to be refilled
    dataTransferDMA.TCD->SADDR = &(rowDataBuffer[dmaBufferBytesPerRow*rowBufferIndex/sizeof(uint32_t)]); // update DMA source address
    NVIC_SET_PENDING(IRQ_DMA_CH0 + timerUpdateDMA.channel); // trigger software interrupt rowCalculationISR()
    setRowAddress(rowAddressBuffer[rowBufferIndex]); // change the row address we send to the panel
  }
}

FASTRUN void rowCalculationISR(void) {
  fillRowBuffer();
}

FASTRUN INLINE void fillRowBuffer(void) { // Refills the row buffer. It may be interrupted by the rowUpdateISR.
  static unsigned int currentRow = 0; // global variable keeps track of the next row to write into the buffer
  while (rowBufferFillCount != dmaBufferNumRows) { // loop until buffer is full
    unsigned int freeRowBuffer = (rowBufferIndex + rowBufferFillCount) % dmaBufferNumRows; // index of the next free location in the rowDataBuffer
    formatRowData(currentRow, freeRowBuffer); // take pixel data from the matrixBuffer and copy/reformat it into the rowDataBuffer
    rowAddressBuffer[freeRowBuffer] = currentRow; // record the corresponding address in the rowAddressBuffer
    ++rowBufferFillCount;
    if (++currentRow >= matrixRowsPerFrame) currentRow = 0;
  }
}

FASTRUN INLINE void formatRowData(unsigned int row, unsigned int freeRowBuffer) {
  /* adapted from smartMatrix library (with modifications)
   * This code reads a new row of pixel data from the matrixBuffer, extracts the bit planes for each pixel, and reformats
   * the data into the format needed for the transfer to FlexIO, and stores that in the rowDataBuffer. 
   * In fact, each row is interleaved with another row, matrixRowPairOffset rows later.
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

    /* Load the 2 rows from the matrixBuffer and store in tempRow1 and tempRow2. Moves through the entire chain of panels and extracts
       rows from each one, using the stacking options to get the correct rows (some panels can be upside down). */
    int y1, y2;
    for(int i=0; i<MATRIX_STACK_HEIGHT; i++) {
        // Z-shape, bottom to top
        if(!(optionFlags & SMARTMATRIX_OPTIONS_C_SHAPE_STACKING) &&
            (optionFlags & SMARTMATRIX_OPTIONS_BOTTOM_TO_TOP_STACKING)) {
            // fill data from bottom to top, so bottom panel is the one closest to Teensy
            y1 = row + (MATRIX_STACK_HEIGHT-i-1)*matrixPanelHeight;
            y2 = y1 + matrixRowPairOffset;
        // Z-shape, top to bottom
        } else if(!(optionFlags & SMARTMATRIX_OPTIONS_C_SHAPE_STACKING) &&
            !(optionFlags & SMARTMATRIX_OPTIONS_BOTTOM_TO_TOP_STACKING)) {
            // fill data from top to bottom, so top panel is the one closest to Teensy
            y1 = row + i*matrixPanelHeight;
            y2 = y1 + matrixRowPairOffset;
        // C-shape, bottom to top
        } else if((optionFlags & SMARTMATRIX_OPTIONS_C_SHAPE_STACKING) &&
            (optionFlags & SMARTMATRIX_OPTIONS_BOTTOM_TO_TOP_STACKING)) {
            // alternate direction of filling (or loading) for each matrixwidth
            // swap row order from top to bottom for each stack (tempRow1 filled with top half of panel, tempRow0 filled with bottom half)
            if((MATRIX_STACK_HEIGHT-i+1)%2) {
                y2 = (matrixRowsPerFrame-row-1) + (i)*matrixPanelHeight;
                y1 = y2 + matrixRowPairOffset;
            } else {
                y1 = row + (i)*matrixPanelHeight;
                y2 = y1 + matrixRowPairOffset;
            }
        // C-shape, top to bottom
        } else if((optionFlags & SMARTMATRIX_OPTIONS_C_SHAPE_STACKING) && 
            !(optionFlags & SMARTMATRIX_OPTIONS_BOTTOM_TO_TOP_STACKING)) {
            if((MATRIX_STACK_HEIGHT-i)%2) {
                y1 = row + (MATRIX_STACK_HEIGHT-i-1)*matrixPanelHeight;
                y2 = y1 + matrixRowPairOffset;
            } else {
                y2 = (matrixRowsPerFrame-row-1) + (MATRIX_STACK_HEIGHT-i-1)*matrixPanelHeight;
                y1 = y2 + matrixRowPairOffset;
            }
        }
        int stackOffset = i*matrixWidth;
        for(int j=0; j<matrixWidth; j++) {
          rgb24 currentPixel1 = matrixBuffer[(y1*matrixWidth)+j];
          rgb24 currentPixel2 = matrixBuffer[(y2*matrixWidth)+j];
          tempRow1[j+stackOffset] = rgb48(gammaLUT8to16[currentPixel1.red],
              gammaLUT8to16[currentPixel1.green],
              gammaLUT8to16[currentPixel1.blue]);
          tempRow2[j+stackOffset] = rgb48(gammaLUT8to16[currentPixel2.red],
              gammaLUT8to16[currentPixel2.green],
              gammaLUT8to16[currentPixel2.blue]);
        }    
    }

    uint32_t *startOfRowBuffer = (uint32_t*)rowDataBuffer + dmaBufferBytesPerRow*freeRowBuffer/sizeof(uint32_t);

    /* Step through the rows and extract 4 pixels (two successive pixels from both tempRow1 and tempRow2) */
    for (int i = 0; i < PIXELS_PER_LATCH; i+=2) {
        uint32_t *ptr = startOfRowBuffer + i*sizeof(uint16_t)/sizeof(uint32_t);
        uint16_t r1, g1, b1, r2, g2, b2, r3, g3, b3, r4, g4, b4;
        int ind0, ind1;

        // for upside down stacks, flip order
        if((optionFlags & SMARTMATRIX_OPTIONS_C_SHAPE_STACKING) && !((i/matrixWidth)%2)) {
            int tempPosition = ((i/matrixWidth) * matrixWidth) + matrixWidth - i%matrixWidth - 1;
            ind0 = tempPosition;
            ind1 = tempPosition-1;
        } else {
            ind0 = i;
            ind1 = i+1;
        }
        r1 = tempRow1[ind0].red;
        g1 = tempRow1[ind0].green;
        b1 = tempRow1[ind0].blue;
        r2 = tempRow2[ind0].red;
        g2 = tempRow2[ind0].green;
        b2 = tempRow2[ind0].blue;
        r3 = tempRow1[ind1].red;
        g3 = tempRow1[ind1].green;
        b3 = tempRow1[ind1].blue;
        r4 = tempRow2[ind1].red;
        g4 = tempRow2[ind1].green;
        b4 = tempRow2[ind1].blue;

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
    arm_dcache_flush_delete((void*)startOfRowBuffer, dmaBufferBytesPerRow);
}

FASTRUN INLINE void setRowAddress(unsigned int row) {
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
