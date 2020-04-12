# ledMatrixDemo_t4
Proof of concept LED Matrix driver for Teensy 4.0

This arduino sketch enables a Teensy 4.0 to drive a [32x64 LED matrix panel](https://www.sparkfun.com/products/14718) in full color using the Pixelmatix SmartLED Shield V4 as an interface.

![Photo of Teensy 4.0 on the SmartLED Shield](photo.JPG)

The panel driver uses hardware features of the Teensy 4.0 and SmartLED Shield to drive the panel with near zero CPU overhead. It uses the FlexIO and FlexPWM peripherals together with Direct Memory Access (DMA) to push pixel data to the panel thousands of times per second. In theory, most of the full capabilities of the SmartMatrix library could be ported over to Teensy 4.0 using this technique. This is only a proof of concept to demonstrate the low-level panel interface and fast row updating to produce good color accuracy. It currently lacks a full suite of features (e.g. double buffering, drawing routines, etc).

UPDATE 4/11/2020: the sketch has been updated to support multiple panel displays.

In order to get it set up, you need to make some modifications to the Teensy: two jumper wires must be soldered to pins 32 (yellow) and 33 (green) on the underside of the board before inserting it into the SmartLED Shield. Additionally, there are 4 more jumper wires which must be inserted into the SmartLED shield sockets. Wire together pins 10 & 14, 2 & 12, 11 & 20, 9 & 21, 5 & 32, and 3 & 33. See the image below.

![Photo of the underside of the SmartLED Shield, showing jumper wires](JumperWires.JPG)

Once up and running, you can adjust `latchesPerRow` to change the color bit depth. At 1 bit, the panel can only display fully saturated colors (red, green, blue, magenta, cyan, yellow, white). Good truecolor requires at least 8 bits. Up to 16 bits are possible (but there is little improvement going from 12 to 16 bits). Higher bit depth and display resolution will limit the achievable frame rate, which should be kept above 120 FPS to avoid obvious flickering.

![Comparison between BMP image and resulting display on the LED panel, at 12 bits of color.](demo.jpg)
