# HM-Rev1 Known Issues

**Note: This board was abandoned and never fully assembled**

## 1. AD8232 Incorrect Wiring
* One 4.7 uF capacitor is missing between IOUT and SW
* One resistor and one capacitor are wired incorrectly between REFIN and GND

This error was caught *before* testing the board, so the AD8232 was never tested. Most likely, the signal filtering would not have worked correctly. This is fixed in [HM-Rev2](../HM-Rev2).