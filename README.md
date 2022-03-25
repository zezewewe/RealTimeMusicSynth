# RealTimeMusicSynth
ELEC60013 Embedded Systems
This documentation contains explanations for the implementations of tasks within the system. For the analysis of their performance, please read [this file](Analysis.md).

## Overview of system <a name="link1"></a>
The system allows the music synthesizer to generate the corresponding note pressed by the user for the duration it is held. It also offers customisable volume, octave and type of sound wave from rotating the knobs. <mark>add knob number?</mark>

# Breakdown of Tasks
The functions within this system can be divided into two groups:
1. [Main Functions](#main)
2. [Helper Functions](#helper).

Additionally, a [class](#class-knobdecoder) was created for convenience of coding as well

## Main functions <a name="main"></a>

### Threads
1. scanKeysTask

2. displayUpdateTask


3. decodeTask

4. CAN_TX_Task
This interrupt will obtain messages in the msgOutQ queue using xQueueReceive(), and then transmit through the CAN bus with ID 0x123.


### Interrupts
1. sampleISR

2. CAN_RX_ISR
The interrupt will obtain messages that are received, and place these messages into a msgInQ queue using xQueueSendFromISR().

3. CAN_TX_ISR


## Helper functions <a name="helper"></a>

<mark>add pic of switch matrix here</mark>

1. setRow(uint8_t rowIdx)
This function selects the desired row rowIdx of the switch matrix by writing the corresponding bits to the row select address pins.

2. readCols(uint8_t rowIdx)
This function reads the inputs 


### Class KnobDecoder

There are several methods in the KnobDecoder class.
1. setParams(uint8_t lowerLim, uint8_t upperLim, uint8_t knobIdx)
The function defines the limits of each knob. It takes in a lower limit and upper limit value, and at the same time sets the knob ID that should be used to assign the lower and upper limit value.

2. updateRotationValue(uint8_t keyValueRaw)
The function calculates the current knob state, and then compares that with the previous knob state to determine the direction of rotation. The knob state is then updated. Masks are used to identify each knob.

3. returnRotationValue()
The function returns the knob value