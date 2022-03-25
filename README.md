# RealTimeMusicSynth
ELEC60013 Embedded Systems

This documentation contains explanations for the implementations of tasks within the system. For the analysis of their performance, please read [this file](Analysis.md).

## Overview of system <a name="link1"></a>
The system allows the music synthesizer to generate the corresponding note pressed by the user for the duration it is held. It also offers customisable volume, octave and type of sound wave from rotating the knobs. Lastly, additional features include different modes such as playing monophony, chords, and polyphony (5 notes at a time). It is to be noted that even more notes can be played at once if required. <mark>add knob number?</mark>

# Breakdown of Tasks
The functions within this system can be divided into two groups:
1. [Main Functions](#main)
2. [Helper Functions](#helper)

Additionally, a [class](#class-knobdecoder) was created for modularity, enabling ease of keeping track of knob states and values.

## Main functions <a name="main"></a>

### Threads
1. scanKeysTask
This is a thread that keeps checking for which key is being pressed, storing that information in the global keyArray[] and updating the volatile currentStepSize. Knobs are used to set the waveform as well as octave and volume.

The global array, keyArray[] stores the keys that are pressed and updating it is handled by using the keyArrayMutex.

<mark>not sure if wanna add in the tx_message part?</mark>

2. displayUpdateTask
Through the bitwise comparison of the elements in keyArray, this thread identifies the key pressed and displays the key played on the screen (by reading the corresponding note from the array keyNames). Additionally it also displays the octave (from knob2Rotation) and volume (from knob3Rotation) chosen by the user.

3. decodeTask
The decodeTask thread is used to process received messages. On every received message, the following global arrays are updated using the RxTxArrayMutex. 

The arrays are globalRxTxPressArray, which keeps track of pressing or releasing, the globalRxTxOctaveArray, which keeps track of the octave of the note that is pressed, and lastly the globalRxTxKeyArray, which keeps track of the key number of the note that is pressed. The array is filled up with the details of all the notes that are pressed.

4. CAN_TX_Task
This interrupt will obtain messages in the msgOutQ queue using xQueueReceive(), and then transmit through the CAN bus with ID 0x123.


### Interrupts
1. sampleISR
This interrupt generates the sound after a key has been pressed. Advanced features have been implemented here. Other than just playing one individual note, the user could also opt to play a chord (3 notes in total) from pressing one key, or to have a different sound waveform to be generated, producing a different sound. Depending on the mode chosen by the user (monophony: 0, sound waveform: 1<mark>not sure about this, think i might be looking at the old code from the master branch</mark>, chords:3), the corresponding element from the array phaseAccChordTable will increment by the currentStepSize updated from scanKeysTask. 
    1. Chords
    
    2. Sound Waveforms

To generate the sound of an individual note, it increments phaseAcc

2. CAN_RX_ISR
The interrupt will obtain messages that are received, and place these messages into a msgInQ queue using xQueueSendFromISR().

3. CAN_TX_ISR


## Helper functions <a name="helper"></a>

<mark>add pic of switch matrix here</mark>

1. setRow(uint8_t rowIdx)
This function selects the desired row rowIdx of the switch matrix by writing the corresponding bits to the row select address pins.

2. readCols(uint8_t rowIdx)
This function reads the inputs from the four columns of the switch matrix and returns the four bits concatenated together as a single byte.


### Class KnobDecoder

There are several methods in the KnobDecoder class.
1. setParams(uint8_t lowerLim, uint8_t upperLim, uint8_t knobIdx)
The function defines the limits of each knob. It takes in a lower limit and upper limit value, and at the same time sets the knob ID that should be used to assign the lower and upper limit value.

2. updateRotationValue(uint8_t keyValueRaw)
The function calculates the current knob state, and then compares that with the previous knob state to determine the direction of rotation. The knob state is then updated. Masks are used to identify each knob.

3. returnRotationValue()
The function returns the knob value