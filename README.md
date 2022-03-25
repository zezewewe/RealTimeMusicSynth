# Real-Time Music Synthesizer Coursework

## Introduction
Documentation for this Repository is split into two parts. This readme file explains the implementation of tasks and explanation of the code. Analysis of the performance of tasks can be found [here](Analysis.md). 

### Overview of Synthesizer <a name="link1"></a>
- Our Synthesizer is modular, and two keyboards can be programmed to be played together, connected through a CAN bus. 
- Sound is output from the speaker of the receiver - the role of each keyboard is displayed clearly on screen. 


### User-defined specifications
- Knobs allow for the user to choose between three waveforms (Sawtooth, Triangle, Sinusoidal) and the preferred volume level (5-16). These options are displayed interactively on the display screen. 
- The octave is pre-defined, with the receiver on octave 5 and the transmitter an octave above. The note played is displayed on screen. 
- Three playing modes are available, and polyphony is the default setting. In polyphony, up to a user-defined number of notes can be played at any time (the default is 10). In monophony, only 1 note can be played at a time. In chords, the playing of one note will result in that major chord being sound. The chords mode was meant as an exploration of adding waveforms together, and acts as a basso continuo. 

### Demo Video
The following video demonstrates polyphony across two keyboards, the left of which is RX, and the right RX. Sound output comes out from the left speaker. Notes are played for all three waveforms, Sawtooth, Triangle and Sinusoidal. 

[![GoelSynth](https://img.youtube.com/vi/0E2B9Qj-xCs/0.jpg)](https://youtu.be/0E2B9Qj-xCs "GoelSynth Demo")


## Explaining the codebase
The code in the repository is split into three main parts:
1. [Main Functions](#main)
2. [Helper Functions](#helper)
3. [Definitions](#defs)

Additionally, a [class](#class-knobdecoder) was created for modularity, enabling ease of keeping track of knob states and values.

### Interesting threads and interrupts <a name="main"></a>

#### Thread 1: scanKeysTask: 
- This thread checks the state of all keys and stores it in the global keyArray[] for comparison in the next time interval. A message is created based on the changed keypresses (both press and release), and depending on the RX/TX role of the keyboard, the message is sent to different queues. 
- Messages on the receiver will be added to the receiver queue directly, while messages on the transmitter will be added to a transmit queue to be sent over the CAN bus.
- Knob states are also checked and updated in this thread. This updates the waveform and volume. 
- The global array, keyArray[] stores the keys that are pressed and updating it is handled by using the keyArrayMutex.

#### Thread 2: displayUpdateTask
- This thread provides visual representation on the following: (1) RX or TX or Loopback; (2) Preset octave number; (3) Latest note played; (4) Tunable Waveform Type; (5) Tunable Volume number

#### Thread 3: decodeTask
- This thread receives messages either directly (from the receiver) or from the CAN bus (from the transmitter). A unique identifier (multipliedID) is associated with each note, where the multipliedID is obtained through multiplying (octave+1) and (key+1). 
- If the message received is a Release, the index of the note is found using this identifier, and globalRxTxMultipliedArrayp[idx] is set to 0 (i.e. empty). 
- Else if the message received is a Press, the decision whether to add the note to the global arrays will be made based on the globalRxTxCounter, which keeps track of the total number of occupants in each array. Only a maximum of notes pre-defined can be accepted. The arrays will update accordingly. The following global arrays are updated while being protected by RxTxArrayMutex, several of which are also accessed by other threads. 
- globalRxTxOctaveArray keeps track of the octave of the note that is pressed; globalRxTxKeyArray keeps track of the key number of the note that is pressed; and globalRxTxMultipliedArray keeps track of the identifier of each note being pressed (empty occupant if 0). globalRxTxCounter is tracked through use of a local copy through atomic load and stores. 


#### sampleISR
- This interrupt has three modes - with the default being set as polyphony. Alternatively, the user can choose (within definitions.h) chord and monophony mode, though they are deprecated and polyphony mode replaced them. 
- Users are able to select between three waveforms, which are displayed on screen and tunable using the knob.
- A lookup table is used to store the stepSizes needed for the generation of a sinusoid wave. The code used to generate this lookup table can be found in /src/generateSineLookupTable.py 

### Helper functions <a name="helper"></a>

These consist of the main functions in the coursework 2 lab practical sheets: 

- setRow(uint8_t rowIdx)
This function selects the desired row rowIdx of the switch matrix by writing the corresponding bits to the row select address pins.

- readCols(uint8_t rowIdx)
This function reads the inputs from the four columns of the switch matrix and returns the four bits concatenated together as a single byte.


### Class KnobDecoder

Due to the numerous knobs available, a class was created with the following methods:
- setParams(uint8_t lowerLim, uint8_t upperLim, uint8_t knobIdx) allows for easy initialization of the upper and lower limits of each knob. It also takes in the knobIdx which is essential to identifying the number of bits to shift right by for state comparison.
- updateRotationValue(uint8_t keyValueRaw) calculates the current knob state, and compares that with the previous knob state to determine the direction of rotation. The knob state is then updated. Masks are used to identify each knob. Direction of motion in each iteration is saved to account for the possibility that a state is missed. 
- returnRotationValue() returns the knob location (within the lower and upper limits)