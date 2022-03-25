# Analysis

# Table of contents
1. [Min initiation interval](#time_analysis)
2. [Critical Instant Analysis](#paragraph1)
    1. [Sub paragraph](#subparagraph1)
4. [Total CPU utilisation](#CPU_utilisation)
5. [Safety Features](#safety_features)
6. [Analysis of inter-task blocking dependencies](#intertask_blocking)


## Overview of system <a name="link1"></a>
The system allows the music synthesizer to generate the corresponding note pressed by the user for the duration it is held. It also offers customisable volume, octave and type of sound wave from rotating the knobs. <mark>add knob number?</mark>

### Explanation <a name="explanation"></a>
This is a sub paragraph, formatted in heading 3 style

## Min initiation interval <a name="time_analysis"></a>
The first paragraph text

### Sub paragraph <a name="subparagraph1"></a>
This is a sub paragraph, formatted in heading 3 style

## Total CPU utilisation <a name="CPU_utilisation"></a>
Insert para here

## Safety Features <a name="safety_features"></a>
Insert para here

## Analysis of inter-task blocking dependencies <a name="intertask_blocking"></a>
Insert para here


## Helper functions <a name="helper"></a>

<mark>add pic of switch matrix here</mark>

1. setRow(uint8_t rowIdx)
This function selects the desired row rowIdx of the switch matrix by writing the corresponding bits to the row select address pins.

2. readCols(uint8_t rowIdx)
This function reads 


### Additional Class created - KnobDecoder

There are several methods in the class.
1. setParams(uint8_t lowerLim, uint8_t upperLim, uint8_t knobIdx)
The function defines the limits of each knob. It takes in a lower limit and upper limit value, and at the same time sets the knob ID that should be used to assign the lower and upper limit value.

2. updateRotationValue(uint8_t keyValueRaw)
The 
