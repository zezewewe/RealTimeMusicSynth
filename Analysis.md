# Breakdown of Tasks
The functions within this system can be divided into two groups - [main functions](#main) and [helper functions](#helper).

## Main functions <a name="main"></a>

### Threads
1. scanKeysTask

2. displayUpdateTask

3. decodeTask

4. CAN_TX_Task



### Interrupts
1. sampleISR

2. CAN_RX_ISR

3. CAN_TX_ISR

## Helper functions <a name="helper"></a>

<mark>add pic of switch matrix here</mark>

1. setRow(uint8_t rowIdx)
This function selects the desired row rowIdx of the switch matrix by writing the corresponding bits to the row select address pins.

2. readCols(uint8_t rowIdx)
This function reads 


### Additional Class created - KnobDecoder