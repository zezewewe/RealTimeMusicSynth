#include <Arduino.h>
#include <U8g2lib.h>
#include <cmath>
#include <STM32FreeRTOS.h>
#include <algorithm>
#include <definitions.h>
#include <helperFunctions.h>
#include <ES_CAN.h>
#include <waveformFunctions.h>

#define polyphony 1 // default setting 
// #define monophony 1
// #define chords 1

void sampleISR() {
  /* 
  Interrupt which converts stepSizes to Vout values played by the speaker
  */

  int32_t Vout;
  uint8_t localKnob2 = __atomic_load_n(&knob2Rotation, __ATOMIC_RELAXED); // retrieve required waveform
  uint8_t localKnob3 = __atomic_load_n(&knob3Rotation, __ATOMIC_RELAXED); // retrieve required volume

  #if polyphony 
    static int32_t localcurrentStepSizeArr[maxNotesStored];
    static int32_t phaseAccChordTable[maxNotesStored];
    static int32_t currentPhaseChordTable[maxNotesStored]; 

    for (int i=0;i<maxNotesStored;i++){
      localcurrentStepSizeArr[i]=currentStepSizeArr[i];
    }

    for (int i=0;i<maxNotesStored;i++){
      phaseAccChordTable[i] += localcurrentStepSizeArr[i];
      currentPhaseChordTable[i] = phaseAccChordTable[i]>>24;
    }

    if (localKnob2==0){ // sawtooth
      Vout = 0;
      for (int i=0;i<maxNotesStored;i++){
        Vout += currentPhaseChordTable[i];
      }
    } else if (localKnob2==1) { // triangle
      for (int i=0;i<maxNotesStored;i++){
        if (currentPhaseChordTable[i]<= 0) { 
          Vout += 128+2*currentPhaseChordTable[i];
        } else {
          Vout += 127-2*currentPhaseChordTable[i];
        } 
      }
    } else if (localKnob2==2) { // sinusoid
      for (int i=0;i<maxNotesStored;i++){
        Vout += sineAmplitudeArray[currentPhaseChordTable[i]+128];
      }
    }
    #elif monophony // Single line played (deprecated with the new polyphony update)
    static int32_t phaseAccChordTable[1];
    static int32_t currentPhaseChordTable[1];

    phaseAccChordTable[0] += currentStepSize; 
    currentPhaseChordTable[0] = phaseAccChordTable[0]>>24;

    if (localKnob2==0){ // sawtooth
      Vout = currentPhaseChordTable[0];
    } else if (localKnob2==1) { // triangle
      if (currentPhaseChordTable[0]<= 0) { 
        Vout = 128+2*currentPhaseChordTable[0];
      } else {
        Vout = 127-2*currentPhaseChordTable[0];
      } 
    } else if (localKnob2==2) { // sinusoid
      Vout = sineAmplitudeArray[currentPhaseChordTable[0]+128];
    }

  #elif chords // Play a note and a chord is played (deprecated with the new polyphony update)
    static int32_t phaseAccChordTable[3];
    static int32_t currentPhaseChordTable[3];

    phaseAccChordTable[0] += currentStepSize; 
    currentPhaseChordTable[0] = phaseAccChordTable[0]>>24;
    for (int i=1;i<3;i++){
      phaseAccChordTable[i] += (int32_t)(currentStepSize*pow(2,(i*3.0+1.0)/12.0));
      currentPhaseChordTable[i] = (phaseAccChordTable[i])>>24;
    }
    
    if (localKnob2==0){ // sawtooth
      Vout = 0;
      for (int i=0;i<3;i++){
        Vout += currentPhaseChordTable[i];
      }
    } else if (localKnob2==1) { // triangle
      for (int i=0;i<3;i++){
        if (currentPhaseChordTable[i]<= 0) { 
          Vout += 128+2*currentPhaseChordTable[i];
        } else {
          Vout += 127-2*currentPhaseChordTable[i];
        } 
      }
    } else if (localKnob2==2) { // sinusoid
      for (int i=0;i<3;i++){
        Vout += sineAmplitudeArray[currentPhaseChordTable[i]+128];
      }
    }

  #endif

  // Volume scaling to -128 to 127 range
  Vout = Vout >> (8 - localKnob3/2);
  analogWrite(OUTR_PIN, Vout + 128);
}

// Interrupt 2
void CAN_RX_ISR(void){
  uint8_t RX_Message_ISR[8];
  uint32_t ID;
  CAN_RX(ID,RX_Message_ISR); // gets message data
  xQueueSendFromISR(msgInQ,RX_Message_ISR,NULL); // places rcv'd data to queue
}

// Thread 4
void CAN_TX_Task(void * pvParameters) {
  uint8_t msgOut[8];
  while (1){
    xQueueReceive(msgOutQ,msgOut,portMAX_DELAY);
    xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
    CAN_TX(0x123,msgOut);
  }
}

// Interrupt 3 for Thread 4
void CAN_TX_ISR(void){
  xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL); //give semaphore each time mailbox is available
}

// Thread 1
void scanKeysTask(void * pvParameters) { 
  /*
  Loop through rows of key matrix; read columns of matrix and store in keyArray 
  Generate TX Message, and send to msgInQ and msgOutQ for RX and TX repsectively 
  */

  static uint16_t prevQuartetStates [] = {0xf, 0xf, 0xf};
  
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS; // convert time in ms to scheduler ticks 
  TickType_t xLastWakeTime = xTaskGetTickCount(); // store the tick count of the last initiation

  // Initialize knobs with lower and upper limit, as well as its knob id:
  KnobDecoder knob2,knob3;
  knob2.setParams(0,2,2); // Waveform: Sawtooth; Triangle; Sinusoid 
  knob3.setParams(10,16,3); // Volume 

  while(1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency); // blocks execution until a certain time has passed since the last time the function was completed
    uint8_t localKeyArray[8];
    uint8_t TX_Message[8];

    // Update keyArray for each row
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    for (int i = 0; i < 8; i++) {
      setRow(i);
      delayMicroseconds(3);
      uint8_t keys = readCols(i);
      keyArray[i] = keys;
      localKeyArray[i] = keys;
    }
    xSemaphoreGive(keyArrayMutex);

    knob2.updateRotationValue(localKeyArray[3]);
    __atomic_store_n(&knob2Rotation,knob2.returnRotationValue(),__ATOMIC_RELAXED);
    knob3.updateRotationValue(localKeyArray[3]);
    __atomic_store_n(&knob3Rotation,knob3.returnRotationValue(),__ATOMIC_RELAXED);

    // Identify keys pressed and store into volatile currentStepSize
    uint8_t localKnob2 = __atomic_load_n(&knob2Rotation, __ATOMIC_RELAXED); // retrieve required octave
    for (uint8_t i=0; i < 3; i++) {
      uint8_t currentQuartetState = localKeyArray[i];

      // for each quartet, check if current key state is same as previous key state 
      if ((currentQuartetState^prevQuartetStates[i])==0){
        continue;
      } else {
        uint8_t prevQuartetState = prevQuartetStates[i]; // recall previous quartet state
        uint8_t changedBits = currentQuartetState^prevQuartetState; // identify changed bits in the quartet
        for (uint8_t j=0; j < 4; j++){ // bitwise comparison for each bit in the quartet
          uint8_t mask = 1<<(3-j);
          if (mask & changedBits) { // this bit has changed
            // this bit has a change
            // TX_Message[1] = localKnob2; // octave
            if (rx_or_tx == 1) {
              TX_Message[1] = defaultRxOctave; // Receiver has octave 4
            } else {
              TX_Message[1] = defaultTxOctave; // Receiver has octave 5
            }
            TX_Message[2] = i*4 + j; // key
            if ((mask & currentQuartetState)>(mask & prevQuartetState)){ // 1 is released 0 is pressed
              // key released
              TX_Message[0] = 'R';
            } else {
              TX_Message[0] = 'P';
            }
          }
        }
      }
      prevQuartetStates[i] = currentQuartetState; // update quartet

      if (rx_or_tx == 2 || rx_or_tx == 0) {//If loopback or tx
        xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
      } else if (rx_or_tx == 1) { // If rx
        xQueueSendFromISR(msgInQ,TX_Message,NULL);
      }
    }
  }
}


// Thread 2
void displayUpdateTask(void * pvParameters) {
  /*
  Visual update on (1) RX or TX or Loopback; (2) Preset octave number; (3) Latest note played; (4) Waveform Type; (5) Volume number
  */

  const TickType_t xFrequency = 100/portTICK_PERIOD_MS; // convert time in ms to scheduler ticks 
  TickType_t xLastWakeTime = xTaskGetTickCount(); // store the tick count of the last initiation
  
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency); // blocks execution until a certain time has passed since the last time the function was completed
    
    // Toggle LED
    digitalToggle(LED_BUILTIN);  
    
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.drawStr(2,10,"GoelSynth");  // write something to the internal memory

    // RX or TX or Loopback
    if (rx_or_tx==1){
      u8g2.drawStr(90,10,"[RX]"); 
    } else if (rx_or_tx==2) {
      u8g2.drawStr(90,10,"[TX]"); 
    } else {
      u8g2.drawStr(90,10,"[Loopback]"); 
    }

    // Note played (on respective keyboard)
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    uint8_t keyPressed = 12; // if no key is pressed, nothing will be displayed
    for (uint8_t i=0; i < 3; i++){
      for (uint8_t j=0; j < 4; j++){ 
        uint8_t mask = 1<<(3-j);
        if (~keyArray[i] & mask){ // checking for key pressed
          keyPressed = i*4+j;
        }
      }
    }
    xSemaphoreGive(keyArrayMutex);
    u8g2.drawStr(2,30,"Note:");
    u8g2.setCursor(34,30);
    u8g2.print(keyNames[keyPressed]);

    // Preset octave number
    u8g2.drawStr(2,20,"Oct:");
    u8g2.setCursor(25,20);
    if (rx_or_tx==2){ // TX
      u8g2.print(defaultTxOctave); 
    } else {
      u8g2.print(defaultRxOctave); 
    }

    // Waveform Type
    if (rx_or_tx!=2){
      u8g2.drawStr(58,20,"|Wave");
      // u8g2.setCursor(84,30);
      uint8_t localKnob2Rotation = __atomic_load_n(&knob2Rotation, __ATOMIC_RELAXED);
      if (localKnob2Rotation==0){
        u8g2.drawStr(64,30,"Saw /|");
      } else if (localKnob2Rotation==1){
        u8g2.drawStr(65,30,"Tri w");
      } else {
        u8g2.drawStr(65,30,"Sin ~");
      }

      // Volume Level
      u8g2.drawStr(95,20,"|Vol|");
      u8g2.setCursor(110,30);
      u8g2.print(knob3Rotation);
    }

    u8g2.sendBuffer();          // transfer internal memory to the display
  }
}

// Thread 3
void decodeTask(void * pvParameters) { 
  /*
  Receives an updated keypress message (press or release) from a queue. 
  Each note has a unique identifier (multipliedID). 
  If message is 'R': find the location of the note using the identifier and set to 0, and decrement the count of notes being played (polyphony).
  If message s 'P': decide whether to play or not (compare num notes being played vs maxNotesStored). if there s space, find empty slot and add to arrays. Increment count.
  Update global array.
  */

  while(1) {
    uint8_t RX_Message_local[8];
    uint8_t localRxTxCounter; 
    uint8_t localRxTxOctaveArray[maxNotesStored] = {};
    uint8_t localRxTxKeyArray[maxNotesStored] = {};
    uint8_t localRxTxMultipliedArray[maxNotesStored] = {};
    
    xQueueReceive(msgInQ, RX_Message_local, portMAX_DELAY);

    // update local with the global arrays
    xSemaphoreTake(RxTxArrayMutex, portMAX_DELAY);
    for (int i=0;i<maxNotesStored;i++){
      localRxTxOctaveArray[i]=globalRxTxOctaveArray[i];
      localRxTxKeyArray[i] = globalRxTxKeyArray[i];
      localRxTxMultipliedArray[i]=globalRxTxMultipliedArray[i];
    }
    xSemaphoreGive(RxTxArrayMutex);

    localRxTxCounter = __atomic_load_n(&globalRxTxCounter, __ATOMIC_RELAXED);
    

    uint8_t multipliedID = (RX_Message_local[1]+1)*(RX_Message_local[2]+1); // unique identifier for each note irrespective of octave
    if ((RX_Message_local[0] == 'P') && (localRxTxCounter<maxNotesStored)) {
      // find location in array that is empty
      for (int i=0;i<maxNotesStored;i++){
        if (localRxTxMultipliedArray[i] == 0) { // found location to add new note
          localRxTxOctaveArray[i] = RX_Message_local[1];
          localRxTxKeyArray[i] = RX_Message_local[2];
          localRxTxMultipliedArray[i] = multipliedID;
          break;
        }
      } 
      localRxTxCounter += 1;
      
      // update global array and variable
      xSemaphoreTake(RxTxArrayMutex, portMAX_DELAY);
      for (int i=0;i<maxNotesStored;i++){
        globalRxTxOctaveArray[i]=localRxTxOctaveArray[i];
        globalRxTxKeyArray[i]=localRxTxKeyArray[i];
        globalRxTxMultipliedArray[i] = localRxTxMultipliedArray[i];
      }
      xSemaphoreGive(RxTxArrayMutex);
      __atomic_store_n(&globalRxTxCounter,localRxTxCounter,__ATOMIC_RELAXED);
    }

    if ((RX_Message_local[0] == 'R') && (localRxTxCounter<=maxNotesStored)) { 
      // finds location of the key to be removed and reset multipliedID to 0 
      for (int i=0;i<maxNotesStored;i++){ 
        if (localRxTxMultipliedArray[i] == multipliedID) {
          localRxTxMultipliedArray[i]=0;
          __atomic_store_n(&testPointCheck,localRxTxMultipliedArray[i],__ATOMIC_RELAXED);
          break;
        }
      } 
      localRxTxCounter-=1; 

      xSemaphoreTake(RxTxArrayMutex, portMAX_DELAY);
      // update global array and variable
      for (int i=0;i<maxNotesStored;i++){
        globalRxTxMultipliedArray[i] = localRxTxMultipliedArray[i];
      }
      xSemaphoreGive(RxTxArrayMutex);
      __atomic_store_n(&globalRxTxCounter,localRxTxCounter,__ATOMIC_RELAXED);
    } 
  }
}

// Thread 5
void generateCurrentStepArrayTask(void * pvParameters){
  /*
  Check the updated key and octave global arrays and convert the information into a global currentStepSizeArr.
  */

  const TickType_t xFrequency = 20/portTICK_PERIOD_MS; // convert time in ms to scheduler ticks 
  TickType_t xLastWakeTime = xTaskGetTickCount(); // store the tick count of the last initiation

  while(1){
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    uint8_t localRxTxOctaveArray[maxNotesStored] = {};
    uint8_t localRxTxKeyArray[maxNotesStored] = {};
    uint8_t localRxTxMultipliedArray[maxNotesStored] = {};

    xSemaphoreTake(RxTxArrayMutex, portMAX_DELAY);
    for (int i=0;i<maxNotesStored;i++){
      localRxTxOctaveArray[i]=globalRxTxOctaveArray[i];
      localRxTxKeyArray[i] = globalRxTxKeyArray[i];
      localRxTxMultipliedArray[i]=globalRxTxMultipliedArray[i];
    }
    xSemaphoreGive(RxTxArrayMutex);
    
    for (int i=0;i<maxNotesStored;i++){
      if (localRxTxMultipliedArray[i]!=0){
        int32_t localStepSize = stepSizes[localRxTxKeyArray[i]]; 
        localStepSize = localStepSize*pow(2,(localRxTxOctaveArray[i]-4));
        __atomic_store_n(&currentStepSizeArr[i],localStepSize,__ATOMIC_RELAXED);
      } else {
        __atomic_store_n(&currentStepSizeArr[i],0,__ATOMIC_RELAXED);
      }
    }
  }
}

void setup() {
  // Initialize the CAN bus
  CAN_Init((rx_or_tx==0)); // true for loopback: rx_or_tx = 0 for loopback, 1 for rx, 2 for tx
  setCANFilter(0x123,0x7ff); //only messages with ID 0x123 will be received; every bit of the ID must match the filter for msg to be accepted
  CAN_RegisterRX_ISR(CAN_RX_ISR); // call ISR whenever CAN msg received -> pass pointer to relevant library function
  CAN_RegisterTX_ISR(CAN_TX_ISR); 
  CAN_Start(); 

  //Initialize queue handler
  msgInQ = xQueueCreate(36,8); // store 36 items -> msg decoding task lower priority as msgs can queue up longer; 8 bytes size for each item
  msgOutQ = xQueueCreate(36,8); // store 36 items -> msg decoding task lower priority as msgs can queue up longer; 8 bytes size for each item

  //Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(3);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  //Initialize buzzer timer
  TIM_TypeDef *Instance = TIM1; 
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT); // function triggered by interrupt 22k times per second
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  
  // Initialize scanKeyTask() THREAD
  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
    scanKeysTask,   // Function that implements task
    "scankeys",     // Text name for the task
    64,             // Stack size in words, not bytes -> to store all local variables of the functions called in the thread
    NULL,           // Param passed into the task
    2,              // Task priority
    &scanKeysHandle //Pointer to store the task handle
  );

  // Initialize displayUpdateTask() THREAD
  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(
    displayUpdateTask,   // Function that implements task
    "displayUpdate",     // Text name for the task
    256,                  // Stack size in words, not bytes -> to store all local variables of the functions called in the thread
    NULL,                // Param passed into the task
    1,                   // Task priority
    &displayUpdateHandle //Pointer to store the task handle
  );

  // Initialize decodeTask() THREAD
  TaskHandle_t decodeHandle = NULL;
  xTaskCreate(
    decodeTask,   // Function that implements task
    "decode",     // Text name for the task
    256,                  // Stack size in words, not bytes -> to store all local variables of the functions called in the thread
    NULL,                // Param passed into the task
    1,                   // Task priority
    &decodeHandle //Pointer to store the task handle
  );

  // Initialize generateCurrentStepArrayTask() THREAD
  TaskHandle_t generateCurrentStepArrayHandle = NULL;
  xTaskCreate(
    generateCurrentStepArrayTask,   // Function that implements task
    "generateCurrentStepArray",     // Text name for the task
    256,                  // Stack size in words, not bytes -> to store all local variables of the functions called in the thread
    NULL,                // Param passed into the task
    1,                   // Task priority
    &generateCurrentStepArrayHandle //Pointer to store the task handle
  );

  // Initialize CAN_TX_Task() THREAD
  TaskHandle_t CAN_TXHandle = NULL;
  xTaskCreate(
    CAN_TX_Task,   // Function that implements task
    "CAN_TX",     // Text name for the task
    256,                  // Stack size in words, not bytes -> to store all local variables of the functions called in the thread
    NULL,                // Param passed into the task
    1,                   // Task priority
    &CAN_TXHandle //Pointer to store the task handle
  );


  // Initialize Mutexes
  keyArrayMutex = xSemaphoreCreateMutex(); 
  RxTxArrayMutex = xSemaphoreCreateMutex(); 

  // Creating Semaphore
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3); 

  vTaskStartScheduler();

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");
}


void loop() {
}
