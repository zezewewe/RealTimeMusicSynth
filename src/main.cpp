#include <Arduino.h>
#include <U8g2lib.h>
#include <cmath>
#include <STM32FreeRTOS.h>
#include <algorithm>
#include <definitions.h>
#include <helperFunctions.h>
#include <ES_CAN.h>
#include <waveformFunctions.h>

#define monophony 1
//#define chords 1
// #define polyphony 1 // not implemented

int rx_or_tx = 0; // 0 for loopback, 1 for rx, 2 for tx

// Interrupt 1: 
void sampleISR() {
  // static int32_t phaseAcc = 0; // static local variable - value stored between successive calls
  int32_t Vout;
  uint8_t localKnob1 = __atomic_load_n(&knob1Rotation, __ATOMIC_RELAXED); // retrieve required waveform
  uint8_t localKnob3 = __atomic_load_n(&knob3Rotation, __ATOMIC_RELAXED); // retrieve required volume


  // init phaseAcc and currentPhase tables
  #if monophony 
    static int32_t phaseAccChordTable[1];
    static int32_t currentPhaseChordTable[1];

    phaseAccChordTable[0] += currentStepSize; 
    currentPhaseChordTable[0] = phaseAccChordTable[0]>>24;

    if (localKnob1==0){ // sawtooth
      Vout = currentPhaseChordTable[0];
    } else if (localKnob1==1) { // triangle
      if (currentPhaseChordTable[0]<= 0) { 
        Vout = 128+2*currentPhaseChordTable[0];
      } else {
        Vout = 127-2*currentPhaseChordTable[0];
      } 
    } else if (localKnob1==2) { // sinusoid
      Vout = sineAmplitudeArray[currentPhaseChordTable[0]+128];
    }

  #elif chords
    static int32_t phaseAccChordTable[3];
    static int32_t currentPhaseChordTable[3];

    phaseAccChordTable[0] += currentStepSize; 
    currentPhaseChordTable[0] = phaseAccChordTable[0]>>24;
    for (int i=1;i<3;i++){
      phaseAccChordTable[i] += (int32_t)(currentStepSize*pow(2,(i*3.0+1.0)/12.0));
      currentPhaseChordTable[i] = (phaseAccChordTable[i])>>24;
    }
    
    if (localKnob1==0){ // sawtooth
      Vout = 0;
      for (int i=0;i<3;i++){
        Vout += currentPhaseChordTable[i];
      }
    } else if (localKnob1==1) { // triangle
      for (int i=0;i<3;i++){
        if (currentPhaseChordTable[i]<= 0) { 
          Vout += 128+2*currentPhaseChordTable[i];
        } else {
          Vout += 127-2*currentPhaseChordTable[i];
        } 
      }
    } else if (localKnob1==2) { // sinusoid
      for (int i=0;i<3;i++){
        Vout += sineAmplitudeArray[currentPhaseChordTable[i]+128];
      }
    }
  #else
    static int32_t phaseAccChordTable[5];
    static int32_t currentPhaseChordTable[5]; 
  #endif

  // uint8_t localCurrentStepSize = __atomic_load_n(&currentStepSize, __ATOMIC_RELAXED); // retrieve required waveform

  
  // if (localKnob1==0){ // sawtooth
  //   Vout = currentPhase + currentPhase2 + currentPhase3;
  // } else if (localKnob1==1) { // triangle
  //   if (currentPhase<= 0) { 
  //     Vout = 128+2*currentPhase;
  //   } else {
  //     Vout = 127-2*currentPhase;
  //   } 
  // } else if (localKnob1==2) { // sinusoid
  //   Vout = sineAmplitudeArray[currentPhase+128] + sineAmplitudeArray[currentPhase2+128] + sineAmplitudeArray[currentPhase3+128];
  // }

  // Volume control
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
  // loop through rows of key matrix; // read columns of matrix and store in keyArray; // update currentStepSize
  static uint16_t prevQuartetStates [] = {0xf, 0xf, 0xf};
  
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS; // convert time in ms to scheduler ticks 
  TickType_t xLastWakeTime = xTaskGetTickCount(); // store the tick count of the last initiation

  // uint8_t TX_Message[8] = {0};

  // Initialize knobs with lower and upper limit, as well as its knob id:
  KnobDecoder knob0,knob1,knob2,knob3;
  knob0.setParams(3,7,0);  
  knob1.setParams(0,2,1); // Waveform: Sawtooth; Triangle; Sinusoid  
  knob2.setParams(2,8,2); // Octave
  knob3.setParams(0,16,3); // Volume 

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

    // Update rotation values for all four knobs: 
    // knob0.updateRotationValue(localKeyArray[4]);
    // knob0Rotation = knob0.returnRotationValue();

    // __atomic_store_n(&knob0Rotation,knob0.returnRotationValue(),__ATOMIC_RELAXED);
    // knob1.updateRotationValue(localKeyArray[4]);
    // // knob1Rotation = knob1.returnRotationValue();
    // __atomic_store_n(&knob1Rotation,knob1.returnRotationValue(),__ATOMIC_RELAXED);

    knob2.updateRotationValue(localKeyArray[3]);
    __atomic_store_n(&knob2Rotation,knob2.returnRotationValue(),__ATOMIC_RELAXED);
    knob3.updateRotationValue(localKeyArray[3]);
    __atomic_store_n(&knob3Rotation,knob3.returnRotationValue(),__ATOMIC_RELAXED);

    // xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    // knob2Rotation = knob2.returnRotationValue();
    // knob3Rotation = knob3.returnRotationValue();
    // xSemaphoreGive(keyArrayMutex);

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
            TX_Message[1] = localKnob2; // octave
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
        // addToKeyArray(TX_Message);
      }
    }
  }
}


// Thread 2
void displayUpdateTask(void * pvParameters) {
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS; // convert time in ms to scheduler ticks 
  TickType_t xLastWakeTime = xTaskGetTickCount(); // store the tick count of the last initiation
  
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency); // blocks execution until a certain time has passed since the last time the function was completed
    
    // Toggle LED
    digitalToggle(LED_BUILTIN);  
    
    //Update display
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.drawStr(2,10,"PIANO!");  // write something to the internal memory

    u8g2.setCursor(2,20);
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    u8g2.print(keyArray[0]<<8|keyArray[1]<<4|keyArray[2]<<0, BIN);
    xSemaphoreGive(keyArrayMutex);

    u8g2.setCursor(2,30);
    u8g2.print(knob0Rotation);
    u8g2.setCursor(8,30);
    u8g2.print(knob1Rotation);
    u8g2.setCursor(14,30);
    u8g2.print(knob2Rotation);
    u8g2.setCursor(20,30);
    u8g2.print(knob3Rotation);

    uint8_t RX_Message_local[8];
    xSemaphoreTake(RX_MessageMutex, portMAX_DELAY);
    for (int i=0;i<8;i++){
      RX_Message_local[i]=RX_Message[i];
    }
    xSemaphoreGive(RX_MessageMutex);

    u8g2.setCursor(66,30);
    u8g2.print((char) output);
    // u8g2.print(testPointCheck);
    u8g2.print((char) RX_Message_local[0]);
    u8g2.print(globalRxTxCounter);
    // u8g2.print(testPointCheck);

    u8g2.sendBuffer();          // transfer internal memory to the display
  }
}

// Thread 3
void decodeTask(void * pvParameters) { 
  while(1) {
    uint8_t RX_Message_local[8];
    uint8_t localRxTxIdx, localRxTxCounter;
    char localRxTxPressArray[maxNotesStored] = {};
    uint8_t localRxTxOctaveArray[maxNotesStored] = {};
    uint8_t localRxTxKeyArray[maxNotesStored] = {};
    uint8_t localRxTxMultipliedArray[maxNotesStored] = {};
    
    xQueueReceive(msgInQ, RX_Message_local, portMAX_DELAY);

    // update RX_Message for display
    xSemaphoreTake(RX_MessageMutex, portMAX_DELAY);
    for (int i=0;i<8;i++){
      RX_Message[i]=RX_Message_local[i];
    }
    xSemaphoreGive(RX_MessageMutex);

    // update local with the global arrays
    xSemaphoreTake(RxTxArrayMutex, portMAX_DELAY);
    for (int i=0;i<maxNotesStored;i++){
      localRxTxPressArray[i]=globalRxTxPressArray[i];
      localRxTxOctaveArray[i]=globalRxTxOctaveArray[i];
      localRxTxMultipliedArray[i]=globalRxTxMultipliedArray[i];
    }
    xSemaphoreGive(RxTxArrayMutex);

    localRxTxIdx = __atomic_load_n(&globalRxTxidx, __ATOMIC_RELAXED); // retrieve required octave
    localRxTxCounter = __atomic_load_n(&globalRxTxCounter, __ATOMIC_RELAXED);
    
    // update local arrays based on whether there is enough space
    // if 'R', key was previously pressed -> find its index and unpress it. reduce the count
    // else if 'P', if not enough space, then don't need add anything to array, 
    //              else if enough space, find index of a empty location and add to array, increment count, and update global array

    if (RX_Message_local[0] == 'R') { // finds location of the key to be removed and resets it 
      uint8_t multipliedID = RX_Message_local[1]*RX_Message_local[2];
      for (int i=0;i<maxNotesStored;i++){
        if (localRxTxMultipliedArray[i] == multipliedID) {
          __atomic_store_n(&output,RX_Message_local[0],__ATOMIC_RELAXED);
          // locationIdx = i; // the index of the array to release is at locationIdx
          localRxTxMultipliedArray[i]=0; // reset to 0 the location with the pressed note
          break;
        }
      } 
      localRxTxCounter-=1; // error
      for (int i=0;i<maxNotesStored;i++){
        globalRxTxMultipliedArray[i] = localRxTxMultipliedArray[i];
      }

      __atomic_store_n(&globalRxTxCounter,localRxTxCounter,__ATOMIC_RELAXED);

    } 
    if ((RX_Message_local[0] == 'P') && (localRxTxCounter<maxNotesStored)) {
      for (int i=0;i<maxNotesStored;i++){
        if (localRxTxMultipliedArray[i] == 0) { // found location to add new note
          __atomic_store_n(&output,RX_Message_local[0],__ATOMIC_RELAXED);
          localRxTxOctaveArray[i] = RX_Message_local[1];
          localRxTxKeyArray[i] = RX_Message_local[2];
          localRxTxMultipliedArray[i] = RX_Message_local[1]*RX_Message_local[2];
          break;
        }
      } 
      localRxTxCounter += 1;

      xSemaphoreTake(RxTxArrayMutex, portMAX_DELAY);
      for (int i=0;i<maxNotesStored;i++){
        globalRxTxPressArray[i]=localRxTxPressArray[i]; // eventually do not even need this press array
        globalRxTxOctaveArray[i]=localRxTxOctaveArray[i];
        globalRxTxKeyArray[i]=localRxTxKeyArray[i];
        globalRxTxMultipliedArray[i] = localRxTxMultipliedArray[i];
      }
      xSemaphoreGive(RxTxArrayMutex);
      __atomic_store_n(&globalRxTxCounter,localRxTxCounter,__ATOMIC_RELAXED);
    }

    // debugging code by playing stuff
    
    uint8_t octave_local = localRxTxOctaveArray[0];
    uint8_t key_local = localRxTxKeyArray[0]; //decrements idx as well
    
    if (output=='R') {
    __atomic_store_n(&currentStepSize,0,__ATOMIC_RELAXED);
    }
    else if (output=='P'){
      int32_t localStepSize = stepSizes[key_local]; // CHECK IF NEED ATOMIC LOAD
      localStepSize = localStepSize*pow(2,(octave_local-4));
      __atomic_store_n(&currentStepSize,localStepSize,__ATOMIC_RELAXED); // scale step size by appropriate octave for correct freq
    }
  }
}


void setup() {
  // put your setup code here, to run once:

  // Initialize the CAN bus
  CAN_Init(~rx_or_tx); // true for loopback. rx_or_tx = 0 is for loopback, 1 is rx, 2 is tx
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


  // Initialize Mutex
  keyArrayMutex = xSemaphoreCreateMutex(); 
  RX_MessageMutex = xSemaphoreCreateMutex(); 
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
