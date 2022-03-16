#include <Arduino.h>
#include <U8g2lib.h>
#include <cmath>
#include <STM32FreeRTOS.h>
#include <algorithm>
#include <definitions.h>
#include <helperFunctions.h>
#include <ES_CAN.h>
#include <waveformFunctions.h>



// Interrupt 1
void sampleISR() {
  static int32_t phaseAcc = 0; // static local variable - value stored between successive calls
  phaseAcc += currentStepSize; 

  // if (knob2Rotation == 0){
    int32_t Vout = phaseAcc >> 24;
    Vout = Vout >> (8 - knob3Rotation/2);
  // }

  int32_t Vout = phaseAcc >> 24;
  Vout = Vout >> (8 - knob3Rotation/2);

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
  static uint16_t prevKeyStates [] = {0xf, 0xf, 0xf};
  
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS; // convert time in ms to scheduler ticks 
  TickType_t xLastWakeTime = xTaskGetTickCount(); // store the tick count of the last initiation

  uint8_t TX_Message[8] = {0};

  KnobDecoder knob0,knob1,knob2,knob3;
  knob0.setParams(3,7,0);  
  knob1.setParams(0,2,1); // Waveform: Sawtooth; Triangle; Sinusoid  
  knob2.setParams(4,7,2); // Octave
  knob3.setParams(0,16,3); // Volume 

  while(1) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency ); // blocks execution until a certain time has passed since the last time the function was completed
    uint8_t keyIdx=12;

    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    // Store pressed keys into keyArray
    for (uint8_t i = 0; i < 5; i++) {
      setRow(i);
      delayMicroseconds(3);
      uint8_t keys = readCols(i);
      keyArray[i] = keys;
    }

    knob1.updateRotationValue(keyArray[4]);
    __atomic_store_n(&knob1Rotation,knob1.returnRotationValue(),__ATOMIC_RELAXED);

    knob2.updateRotationValue(keyArray[3]);
    __atomic_store_n(&knob2Rotation,knob2.returnRotationValue(),__ATOMIC_RELAXED);



    // Identify keys pressed and store into volatile currentStepSize
    for (uint8_t i=0; i < 3; i++) {
      uint8_t currentKeyState = keyArray[i];

      // find out the key being pressed and set keyIdx
      // if (keysTmp == 0xf) { // if keysTmp is 15, none in this row is pressed
      //   continue;
      // }
      for (uint8_t j=0; j < 4; j++){ // else find out what is pressed
        uint8_t mask = 1<<(3-j);
        bool isPressed = currentKeyState & mask; 
        if (!isPressed) {
          keyIdx = i*4 + j;
        }
      }

      // check if there is any difference in state
      if ((currentKeyState^prevKeyStates[i])==0){
        continue;
      } else {
        uint8_t prevKeyState = prevKeyStates[i];
        uint8_t changedBits = currentKeyState^prevKeyState;
        for (uint8_t j=0; j < 4; j++){ // do bitwise 
          uint8_t mask = 1<<(3-j);
          if (mask & changedBits) {
            // this bit has a change
            TX_Message[1] = knob2.returnRotationValue();
            TX_Message[2] = i*4 + j;
            if ((mask & currentKeyState)>>(mask & prevKeyState)){
              // key released
              TX_Message[0] = 'R';
              
            } else {
              TX_Message[0] = 'P';
            }
            // CAN_TX(0x123,TX_Message);
            xQueueSend( msgOutQ, TX_Message, portMAX_DELAY );
          }
        }
      }
      prevKeyStates[i] = currentKeyState;
    }
    // __atomic_store_n(&currentStepSize,stepSizes[keyIdx],__ATOMIC_RELAXED);

    knob3.updateRotationValue(keyArray[3]);
    __atomic_store_n(&knob3Rotation,knob3.returnRotationValue(),__ATOMIC_RELAXED);
    xSemaphoreGive(keyArrayMutex);
  }
}

// Thread 2
void displayUpdateTask(void * pvParameters) {
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS; // convert time in ms to scheduler ticks 
  TickType_t xLastWakeTime = xTaskGetTickCount(); // store the tick count of the last initiation
  
  // uint32_t ID;
  // uint8_t RX_Message[8]={0};

  while(1) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency ); // blocks execution until a certain time has passed since the last time the function was completed
    
    // while (CAN_CheckRXLevel()) 
    //   CAN_RX(ID,RX_Message);
    uint8_t RX_Message_local[8] = {0};
    xSemaphoreTake(RX_MessageMutex, portMAX_DELAY);
    RX_Message_local[0]=RX_Message[0];
    RX_Message_local[1]=RX_Message[1];
    RX_Message_local[2]=RX_Message[2];
    xSemaphoreGive(RX_MessageMutex);


    //Toggle LED
    digitalToggle(LED_BUILTIN);  
    
    //Update display
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.drawStr(2,10,"PIANO!");  // write something to the internal memory
    // u8g2.setCursor(2,20);
    // u8g2.print(rotationVar);

    u8g2.setCursor(2,20);
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    u8g2.print(keyArray[0]<<8|keyArray[1]<<4|keyArray[2]<<0, BIN);
    xSemaphoreGive(keyArrayMutex);

    u8g2.setCursor(2,30);
    u8g2.print(knob2Rotation);

    u8g2.setCursor(66,30);
    u8g2.print((char) RX_Message_local[0]);
    u8g2.print(RX_Message_local[1]);
    u8g2.print(RX_Message_local[2]);

    u8g2.sendBuffer();          // transfer internal memory to the display


  }
}

// Thread 3
void decodeTask(void * pvParameters) { 
  while(1) {
    uint8_t RX_Message_local[8] = {0};
    int32_t localStepSize;
    xQueueReceive(msgInQ, RX_Message_local, portMAX_DELAY);

    xSemaphoreTake(RX_MessageMutex, portMAX_DELAY);
    RX_Message[0]=RX_Message_local[0];
    RX_Message[1]=RX_Message_local[1];
    RX_Message[2]=RX_Message_local[2];
    xSemaphoreGive(RX_MessageMutex);


    if (RX_Message_local[0]=='R') {
      localStepSize = 0;
    } else if (RX_Message[0]=='P'){
      localStepSize = stepSizes[RX_Message[2]];
      localStepSize = (localStepSize << (RX_Message[1]-4)); // scale step size by appropriate octave for correct freq
    }

    // currentStepSize = localStepSize;
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    __atomic_store_n(&currentStepSize,localStepSize,__ATOMIC_RELAXED);
    xSemaphoreGive(keyArrayMutex);



  }
}


void setup() {
  // put your setup code here, to run once:

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
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");

  //Initialize timer to trigger the INTERRUPT that will call sampleISR()
  TIM_TypeDef *Instance = TIM1; 
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT); // function triggered by iinterrupt 22k times per second
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  
  // Initialize scanKeyTask() THREAD
  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
    scanKeysTask,   // Function that implements task
    "scankeys",     // Text name for the task
    64,             // Stack size in words, not bytes -> to store all local variables of the functions called in the thread
    NULL,           // Param passed into the task
    4,              // Task priority
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
    128,                  // Stack size in words, not bytes -> to store all local variables of the functions called in the thread
    NULL,                // Param passed into the task
    2,                   // Task priority
    &decodeHandle //Pointer to store the task handle
  );

  // Initialize CAN_TX_Task() THREAD
  TaskHandle_t CAN_TXHandle = NULL;
  xTaskCreate(
    CAN_TX_Task,   // Function that implements task
    "CAN_TX",     // Text name for the task
    128,                  // Stack size in words, not bytes -> to store all local variables of the functions called in the thread
    NULL,                // Param passed into the task
    3,                   // Task priority
    &CAN_TXHandle //Pointer to store the task handle
  );



  // Creating Mutex
  keyArrayMutex = xSemaphoreCreateMutex(); 
  RX_MessageMutex = xSemaphoreCreateMutex(); 

  // Creating Semaphore
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3); 


  // Initialize the CAN bus
  CAN_Init(true); // receive and ack own messages
  setCANFilter(0x123,0x7ff); //only messages with ID 0x123 will be received; every bit of the ID must match the filter for msg to be accepted
  CAN_RegisterRX_ISR(CAN_RX_ISR); // call ISR whenever CAN msg received -> pass pointer to relevant library function
  CAN_RegisterTX_ISR(CAN_TX_ISR); 
  CAN_Start(); 



  //Initialize queue handler
  msgInQ = xQueueCreate(36,8); // store 36 items -> msg decoding task lower priority as msgs can queue up longer; 8 bytes size for each item
  msgOutQ = xQueueCreate(36,8); // store 36 items -> msg decoding task lower priority as msgs can queue up longer; 8 bytes size for each item

  vTaskStartScheduler();
}


void loop() {
}