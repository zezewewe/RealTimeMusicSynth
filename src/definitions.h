//Constants
  const uint32_t interval = 100; //Display update interval
  const double fs = 22e3;
  const double K = pow(2,32)/fs;
  const int32_t stepSizes [] = {51076059,54113194,57330934,60740009,64351800,68178357,72232450,76527614,81078184,85899345,91007186,96418755,0};
  const int32_t stepSizes2 [] = {102152118, 108226388, 114661868, 121480018, 128703600, 136356714, 144464900, 153055228, 162156368, 171798690, 182014372, 192837510,0};

  const char* keyNames[] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B", ""};

//Volatiles
  volatile int32_t currentStepSize; 
  volatile uint8_t keyArray[7];
  volatile int8_t knob0Rotation;
  volatile int8_t knob1Rotation;
  volatile uint8_t knob2Rotation;
  volatile uint8_t knob3Rotation;
  volatile uint8_t testPointCheck;

  uint8_t RX_Message[8] = {0}; // store outgoing messages

//Queue handler
  QueueHandle_t msgInQ;
  QueueHandle_t msgOutQ;
  
//FreeRTOS mutex -> used by different threads to access mutex obj
  SemaphoreHandle_t keyArrayMutex;
  SemaphoreHandle_t RX_MessageMutex;
  SemaphoreHandle_t CAN_TX_Semaphore;

//Pin definitions
  //Row select and enable
  const int RA0_PIN = D3;
  const int RA1_PIN = D6;
  const int RA2_PIN = D12;
  const int REN_PIN = A5;

  //Matrix input and output
  const int C0_PIN = A2;
  const int C1_PIN = D9;
  const int C2_PIN = A6;
  const int C3_PIN = D1;
  const int OUT_PIN = D11;

  //Audio analogue out
  const int OUTL_PIN = A4;
  const int OUTR_PIN = A3;

  //Joystick analogue in
  const int JOYY_PIN = A0;
  const int JOYX_PIN = A1;

  //Output multiplexer bits
  const int DEN_BIT = 3;
  const int DRST_BIT = 4;
  const int HKOW_BIT = 5;
  const int HKOE_BIT = 6;

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
      digitalWrite(REN_PIN,LOW);
      digitalWrite(RA0_PIN, bitIdx & 0x01);
      digitalWrite(RA1_PIN, bitIdx & 0x02);
      digitalWrite(RA2_PIN, bitIdx & 0x04);
      digitalWrite(OUT_PIN,value);
      digitalWrite(REN_PIN,HIGH);
      delayMicroseconds(2);
      digitalWrite(REN_PIN,LOW);
}
