//Constants
  const uint32_t interval = 100; //Display update interval
  const double fs = 22e3;
  const double K = pow(2,32)/fs;
  const int32_t stepSizes [] = {51076059,54113194,57330934,60740009,64351800,68178357,72232450,76527614,81078184,85899345,91007186,96418755,0};
  const int8_t sineAmplitudeArray [] = {0, 3, 6, 9, 12, 16, 19, 22, 25, 28, 31, 34, 37, 40, 43, 46, 49, 52, 54, 57, 60, 63, 66, 68, 71, 73, 76, 78, 81, 83, 86, 88, 90, 92, 94, 96, 98, 100, 102, 104, 106, 108, 109, 111, 112, 114, 115, 116, 118, 119, 120, 121, 122, 123, 123, 124, 125, 125, 126, 126, 126, 127, 127, 127, 127, 127, 127, 127, 126, 126, 125, 125, 124, 124, 123, 122, 121, 120, 119, 118, 117, 116, 114, 113, 112, 110, 108, 107, 105, 103, 101, 99, 97, 95, 93, 91, 89, 87, 84, 82, 80, 77, 75, 72, 69, 67, 64, 61, 59, 56, 53, 50, 47, 44, 41, 39, 36, 32, 29, 26, 23, 20, 17, 14, 11, 8, 5, 2, -2, -5, -8, -11, -14, -17, -20, -23, -26, -29, -32, -36, -39, -41, -44, -47, -50, -53, -56, -59, -61, -64, -67, -69, -72, -75, -77, -80, -82, -84, -87, -89, -91, -93, -95, -97, -99, -101, -103, -105, -107, -108, -110, -112, -113, -114, -116, -117, -118, -119, -120, -121, -122, -123, -124, -124, -125, -125, -126, -126, -127, -127, -127, -127, -127, -127, -127, -126, -126, -126, -125, -125, -124, -123, -123, -122, -121, -120, -119, -118, -116, -115, -114, -112, -111, -109, -108, -106, -104, -102, -100, -98, -96, -94, -92, -90, -88, -86, -83, -81, -78, -76, -73, -71, -68, -66, -63, -60, -57, -54, -52, -49, -46, -43, -40, -37, -34, -31, -28, -25, -22, -19, -16, -12, -9, -6, -3, 0};
  const char* keyNames[] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B", ""};

  const uint8_t maxNotesStored = 10;
  const int rx_or_tx = 1; // 0 for loopback, 1 for rx, 2 for tx
  const int defaultRxOctave = 5;
  const int defaultTxOctave = defaultRxOctave+1;

//Volatiles
  volatile int32_t currentStepSize; 
  volatile uint8_t keyArray[8];
  volatile uint8_t knob0Rotation;
  volatile uint8_t knob1Rotation=2; // 0: sawtooth; 1: triangle; 2: sinusoid
  volatile uint8_t knob2Rotation; // octave
  volatile uint8_t knob3Rotation; // volume
  volatile uint8_t testPointCheck;
  volatile uint8_t testPointCheck1;
  volatile uint8_t testPointCheck2;
  volatile uint8_t testPointCheck3;
  volatile uint8_t testPointCheck4;


  volatile uint8_t RX_Message[8] = {0}; // store outgoing messages
  
  volatile uint8_t globalRxTxKeyArray[maxNotesStored] = {};
  volatile uint8_t globalRxTxOctaveArray[maxNotesStored] = {};
  volatile char globalRxTxPressArray[maxNotesStored] = {};
  volatile uint8_t globalRxTxMultipliedArray[maxNotesStored] = {}; // multiply (1+octave) and (1+key) to get a unique ID -> 0 if dont play, else play 
  volatile uint8_t globalRxTxCounter = 0; // to keep track the number of elements filled -> must not exceed maxNotesStored
  volatile int32_t currentStepSizeArr[maxNotesStored] = {};

//Queue handler
  QueueHandle_t msgInQ;
  QueueHandle_t msgOutQ;
  
//FreeRTOS mutex -> used by different threads to access mutex obj
  SemaphoreHandle_t keyArrayMutex;
  SemaphoreHandle_t RxTxArrayMutex;
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
