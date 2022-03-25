void setRow(uint8_t rowIdx){
  digitalWrite(REN_PIN,LOW);

  digitalWrite(RA2_PIN, rowIdx & (1<<2)); 
  digitalWrite(RA1_PIN, rowIdx & (1<<1));
  digitalWrite(RA0_PIN, rowIdx & (1<<0));
  
  digitalWrite(REN_PIN,HIGH);
}

uint8_t readCols(uint8_t rowIdx) {
  uint8_t C0 = digitalRead(C0_PIN);
  uint8_t C1 = digitalRead(C1_PIN);
  uint8_t C2 = digitalRead(C2_PIN);
  uint8_t C3 = digitalRead(C3_PIN);
  return C0<<3|C1<<2|C2<<1|C3<<0; // concatenate four integers together
}




class KnobDecoder {
    const uint8_t C0_mask=8; // Column 0 mask = 1000
    const uint8_t C1_mask=4; // Column 1 mask = 0100
    const uint8_t C2_mask=2; // Column 2 mask = 0010
    const uint8_t C3_mask=1; // Column 3 mask = 0001

    uint8_t prevKnobState, rotationLowerLim, rotationUpperLim;
    float prevKnobDirection; // prevKnobDrection indicates direction (CW/CCW) of rotation from t-2 to t-1
    int8_t localKnobRotation;

    uint8_t knobIdx;

  public:
    uint8_t currentKnobState;

    // Initialize knob by defining limits and knob ID
    // Knobs 2-3 use row 3 while knobs 0-1 use row 4
    void setParams(uint8_t lowerLim, uint8_t upperLim, uint8_t knobIdx){
      rotationLowerLim = lowerLim;
      rotationUpperLim = upperLim;
      this->knobIdx = knobIdx;
    };
    
    // Update rotation value by comparing pair of row values at time t to t-1
    void updateRotationValue(uint8_t keyValueRaw) {
      uint8_t left_mask, right_mask, left_mask_shift, right_mask_shift;
      float rotationAddition; // number units to rotate

      // knob 1 and 3 are on cols 0 and 1, have to be masked and right shifted
      if (knobIdx==3 || knobIdx==1){
        left_mask = C0_mask;
        left_mask_shift = 3;
        right_mask = C1_mask;
        right_mask_shift = 2;
      } else {
        left_mask = C2_mask;
        left_mask_shift = 1;
        right_mask = C3_mask;
        right_mask_shift = 0;
      }

      // calculate current knob state
      currentKnobState = ((keyValueRaw&left_mask)>>left_mask_shift) + ((keyValueRaw & right_mask)>>right_mask_shift)*2;

      // calculate rotation based on previous and current state positions
      if ((prevKnobState==0 && currentKnobState==1) || (prevKnobState==1 && currentKnobState==3) || (prevKnobState==2 && currentKnobState==0) || (prevKnobState==3 && currentKnobState==2) ) {
        rotationAddition = 1;
        prevKnobDirection = 1;
      } else if ((prevKnobState==0 && currentKnobState==2) || (prevKnobState==1 && currentKnobState==0) || (prevKnobState==2 && currentKnobState==3) || (prevKnobState==3 && currentKnobState==1)) {
        rotationAddition = -1;
        prevKnobDirection = -1;
      } else if ((prevKnobState==0 && currentKnobState==3) || (prevKnobState==1 && currentKnobState==2) || (prevKnobState==2 && currentKnobState==1) || (prevKnobState==3 && currentKnobState==0)) {
        rotationAddition = 2*prevKnobDirection; // use the previous direction
      } else {
        rotationAddition = 0;
      }

      localKnobRotation += rotationAddition;

      // Set a limit to the rotations for each knob
      if (localKnobRotation > rotationUpperLim) {
        localKnobRotation = rotationUpperLim;
      } else if (localKnobRotation < rotationLowerLim) {
        localKnobRotation = rotationLowerLim;
      }

      // save current knob state for next
      prevKnobState = currentKnobState;
    };

    uint8_t returnRotationValue() {
      return localKnobRotation;
      // return currentKnobState;
    };
};

void addToKeyArray(uint8_t receivedMessageArr[]){
  rxtxPressArray[rxtxKeyArray_idx] = receivedMessageArr[0];
  rxtxOctaveArray[rxtxKeyArray_idx] = receivedMessageArr[1];
  rxtxKeyArray[rxtxKeyArray_idx] = receivedMessageArr[2];
  rxtxKeyArray_idx += 1;
}

//Pop the press and return step
char popFromPressArray()
{
  return rxtxPressArray[rxtxKeyArray_idx];
}

//Pop the octave and return step
uint8_t popFromOctaveArray()
{
  return rxtxOctaveArray[rxtxKeyArray_idx];
}

//Pop the key and return step AND decrement
uint8_t popFromKeyArray()
{
  uint8_t key = rxtxKeyArray[rxtxKeyArray_idx];
  rxtxKeyArray_idx -= 1;
  return key;
}
