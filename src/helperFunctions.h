void setRow(uint8_t rowIdx){
  digitalWrite(REN_PIN,LOW);

  digitalWrite(RA2_PIN,LOW); // LOW for first 3 rows
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
    const uint8_t C0_mask=8;
    const uint8_t C1_mask=4; 
    const uint8_t C2_mask=2;
    const uint8_t C3_mask=1;

    uint8_t prevKnobState, rotationLowerLim, rotationUpperLim;
    float prevKnobDirection;
    int8_t localKnobRotation;

    uint8_t knobIdx;

  public:
    uint8_t currentKnobState;
    void setParams(uint8_t lowerLim, uint8_t upperLim, uint8_t knobIdx){
      rotationLowerLim = lowerLim;
      rotationUpperLim = upperLim;
      this->knobIdx = knobIdx;
    };
    
    void updateRotationValue(uint8_t keyValueRaw) {
      uint8_t left_mask, right_mask, left_mask_shift, right_mask_shift;
      float rotationAddition;
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

      currentKnobState = ((keyValueRaw&left_mask)>>left_mask_shift) + ((keyValueRaw & right_mask)>>right_mask_shift)*2;

      if ((prevKnobState==0 && currentKnobState==1) || (prevKnobState==1 && currentKnobState==3) || (prevKnobState==2 && currentKnobState==0) || (prevKnobState==3 && currentKnobState==2) ) {
        rotationAddition = 1;
        prevKnobDirection = 1;
      } else if ((prevKnobState==0 && currentKnobState==2) || (prevKnobState==1 && currentKnobState==0) || (prevKnobState==2 && currentKnobState==3) || (prevKnobState==3 && currentKnobState==1)) {
        rotationAddition = -1;
        prevKnobDirection = -1;
      } else if ((prevKnobState==0 && currentKnobState==3) || (prevKnobState==1 && currentKnobState==2) || (prevKnobState==2 && currentKnobState==1) || (prevKnobState==3 && currentKnobState==0)) {
        rotationAddition = 2*prevKnobDirection; // supposed to ignore -> use the previous sign 
      } else {
        rotationAddition = 0;
      }

      localKnobRotation += rotationAddition;

      if (localKnobRotation > rotationUpperLim) {
        localKnobRotation = rotationUpperLim;
      } else if (localKnobRotation < rotationLowerLim) {
        localKnobRotation = rotationLowerLim;
      }

      prevKnobState = currentKnobState;
    };

    uint8_t returnRotationValue() {
      return localKnobRotation;
    };
};
