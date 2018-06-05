
void FilterCH1()
{
  total1 = total1 - readings1[readIndex1];
  // read from the sensor:
  readings1[readIndex1] = CH1;
  // add the reading to the total:
  total1 = total1 + readings1[readIndex1];
  // advance to the next position in the array:
  readIndex1 = readIndex1 + 1;

  // if we're at the end of the array...
  if (readIndex1 >= numReadings) {
    // ...wrap around to the beginning:
    readIndex1 = 0;
  }

  // calculate the average:
  CH1av = total1 / numReadings;
  delay(1);
}

void FilterCH2()
{
  total2 = total2 - readings2[readIndex2];
  // read from the sensor:
  readings2[readIndex2] = CH2;
  // add the reading to the total:
  total2 = total2 + readings2[readIndex2];
  // advance to the next position in the array:
  readIndex2 = readIndex2 + 1;

  // if we're at the end of the array...
  if (readIndex2 >= numReadings) {
    // ...wrap around to the beginning:
    readIndex2 = 0;
  }

  // calculate the average:
  CH2av = total2 / numReadings;
  delay(1);
}

void FilterCH3()
{
  total3 = total3 - readings3[readIndex3];void EncoderM1(){
  n1 = digitalRead(Motor1_Encoder1);
  if ((Motor1_Encoder1Last == LOW) && (n1 == HIGH)) {
    if (digitalRead(Motor1_Encoder2) == LOW) {
      encoderM1Pos--;
    } else {
      encoderM1Pos++;
    }
  }
  Motor1_Encoder1Last = n1;
  Serial.print( "    " );
  Serial.print( encoderM1Pos );
}

void EncoderM2(){
  n2 = digitalRead(Motor2_Encoder1);
  if ((Motor2_Encoder1Last == LOW) && (n2 == HIGH)) {
    if (digitalRead(Motor2_Encoder2) == LOW) {
      encoderM2Pos--;
    } else {
      encoderM2Pos++;
    }
  }
  Motor2_Encoder1Last = n2;
  Serial.print( "    " );
  Serial.print( encoderM2Pos );
}

void EncoderM3(){
  n3 = digitalRead(Motor3_Encoder1);
  if ((Motor3_Encoder1Last == LOW) && (n3 == HIGH)) {
    if (digitalRead(Motor3_Encoder2) == LOW) {
      encoderM3Pos--;
    } else {
      encoderM3Pos++;
    }
  }
  Motor3_Encoder1Last = n3;
  Serial.print( "    " );
  Serial.print( encoderM3Pos );
}

void EncoderM4(){
  n4 = digitalRead(Motor4_Encoder1);
  if ((Motor4_Encoder1Last == LOW) && (n4 == HIGH)) {
    if (digitalRead(Motor4_Encoder2) == LOW) {
      encoderM4Pos--;
    } else {
      encoderM4Pos++;
    }
  }
  Motor4_Encoder1Last = n4;
  Serial.print( "    " );
  Serial.print( encoderM4Pos );
  Serial.print( "    " );
}
