
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
  total3 = total3 - readings3[readIndex3];void GetInput() {
  CH1 = map(pulseIn(A0, HIGH, 30000), 1000, 2000, -255, 255);
  CH2 = map(pulseIn(A1, HIGH, 30000), 1000, 2000, -255, 255);
  CH3 = map(pulseIn(A2, HIGH, 30000), 1000, 2000, -255, 255);
  CH5 = map(pulseIn(A3, HIGH, 30000), 1000, 2000, -255, 255);

  FilterCH1();
  FilterCH2();
  FilterCH3();

  ///////////Upper and Low Dead band
  if ( CH1av > 256) CH1av = 255;
  if ( CH1av > 350) CH1av = 0;
  if ( CH1av < -350) CH1av = 0;
  if ( CH1av < -256) CH1av = -255;
  if ( CH2av > 256) CH2av = 255;
  if ( CH2av > 350) CH2av = 0;
  if ( CH2av < -350) CH2av = 0;
  if ( CH2av < -256) CH2av = -255;
  if ( CH3av > 256) CH3av = 255;
  if ( CH3av > 350) CH3av = 0;
  if ( CH3av < -350) CH3av = 0;
  if ( CH3av < -256) CH3av = -255;

  ////////// center Dead band
  if ( CH1av <= 10 && CH1av >= -10) {
    CH1av = 0;
  }

  if ( CH2av <= 10 && CH2av >= -10) {
    CH2av = 0;
  }

  if ( CH3av <= 10 && CH3av >= -10) {
    CH3av = 0;
  }
  ////////////////
  if(CH5>0){
    digitalWrite(Motor1_Run, HIGH);
    digitalWrite(Motor2_Run, HIGH);
    digitalWrite(Motor3_Run, HIGH);
    digitalWrite(Motor4_Run, HIGH);
  }
  else {
    digitalWrite(Motor1_Run, LOW);
    digitalWrite(Motor2_Run, LOW);
    digitalWrite(Motor3_Run, LOW);
    digitalWrite(Motor4_Run, LOW);
  }


  
  //  Serial.print( CH1 );
  //  Serial.print( "    " );
  //  Serial.print( CH2 );
  //  Serial.print( "    " );
  //  Serial.print( CH3 );d
  //  Serial.print( "    " );
  //  Serial.print( CH1av );
  //  Serial.print( "    " );
  //  Serial.print( CH2av );
  //  Serial.print( "    " );
  //  Serial.println( CH3av );
}
