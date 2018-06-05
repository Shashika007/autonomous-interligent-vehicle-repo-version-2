
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
  total3 = total3 - readings3[readIndex3];
  // read from the sensor:
  readings3[readIndex3] = CH3;
  // add the reading to the total:
  total3 = total3 + readings3[readIndex3];
  // advance to the next position in the array:
  readIndex3 = readIndex3 + 1;

  // if we're at the end of the array...
  if (readIndex3 >= numReadings) {
    // ...wrap around to the beginning:
    readIndex3 = 0;
  }

  // calculate the average:
  CH3av = total3 / numReadings;
  delay(1);
}


