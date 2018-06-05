void ControlMotorDirection(){
///////////////////////////////Motor1

  if (M1 >0){
    M1= abs(M1);
    digitalWrite(Motor1_Start, LOW);
    digitalWrite(Motor1_Direction, LOW);    //cw
    analogWrite(nh.subscribe(subCmdLeft2);Motor1_Speed, M1);
    //Serial.print("M1 cw             "); 
  }
  else if (M1 <0){
    M1= abs(M1);
    digitalWrite(Motor1_Start, LOW);
    digitalWrite(Motor1_Direction, HIGH);    //ccw
    analogWrite(Motor1_Speed, M1);
    //Serial.print("M1 ccw            ");
  }
  else{
    //analogWrite(Motor1_Speed,0);
    digitalWrite(Motor1_Start, HIGH);
    //Serial.println(M1);
  }

///////////////////////////////Motor2

  if (M2 >0){
    digitalWrite(Motor2_Start, LOW);
    digitalWrite(Motor2_Direction, LOW);    //cw
    analogWrite(Motor2_Speed, M2);
    //Serial.print("M2 cw             "); 
  }
  else if (M2 <0){
    M2= M2*(-1);
    digitalWrite(Motor2_Start, LOW);
    digitalWrite(Motor2_Direction, HIGH);    //ccw
    analogWrite(Motor2_Speed, M2);
    //Serial.print("M2 ccw            ");
  }
  else{
    analogWrite(Motor2_Speed,0);
    digitalWrite(Motor2_Start, HIGH);
    //Serial.println(M2);
  }

///////////////////////////////Motor3

  if (M3 >0){
    digitalWrite(Motor3_Start, LOW);
    digitalWrite(Motor3_Direction, LOW);    //cw
    analogWrite(Motor3_Speed, M3);
    //Serial.print("M3 cw             "); 
  } 
  else if (M3 <0){
    M3= M3*(-1);
    digitalWrite(Motor3_Start, LOW);
    digitalWrite(Motor3_Direction, HIGH);    //ccw
    analogWrite(Motor3_Speed, M3);
    //Serial.print("M3 ccw            ");
  }
  else{
    analogWrite(Motor3_Speed,0);
    digitalWrite(Motor3_Start, HIGH);
    //Serial.println(M3);
  }

///////////////////////////////Motor4

  if (M4 >0){
    digitalWrite(Motor4_Start, LOW);
    digitalWrite(Motor4_Direction, LOW);    //cw
    analogWrite(Motor4_Speed, M4);
    //Serial.print("M4cw             "); 
  }
  else if (M4 <0){
    M4= M4*(-1);
    digitalWrite(Motor4_Start, LOW);
    digitalWrite(Motor4_Direction, HIGH);    //ccw
    analogWrite(Motor4_Speed, M4);
    //Serial.print("M4 ccw            ");
  }
  else{
    analogWrite(Motor4_Speed,0);
    digitalWrite(Motor4_Start, HIGH);
    //Serial.println(M4);
  }

}

