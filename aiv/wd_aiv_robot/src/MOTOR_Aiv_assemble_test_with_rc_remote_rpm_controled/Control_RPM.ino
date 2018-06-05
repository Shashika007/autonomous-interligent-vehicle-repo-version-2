void GetTimeIntervalM1(){
  slotIntervalM1 = counterM1;
  counterM1 = 0;
}

void GetTimeIntervalM2(){
  slotIntervalM2 = counterM2;
  counterM2 = 0;
}

void GetTimeIntervalM3(){
  slotIntervalM3 = counterM3;
  counterM3 = 0;
}

void GetTimeIntervalM4(){
  slotIntervalM4 = counterM4;
  counterM4 = 0;
}

void RPMM1(){
  w1=1000.0/(90.0*slotIntervalM1);    ////angle velocity of motor 1 final output
}

void RPMM2(){
  w2=1000.0/(90.0*slotIntervalM2);    ////angle velocity of motor 1 final output
}

void RPMM3(){
  w3=1000.0/(90.0*slotIntervalM3);    ////angle velocity of motor 1 final output
}

void RPMM4(){
  w4=1000.0/(90.0*slotIntervalM4);    ////angle velocity of motor 1 final output
}


void ControlRPM(){

  RPMM1();
  RPMM2();
  RPMM3();
  RPMM4();
////////////////////////////// ensure W is 0 while no wheel rotation
  if(abs(M1)<=2) w1=0;
  if(abs(M2)<=2) w2=0;
  if(abs(M3)<=2) w3=0;
  if(abs(M4)<=2) w4=0;
///////////////////////////// scale angle velocity read from encoder disk(scale up)
  w1= 1000.0*w1;
  w2= 1000.0*w2;
  w3= 1000.0*w3;
  w4= 1000.0*w4;
////////////////////

//  M1PID.Compute();
//  M2PID.Compute();
//  M3PID.Compute();
//  M4PID.Compute();
//  analogWrite(Motor1_Speed, M1Speed);
//  analogWrite(Motor2_Speed, M2Speed);
//  analogWrite(Motor3_Speed, M3Speed);
//  analogWrite(Motor4_Speed, M4Speed);


  Serial.print( w1,6 );
  Serial.print( "    " );
  Serial.print( w2,6 );
  Serial.print( "    " );
  Serial.print( w3,6 );
  Serial.print( "    " );
  Serial.println( w4,6 );

  

//  Serial.print( slotIntervalM1 );
//  Serial.print( "    " );
//  Serial.print( slotIntervalM2 );
//  Serial.print( "    " );
//  Serial.print( slotIntervalM3 );
//  Serial.print( "    " );
//  Serial.println( slotIntervalM4 );




  
}

