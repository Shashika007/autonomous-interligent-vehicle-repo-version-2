void CalculateMotorSpeed(){
  M1 =  CH1av + CH2av + CH3av;
  M2 =  CH1av - CH2av + CH3av;
  M3 = -CH1av - CH2av + CH3av;
  M4 = -CH1av + CH2av + CH3av;
  
//////////////////////////////////////// set working window
  if ( M1>=MotorMaxSpeed) M1 = MotorMaxSpeed;
  if ( M1<=-MotorMaxSpeed) M1 = -MotorMaxSpeed;
  if ( M2>=MotorMaxSpeed) M2 = MotorMaxSpeed;
  if ( M2<=-MotorMaxSpeed) M2 = -MotorMaxSpeed;
  if ( M3>=MotorMaxSpeed) M3 = MotorMaxSpeed;
  if ( M3<=-MotorMaxSpeed) M3 = -MotorMaxSpeed;
  if ( M4>=MotorMaxSpeed) M4 = MotorMaxSpeed;
  if ( M4<=-MotorMaxSpeed) M4 = -MotorMaxSpeed;
  
/////////////////////////////////////// scale angle velocity SetPoint(scale up)
  w1SetPoint =  abs(2*M1);
  w2SetPoint =  abs(2*M2);
  w3SetPoint =  abs(2*M3);
  w4SetPoint =  abs(2*M4);

//  Serial.print( w1SetPoint,2 );
//  Serial.print( "    " );
//  Serial.print( w2SetPoint,2 );
//  Serial.print( "    " );
//  Serial.print( w3SetPoint,2 );
//  Serial.print( "    " );  Serial.print( M1 );
//  Serial.print( "    " );
//  Serial.print( M2 );
//  Serial.print( "    " );
//  Serial.print( M3 );
//  Serial.print( "    " );
//  Serial.print( M4 );
//  Serial.print( "    " );
//  Serial.println( w4SetPoint,2 );

//
}

