#include <PID_v1.h>

#define Motor1_AlarmOut 52
#define Motor1_SpeedOut 2
#define Motor1_AlarmReset 50
#define Motor1_Direction 48
#define Motor1_Run  46
#define Motor1_Start 44
#define Motor1_Speed 8
#define Motor1_Encoder1 20
#define Motor1_Encoder2 24

#define Motor2_AlarmOut 53
#define Motor2_SpeedOut 3
#define Motor2_AlarmReset 51
#define Motor2_Direction 49
#define Motor2_Run  47
#define Motor2_Start 45
#define Motor2_Speed 9
#define Motor2_Encoder1 21
#define Motor2_Encoder2 25

#define Motor3_AlarmOut 42
#define Motor3_SpeedOut 4
#define Motor3_AlarmReset 40
#define Motor3_Direction 38
#define Motor3_Run  36
#define Motor3_Start 34
#define Motor3_Speed 7
#define Motor3_Encoder1 18
#define Motor3_Encoder2 22

#define Motor4_AlarmOut 43
#define Motor4_SpeedOut 5
#define Motor4_AlarmReset 41
#define Motor4_Direction 39
#define Motor4_Run  37
#define Motor4_Start 35
#define Motor4_Speed 13
#define Motor4_Encoder1 19
#define Motor4_Encoder2 23

int CH1 = 1500;
int CH2 = 1500;
int CH3 = 1500;
int CH5 = 1500;

int MotorMaxSpeed = 255;
int M1=0;     ////speed calculated
int M2=0;     ////speed calculated
int M3=0;     ////speed calculated
int M4=0;     ////speed calculated

unsigned int slotIntervalM1=0;
unsigned int slotIntervalM2=0;
unsigned int slotIntervalM3=0;
unsigned int slotIntervalM4=0;
unsigned int counterM1=0;
unsigned int counterM2=0;
unsigned int counterM3=0;
unsigned int counterM4=0;

float w1=0;
float w2=0;
float w3=0;
float w4=0;

//////////////////////////////////encoder
int encoderM1Pos = 0;         // encoder counter
int encoderM2Pos = 0;
int encoderM3Pos = 0;
int encoderM4Pos = 0;

int n1 = 0;       // encoder state
int n2 = 0;
int n3 = 0;
int n4 = 0;

int Motor1_Encoder1Last = LOW;
int Motor2_Encoder1Last = LOW;
int Motor3_Encoder1Last = LOW;
int Motor4_Encoder1Last = LOW;


////////////////////////////////// filter
const int numReadings = 5;
int readings1[numReadings];      // the readings from the analog input
int readIndex1 = 0;              // the index of the current reading
int total1 = 0;                  // the running total
int CH1av = 0;

int readings2[numReadings];      // the readings from the analog input
int readIndex2 = 0;              // the index of the current reading
int total2 = 0;                  // the running total
int CH2av = 0;

int readings3[numReadings];      // the readings from the analog input
int readIndex3 = 0;              // the index of the current reading
int total3 = 0;                  // the running total
int CH3av = 0;

int readings5[numReadings];      // the readings from the analog input
int readIndex5 = 0;              // the index of the current reading
int total5 = 0;                  // the running total
int CH5av = 0;
/////////////////////////////PID

double M1Speed, M2Speed, M3Speed, M4Speed;                // output speed to motor driver
double w1SetPoint, w2SetPoint, w3SetPoint, w4SetPoint;    // set Point scale from speed calculated
double w1input, w2input, w3input, w4input;                // input scale from w (angle velocity)

double aggKp=4, aggKi=0.2, aggKd=1;
double Kp=10, Ki=0.05, Kd=0.25;


//PID M1PID(&w1input, &M1Speed, &w1SetPoint, Kp, Ki, Kd, P_ON_M, DIRECT);
//PID M2PID(&w2input, &M2Speed, &w2SetPoint, Kp, Ki, Kd, P_ON_M, DIRECT);
//PID M3PID(&w3input, &M3Speed, &w3SetPoint, Kp, Ki, Kd, P_ON_M, DIRECT);
//PID M4PID(&w4input, &M4Speed, &w4SetPoint, Kp, Ki, Kd, P_ON_M, DIRECT);


void setup()
{
  for (int i= 4;i<=13;i++)
  {
    pinMode(i, OUTPUT);
  }
  for (int i= 22;i<=53;i++)
  {
    pinMode(i, OUTPUT);
  }
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);

///////////////////////// filter
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings1[thisReading] = 0;
  }
    for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings2[thisReading] = 0;
  }

  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings3[thisReading] = 0;
  }

/////////////////////////////PID
//  M1PID.SetMode(AUTOMATIC);
//  M2PID.SetMode(AUTOMATIC);
//  M3PID.SetMode(AUTOMATIC);
//  M4PID.SetMode(AUTOMATIC);
//  M1PID.SetTunings(Kp, Ki,Kd);
  
///////////////////////  initial
  digitalWrite(Motor1_Run, HIGH);
  digitalWrite(Motor1_Start, HIGH);
  digitalWrite(Motor2_Run, LOW);
  digitalWrite(Motor2_Start, HIGH);
  digitalWrite(Motor3_Run, LOW);
  digitalWrite(Motor3_Start, HIGH);
  digitalWrite(Motor4_Run, LOW);
  digitalWrite(Motor4_Start, HIGH);
  Serial.begin(115200);

////////////////////////  // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 2000;               // compare match register 16MHz/8/1000Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS11);    // 8 prescaler 
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts
  
///////////////////////////// external interupt 
  attachInterrupt(digitalPinToInterrupt(Motor1_Encoder1), GetTimeIntervalM1, RISING);
  attachInterrupt(digitalPinToInterrupt(Motor2_Encoder1), GetTimeIntervalM2, RISING);
  attachInterrupt(digitalPinToInterrupt(Motor3_Encoder1), GetTimeIntervalM3, RISING);
  attachInterrupt(digitalPinToInterrupt(Motor4_Encoder1), GetTimeIntervalM4, RISING);

  Serial.begin(115200);
}

ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{
  counterM1++;
  counterM2++;
  counterM3++;
  counterM4++;
}

void loop() 
{
  GetInput();
  CalculateMotorSpeed();
  ControlMotorDirection();
  ControlRPM();
  EncoderM1();
  EncoderM2();
  EncoderM3();
  EncoderM4();


//  Serial.print( counterM1 );
//  Serial.print( "    " );
//  Serial.print( counterM2 );
//  Serial.print( "    " );
//  Serial.print( counterM3 );
//  Serial.print( "    " );
//  Serial.println( counterM4 );

}







