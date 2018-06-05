#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <TimerThree.h>

#define LOOP_TIME 200000  

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

unsigned int counter_right1 = 0; // counter
unsigned int counter_left2=0;
unsigned int counter_left3=0;
unsigned int counter_right4 = 0;

int n1 = 0;       // encoder state
int n2 = 0;
int n3 = 0;
int n4 = 0;

int Motor1_Encoder1Last = LOW;
int Motor2_Encoder1Last = LOW;
int Motor3_Encoder1Last = LOW;
int Motor4_Encoder1Last = LOW;

float radius = 0.077;
float pi = 3.1415;

ros::NodeHandle  nh;

std_msgs::Float32 right_wheel1_vel;
ros::Publisher right_wheel1_vel_pub("/right_wheel1_velocity", &right_wheel1_vel);

std_msgs::Float32 left_wheel2_vel;
ros::Publisher left_wheel2_vel_pub("/left_wheel2_velocity", &left_wheel2_vel);

std_msgs::Float32 left_wheel3_vel;
ros::Publisher left_wheel3_vel_pub("/left_wheel3_velocity", &left_wheel3_vel);

std_msgs::Float32 right_wheel4_vel;
ros::Publisher right_wheel4_vel_pub("/right_wheel4_velocity", &right_wheel4_vel);

void docount_right1()  // counts from the speed sensor
{
  counter_right1++;  // increase +1 the counter value
} 

void docount_left2()  // counts from the speed sensor
{
  counter_left2++;  // increase +1 the counter value
} 

void docount_left3()  // counts from the speed sensor
{
  counter_left3++;  // increase +1 the counter value
} 

void docount_right4()  // counts from the speed sensor
{
  counter_right4++;  // increase +1 the counter value
} 

 void timerIsr()
{
  Timer3.detachInterrupt();  //stop the timer
  //Left Motor Speed 
  right_wheel1_vel.data = float(counter_right1)*2*pi*5/8;
  right_wheel1_vel_pub.publish(&right_wheel1_vel);
  left_wheel2_vel.data = float(counter_left2)*2*pi*5/8;
  left_wheel2_vel_pub.publish(&left_wheel2_vel);
  left_wheel3_vel.data = float(counter_left3)*2*pi*5/8;
  left_wheel3_vel_pub.publish(&left_wheel3_vel);
  right_wheel4_vel.data = float(counter_right4)*2*pi*5/8;
  right_wheel4_vel_pub.publish(&right_wheel4_vel);
  counter_right1=0;
  counter_left2=0;
  counter_left3=0;
  counter_right4=0;
  Timer3.attachInterrupt( timerIsr );  //enable the timer
}
 
void cmdRightWheel1( const std_msgs::Int16& msg)
{
  if(msg.data > 0)
  {
    
    digitalWrite(Motor1_Start, LOW);
    digitalWrite(Motor1_Direction, LOW);  
    analogWrite(Motor1_Speed,msg.data);
    
 
  }
  else if(msg.data < 0)
  {  
    digitalWrite(Motor1_Start, LOW);
    digitalWrite(Motor1_Direction, HIGH);
    analogWrite(Motor1_Speed,-msg.data);
  }
  else{
    digitalWrite(Motor1_Start, HIGH);
    analogWrite(Motor1_Speed,0);
  }
}

void cmdLeftWheel2( const std_msgs::Int16& msg)
{
  
  if(msg.data > 0)
  {
    
    digitalWrite(Motor2_Start, LOW);
    digitalWrite(Motor2_Direction, LOW);  
    analogWrite(Motor2_Speed,msg.data);
  }
   else if(msg.data < 0)
  {  
    digitalWrite(Motor2_Start, LOW);
    digitalWrite(Motor2_Direction, HIGH);
    analogWrite(Motor2_Speed,-msg.data);
  }
  else{
    digitalWrite(Motor2_Start, HIGH);
    analogWrite(Motor2_Speed,0);
  }
}

void cmdLeftWheel3( const std_msgs::Int16& msg)
{
  if(msg.data > 0)
  {    
    digitalWrite(Motor3_Start, LOW);
    digitalWrite(Motor3_Direction, LOW); 
    analogWrite(Motor3_Speed,msg.data); 
  }
  else if(msg.data < 0)
  {  
    digitalWrite(Motor3_Start, LOW);
    digitalWrite(Motor3_Direction, HIGH);
    analogWrite(Motor3_Speed,-msg.data);
  }
  else{
    digitalWrite(Motor3_Start, HIGH);
    analogWrite(Motor3_Speed,0);
  }
}

void cmdRightWheel4( const std_msgs::Int16& msg)
{
  
  if(msg.data > 0)
  {
    
    digitalWrite(Motor4_Start, LOW);
    digitalWrite(Motor4_Direction, LOW); 
    analogWrite(Motor4_Speed,msg.data); 
  }
  else if(msg.data < 0)
  {  
    digitalWrite(Motor4_Start, LOW);
    digitalWrite(Motor4_Direction, HIGH);
    analogWrite(Motor4_Speed,-msg.data);
  }
  else{
    digitalWrite(Motor4_Start, HIGH);
    analogWrite(Motor4_Speed,0);
  }
}

ros::Subscriber<std_msgs::Int16> subCmdRight1("cmd_right_wheel1",cmdRightWheel1 );
ros::Subscriber<std_msgs::Int16> subCmdLeft2("cmd_left_wheel2", cmdLeftWheel2);
ros::Subscriber<std_msgs::Int16> subCmdLeft3("cmd_left_wheel3", cmdLeftWheel3 );
ros::Subscriber<std_msgs::Int16> subCmdRight4("cmd_right_wheel4",cmdRightWheel4 );

void setup() {
  // put your setup code here, to run once:
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
  
  digitalWrite(Motor1_Run, LOW);
  digitalWrite(Motor1_Start, HIGH);
  digitalWrite(Motor2_Run, LOW);
  digitalWrite(Motor2_Start, HIGH);
  digitalWrite(Motor3_Run, LOW);
  digitalWrite(Motor3_Start, HIGH);
  digitalWrite(Motor4_Run, LOW);
  digitalWrite(Motor4_Start, HIGH);

  //Setup for encoders
  pinMode(Motor1_Encoder1, INPUT_PULLUP);
  pinMode(Motor2_Encoder1, INPUT_PULLUP);  
  pinMode(Motor3_Encoder1, INPUT_PULLUP);
  pinMode(Motor4_Encoder1, INPUT_PULLUP);  
  
  Timer3.initialize(LOOP_TIME); 
  
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
  
  attachInterrupt(digitalPinToInterrupt(Motor1_Encoder1), docount_right1, RISING);  // increase counter when speed sensor pin goes High
  attachInterrupt(digitalPinToInterrupt(Motor2_Encoder1), docount_left2, RISING);  // increase counter when speed sensor pin goes High
  attachInterrupt(digitalPinToInterrupt(Motor3_Encoder1), docount_left3, RISING);  // increase counter when speed sensor pin goes High
  attachInterrupt(digitalPinToInterrupt(Motor4_Encoder1), docount_right4, RISING);  // increase counter when speed sensor pin goes High


  nh.initNode();
  nh.subscribe(subCmdRight1);
  nh.subscribe(subCmdLeft2);
  nh.subscribe(subCmdLeft3);
  nh.subscribe(subCmdRight4);
  nh.advertise(right_wheel1_vel_pub);
  nh.advertise(left_wheel2_vel_pub);
  nh.advertise(left_wheel3_vel_pub);
  nh.advertise(right_wheel4_vel_pub);

  Timer3.attachInterrupt( timerIsr ); // enable the timer
}

void loop() 
{
    nh.spinOnce();  
}



