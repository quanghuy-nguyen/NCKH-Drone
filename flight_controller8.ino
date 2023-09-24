
#include <Wire.h>

unsigned long DongCo1, DongCo2, DongCo3, DongCo4, esc_timer, esc_loop_timer;
float RateRoll, RatePitch, RateYaw;

float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;

int RateCalibrationNumber;

float ReceiverValue[5];
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;

uint32_t LoopTimer;

//define all variables for PID controller, including variables of P, I, D parameters
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float PIDReturn[] = {0, 0, 0};

float PRateRoll = 1.3; float PRatePitch = PRateRoll; float PRateYaw = 4;
float IRateRoll = 0.04; float IRatePitch = IRateRoll; float IRateYaw = 0.02;
float DRateRoll = 18; float DRatePitch = DRateRoll; float DRateYaw = 0;

//declare the input variables to the motors
float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;

float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
float Kalman1DOutput[]={0,0};

float DesiredAngleRoll, DesiredAnglePitch;
float ErrorAngleRoll, ErrorAnglePitch;

float PrevErrorAngleRoll, PrevErrorAnglePitch;
float PrevItermAngleRoll, PrevItermAnglePitch;
float PAngleRoll=2; float PAnglePitch=PAngleRoll;
float IAngleRoll=0; float IAnglePitch=IAngleRoll;
float DAngleRoll=0; float DAnglePitch=DAngleRoll;

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState+0.004*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}

//read Receiver
ISR(PCINT0_vect){
  current_time = micros();
  //Channel 1=========================================
  if(PINB & B00000001){                                                     //Is input 8 high?
    if(last_channel_1 == 0){                                                //Input 8 changed from 0 to 1.
      last_channel_1 = 1;                                                   //Remember current input state.
      timer_1 = current_time;                                               //Set timer_1 to current_time.
    }
  }
  else if(last_channel_1 == 1){                                             //Input 8 is not high and changed from 1 to 0.
    last_channel_1 = 0;                                                     //Remember current input state.
    ReceiverValue[0] = current_time - timer_1;                             //Channel 1 is current_time - timer_1.
  }
  //Channel 2=========================================
  if(PINB & B00000010 ){                                                    //Is input 9 high?
    if(last_channel_2 == 0){                                                //Input 9 changed from 0 to 1.
      last_channel_2 = 1;                                                   //Remember current input state.
      timer_2 = current_time;                                               //Set timer_2 to current_time.
    }
  }
  else if(last_channel_2 == 1){                                             //Input 9 is not high and changed from 1 to 0.
    last_channel_2 = 0;                                                     //Remember current input state.
    ReceiverValue[1] = current_time - timer_2;                             //Channel 2 is current_time - timer_2.
  }
  //Channel 3=========================================
  if(PINB & B00000100 ){                                                    //Is input 10 high?
    if(last_channel_3 == 0){                                                //Input 10 changed from 0 to 1.
      last_channel_3 = 1;                                                   //Remember current input state.
      timer_3 = current_time;                                               //Set timer_3 to current_time.
    }
  }
  else if(last_channel_3 == 1){                                             //Input 10 is not high and changed from 1 to 0.
    last_channel_3 = 0;                                                     //Remember current input state.
    ReceiverValue[2] = current_time - timer_3;                             //Channel 3 is current_time - timer_3.

  }
  //Channel 4=========================================
  if(PINB & B00001000 ){                                                    //Is input 11 high?
    if(last_channel_4 == 0){                                                //Input 11 changed from 0 to 1.
      last_channel_4 = 1;                                                   //Remember current input state.
      timer_4 = current_time;                                               //Set timer_4 to current_time.
    }
  }
  else if(last_channel_4 == 1){                                             //Input 11 is not high and changed from 1 to 0.
    last_channel_4 = 0;                                                     //Remember current input state.
    ReceiverValue[3] = current_time - timer_4;                             //Channel 4 is current_time - timer_4.
  }
}


//read mpu6050
void gyro_signals(void)
{
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();                                                   
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
  AccX=(float)AccXLSB/4096-0.01;
  AccY=(float)AccYLSB/4096+0.02;
  AccZ=(float)AccZLSB/4096-0.55;
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
}

void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm)
{
  float Pterm = P*Error;
  
  float Iterm = PrevIterm + I*(Error + PrevError); 
  if (Iterm > 400) Iterm = 400;
  else if (Iterm < -400) Iterm = -400;
  
  float  Dterm = D* (Error - PrevError);

  float PIDOutput = Pterm + Iterm + Dterm;
  if (PIDOutput > 400) PIDOutput = 400;
  else if (PIDOutput < -400) PIDOutput = -400;

  //return the ouputs from PID function
  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

void reset_pid(void)
{
  PrevErrorRateRoll = 0; 
  PrevErrorRatePitch = 0;
  PrevErrorRateYaw = 0;

  PrevItermRateRoll = 0;
  PrevItermRatePitch = 0;
  PrevItermRateYaw = 0;

  PrevErrorAngleRoll = 0; PrevErrorAnglePitch = 0;
  PrevItermAngleRoll = 0; PrevItermAnglePitch = 0;
}

void setup() 
{
  pinMode(8, INPUT);
  pinMode(9, INPUT);
  pinMode(10, INPUT);
  pinMode(11, INPUT);

  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);

  pinMode(13, OUTPUT);

  PCICR |= (1 << PCIE0);                                                    //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                                  //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                                                  //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                                                  //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3);  

  Wire.setClock(400000);
  Wire.begin();
  delay(250);
    
  //start gyro in power mode
  Wire.beginTransmission(0x68);               
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
    
  //perform calibration number
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++)
  {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;
  
  
  
  //avoid accidental lift of after setup process
  while (ReceiverValue[2] < 1020 || ReceiverValue[2] > 1050)
  {
    digitalWrite(13, HIGH);
  }

  PORTD |= B11110000;                                                     //Set digital port 4, 5, 6 and 7 high.
  delayMicroseconds(1000);                                                //Wait 1000us.
  PORTD &= B00001111;                                                     //Set digital port 4, 5, 6 and 7 low.

  LoopTimer = micros();
  
  }

void loop() 
{
  gyro_signals();
  
  RateRoll-=RateCalibrationRoll;
  RatePitch-=RateCalibrationPitch;
  RateYaw-=RateCalibrationYaw;
  
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll=Kalman1DOutput[0]; KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch=Kalman1DOutput[0]; KalmanUncertaintyAnglePitch=Kalman1DOutput[1];

  InputThrottle=ReceiverValue[2];

  if(ReceiverValue[0] > 1508) DesiredAngleRoll=0.1*(ReceiverValue[0]-1508);
  else if(ReceiverValue[0] < 1492) DesiredAngleRoll=0.1*(ReceiverValue[0]-1492);

  if(ReceiverValue[1] > 1508) DesiredAnglePitch=0.1*(ReceiverValue[1]-1508);
  else if(ReceiverValue[1] < 1492) DesiredAnglePitch=0.1*(ReceiverValue[1]-1492);
  
  if(ReceiverValue[3] > 1508) DesiredRateYaw=0.15*(ReceiverValue[3]-1508);
  else if(ReceiverValue[3] < 1492) DesiredRateYaw=0.15*(ReceiverValue[3]-1492);
  
  
  ErrorAngleRoll=DesiredAngleRoll-KalmanAngleRoll;
  ErrorAnglePitch=DesiredAnglePitch-KalmanAnglePitch;
  
  pid_equation(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll, PrevItermAngleRoll);     
  DesiredRateRoll=PIDReturn[0]; 
  PrevErrorAngleRoll=PIDReturn[1];
  PrevItermAngleRoll=PIDReturn[2];
  pid_equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorAnglePitch, PrevItermAnglePitch);
  DesiredRatePitch=PIDReturn[0]; 
  PrevErrorAnglePitch=PIDReturn[1];
  PrevItermAnglePitch=PIDReturn[2];
  
  ErrorRateRoll=RateRoll-DesiredRateRoll;
  ErrorRatePitch=RatePitch-DesiredRatePitch;
  ErrorRateYaw=RateYaw-DesiredRateYaw;
  
  pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
       InputRoll=PIDReturn[0];
       PrevErrorRateRoll=PIDReturn[1]; 
       PrevItermRateRoll=PIDReturn[2];
  pid_equation(ErrorRatePitch, PRatePitch,IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
       InputPitch=PIDReturn[0]; 
       PrevErrorRatePitch=PIDReturn[1]; 
       PrevItermRatePitch=PIDReturn[2];
  pid_equation(ErrorRateYaw, PRateYaw,IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
       InputYaw=PIDReturn[0]; 
       PrevErrorRateYaw=PIDReturn[1]; 
       PrevItermRateYaw=PIDReturn[2];

  //limit the throttle
  if (InputThrottle > 1800) InputThrottle = 1800;

  //motor input
  MotorInput1 = (InputThrottle - InputRoll - InputPitch + InputYaw);
  MotorInput2 = (InputThrottle - InputRoll + InputPitch - InputYaw);
  MotorInput3 = (InputThrottle + InputRoll + InputPitch + InputYaw);
  MotorInput4 = (InputThrottle + InputRoll - InputPitch - InputYaw);

  if (MotorInput1 > 1999) MotorInput1 = 1999;
  if (MotorInput2 > 1999) MotorInput2 = 1999; 
  if (MotorInput3 > 1999) MotorInput3 = 1999; 
  if (MotorInput4 > 1999) MotorInput4 = 1999;
  
  int ThrottleIdle=1180;
  if (MotorInput1 < ThrottleIdle) MotorInput1 =  ThrottleIdle;
  if (MotorInput2 < ThrottleIdle) MotorInput2 =  ThrottleIdle;
  if (MotorInput3 < ThrottleIdle) MotorInput3 =  ThrottleIdle;
  if (MotorInput4 < ThrottleIdle) MotorInput4 =  ThrottleIdle;
  
  int ThrottleCutOff = 1000;
  if (ReceiverValue[2]<1050) 
  {
    MotorInput1 = ThrottleCutOff; 
    MotorInput2 = ThrottleCutOff;
    MotorInput3 = ThrottleCutOff; 
    MotorInput4 = ThrottleCutOff;
    reset_pid();
  }

  while(micros() - LoopTimer  < 4000);
  LoopTimer = micros(); //set thời gian.
  PORTD |= B11110000; //Set chân outputs 4,5,6 and 7 high.
  
  DongCo1 = MotorInput1 + LoopTimer; //tính toán thời gian xung PWM rơi của esc1.
  DongCo2 = MotorInput2 + LoopTimer; //tính toán thời gian xung PWM rơi của esc2.
  DongCo3 = MotorInput3 + LoopTimer; //tính toán thời gian xung PWM rơi của esc3.
  DongCo4 = MotorInput4 + LoopTimer; //tính toán thời gian xung PWM rơi của esc4.
  
  while(PORTD >= 16)
  {
  //lập lại cho đến khi chân output 4,5,6 and 7 ở mức thấp.
  esc_loop_timer = micros();
  //đọc thời gian hiện tại.
  if(DongCo1 <= esc_loop_timer)PORTD &= B11101111; //Set chân output 4 xuống mức thấp khi đủ thời gian.
  
  if(DongCo2 <= esc_loop_timer)PORTD &= B11011111; //Set chân output 5 xuống mức thấp khi đủ thời gian.
  
  if(DongCo3 <= esc_loop_timer)PORTD &= B10111111; //Set chân output 6 xuống mức thấp khi đủ thời gian.

  if(DongCo4 <= esc_loop_timer)PORTD &= B01111111; //Set chân output 7 xuống mức thấp khi đủ thời gian.
  }
}
