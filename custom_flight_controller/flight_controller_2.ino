#include <Wire.h> //I2C protocal library
// Constants
#define MPU_ADDRESS 0x68
#define GYRO_SENSITIVITY 65.5  // For ±500°/s
#define Ts 0.04 // Per 250Hz
//Variables for gyro reading and clibration offsets
float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw;
int RateCalibrationNumber;


#include <PulsePosition.h>
PulsePositionInput ReceiverInput(RISING); // ppm input object from Wire.h library
float ReceiverValue[] = {0, 0, 0, 0, 0, 0, 0, 0}; // array to store up to 8 channel values, wont use all of them
int ChannelNumber = 0; // stores number of channels transmitted by the receiver 
//Variables for flight control checking
int ThrottleI=1200; //Idel flight at 20%
int ThrottleCO=1000; // No flight at 0%

float Voltage, Current, BatteryRemaining, BatteryAtStart;
float CurrentConsumed=0; //At the begining 
float BatteryDefault=2200; //mAh, this is my 3s battery rating

uint32_t LoopTimer; //Time Parameter for inside of control loop
//PID Controller Variables
float ErrorRoll, ErrorPitch, ErrorYaw;
float PrevItermRoll, PrevItermPitch, PrevItermYaw;
float PrevErrorRoll, PrevErrorPitch, PrevErrorYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float DesiredRoll, DesiredPitch,DesiredYaw;
float PIDReturn[]= {0,0,0};

//Following values are constants in the PID control loop from the interweb 
float PRoll=0.6;
float PPitch=0.6; 
float IRoll=3.5;
float IPitch=3.5;
float DRoll=0.03;
float DPitch= 0.03;
float PYaw=2;
float IYaw=12;
float DYaw=0;

//Values that will be determined based on PID values and sent to motors
float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
//Acceleration Values 
float AccX, AccY, AccZ;
//AngleValues
float AngleRoll,AnglePitch;


float KalmanAngleRoll=0,//Predicted roll angle 
  KalmanUncertaintyAngleRoll=2*2;//Guesing about 2 degress/second uncertainty 
float KalmanAnglePitch=0, //Predicted Pitch angle 
  KalmanUncertaintyAnglePitch=2*2;//Guqessing about 2 degress/second uncertainty
float Kalman1DOutput[]={0,0}; //first 0 is the angle prediction and second zero is the uncertainty of the prediction
//Defining error and desired angle values
float DesiredAngleRoll, DesiredAnglePitch;
float ErrorAngleRoll, ErrorAnglePitch; 

//Values for angle PID 
float PrevErrorAngleRoll, PrevErrorAnglePitch;
float PrevItermAngleRoll, PrevItermAnglePitch;

float PAngleRoll=2; 
float PAnglePitch=2;
float IAngleRoll=0;
float IAnglePitch=0;
float DAngleRoll=0;
float DAnglePitch=0;

void kalman_1d(float &KalmanState, float &KalmanUncertainty, float &KalmanInput, float &KalmanMeasurement) {
  KalmanState=KalmanState+0.004*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 2 * 2;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 1.5 * 1.5);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;

  //Kalman filter output
  Kalman1DOutput[0]=KalmanState;
  Kalman1DOutput[1]=KalmanUncertainty;

}

//Battery Voltage measuring function 
void battery_voltage(void){
  Voltage=(float)analogRead(15)/63.1;
  Current=(float)analogRead(21)*0.0822;// Math in google doc or yellow sheet I wrote notes on
}

// Function to read receiver data 
void read_receiver(void) {
  ChannelNumber = ReceiverInput.available(); // checking how many channels are available
  if (ChannelNumber > 0) { // if channel is available 
    int maxChannels = sizeof(ReceiverValue) / sizeof(ReceiverValue[0]); // protect against overflow
    for (int i = 1; i <= ChannelNumber && i <= maxChannels; i++) {
      ReceiverValue[i - 1] = ReceiverInput.read(i); // go through for loop of taking those values and storing it in the array, this would be happening 100s of times per sec
    }
  }
}

void printReceiverValues() {
  Serial.print("Motor1: ");
  Serial.print(MotorInput1);
  Serial.print("Motor2: ");
  Serial.print(MotorInput2);
  Serial.print("Motor3: ");
  Serial.print(MotorInput3);
  Serial.print("Motor4: ");
  Serial.println(MotorInput4);
}


void gyro_signals(void){
  Wire.beginTransmission(0x68); //Start I2C protocal 
  Wire.write(0x1A); //Open up Low Pass filter register adress Hex, so we dont have random signals
  Wire.write(0x05); //Choosigin 10Hz cut off frequency 
  Wire.endTransmission(); //Setting Chosen
  Wire.beginTransmission(0x68); //Start I2C protocal 
  Wire.write(0x1C); //Open up accelerometer configuration settings with register adress Hex
  Wire.write(0x10); // 8g range, AFS_SEL setting is 2 which correlates to a 0 for bit 3 and 1 for bit 4, all other remainder bits are set to 0 
  // so we have 1*x2^4 which is binary of 16 or hex 10
  Wire.endTransmission();
  Wire.beginTransmission(0x68); //Start I2C protocal 
  Wire.write(0x3B);//Opening up first register for the ACC
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);//Request 6 bites so I can pull info from sensor
  int16_t AccXLSB = Wire.read() <<8 | Wire.read();//ACC in LSB, this is an unsigned 16 bit measurment across 2 registers 0 
  //With 0-7 bits being read in decimal 60 and 8-15, being red in decimal 59, meaning we need to read the values twice 
  int16_t AccYLSB = Wire.read() <<8 | Wire.read();// ACC in LSB
  int16_t AccZLSB = Wire.read() <<8 | Wire.read(); //ACC in LSB 
  Wire.beginTransmission(0x68);//Starting I2C protocal 
  Wire.write(0x1B); //Register Hex to adjust the sensitivity factor
  Wire.write(0x8); //Hex converted to give senstivity of 65.5 LSB/s
  Wire.endTransmission(); //Finish this command
  Wire.beginTransmission(0x68);//Starting I2C protocal with the MPU6050
  Wire.write(0x43);//Indicate the first register tat we want to use 
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);//Taking 6 bytes 
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  Serial.print("GyroX: "); Serial.print(GyroX);
  Serial.print(" GyroY: "); Serial.print(GyroY);
  Serial.print(" GyroZ: "); Serial.println(GyroZ);
  RateRoll=(float)GyroX/GYRO_SENSITIVITY;
  RatePitch=(float)GyroY/GYRO_SENSITIVITY;
  RateYaw=(float)GyroZ/GYRO_SENSITIVITY;

  AccX=(float)AccXLSB/4096 -0.01; //4096 is the LSB sensativty given with the 8g range that we chose in the 1C register
  AccY=(float)AccYLSB/4096 +0.01;
  AccZ=(float)AccZLSB/4096  -0.02;

  AnglePitch= -atan(AccX/(sqrt(AccY*AccY + AccZ*AccZ)))*1/(180/3.142);//Rads to degrees
  AngleRoll= atan(AccY/sqrt(AccX*AccX+AccZ*AccZ)) *1/(180/3.142);

  Serial.print("AnglePitch: "); Serial.print(AnglePitch);
  Serial.print(" AngleRoll: "); Serial.println(AngleRoll);
}

void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm){
  float Pterm= P*Error; 
  float Iterm= PrevIterm + (I*(Error + PrevError))*Ts/2;
  if (Iterm>400) { // These if's are to avoid integral windup, which would result if we try 
    Iterm=400; // and put the motor really high all at once, overshooting our set point, having saturation of our motor
    //essentially when we ask the motor for a lot but its already at its max
    //slower rising slope to our equilibrium point and higher overshoot 
  }
  else if (Iterm <-400){
    Iterm=-400;
  }
  float Dterm= D*(Error-PrevError)/Ts; 
  float PIDOutput = Pterm+Iterm+Dterm; 
  if (PIDOutput>400){ //Same thing as for Iterm, but with out overall output, we dont want values to extreme
    PIDOutput=400;
  }
  else if (PIDOutput<-400){
    PIDOutput=-400;
  }
  PIDReturn[0]=PIDOutput; //Maping our output Value to send to motor
  PIDReturn[1]=Error; //Error Value so we can have previous error
  PIDReturn[2]=Iterm; //Iterm Value so we can have previous Iterm
}

void reset_pid(void){ //Reset the variable for when motors are not on 
  PrevErrorRoll=0; 
  PrevErrorPitch=0;
  PrevErrorYaw=0;
  PrevItermRoll=0;
  PrevItermPitch=0;
  PrevItermYaw=0;
  //For New Angle Loop as well
  PrevErrorAngleRoll=0;
  PrevErrorAnglePitch=0;
  PrevItermAngleRoll=0;
  PrevItermAnglePitch=0;
}

void setup(){
  pinMode (5,OUTPUT);
  digitalWrite(5,HIGH);
  pinMode (13,OUTPUT);
  digitalWrite(13,HIGH);
  Wire.setClock(400000); // Fast I2C
  Wire.begin();
  

  delay(250);  // Stabilization time

  // Wake up MPU-6050
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x6B);  // Power Management 1
  Wire.write(0x00);  // Wake up
  Wire.endTransmission();

  // Calibrate gyroscope
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }
  RateCalibrationRoll/= 2000;
  RateCalibrationPitch/= 2000;
  RateCalibrationYaw/= 2000;

//Communication with Motors 
  analogWriteFrequency(1,250);
  analogWriteFrequency(2,250);
  analogWriteFrequency(3,250);
  analogWriteFrequency(4,250);
  analogWriteResolution(12);

  pinMode(6,OUTPUT);
  digitalWrite(6,HIGH);
  battery_voltage();
  if(Voltage>12.45) {digitalWrite(5,LOW);
    BatteryAtStart=BatteryDefault;}
  else if (Voltage<11.3){
    BatteryAtStart=30/100*BatteryDefault;
  }
  else {digitalWrite(5,LOW);
      BatteryAtStart=(53.361*Voltage-566.2)/100 * BatteryDefault;}
      //percentage in mAh of battery left

  ReceiverInput.begin(14); //Avoid Accidental lift off after the seup

  LoopTimer=micros(); //Sarting the timer for the control loop

}

void loop(){
  
 
  gyro_signals();
  RateRoll-=RateCalibrationRoll;
  RatePitch-=RateCalibrationPitch;
  RateYaw-=RateCalibrationYaw;

  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll=Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch,RatePitch, AnglePitch);
  KalmanAnglePitch=Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch=Kalman1DOutput[1];

  read_receiver();
  Serial.print("Throttle: "); Serial.print(ReceiverValue[0]);
  Serial.print(" Pitch: "); Serial.print(ReceiverValue[2]);
  Serial.print(" Roll: "); Serial.print(ReceiverValue[1]);
  Serial.print(" Yaw: "); Serial.println(ReceiverValue[3]);


  //Desired angles from the receiver, slightly different from before because we are between 50 and -50 degrees
  DesiredAngleRoll=0.10*(ReceiverValue[1]-1500);
  DesiredAnglePitch=0.10*(ReceiverValue[2]-1500);
  InputThrottle=ReceiverValue[0];
  DesiredYaw=0.15*(ReceiverValue[3]-1500);
  ErrorAngleRoll=DesiredAngleRoll-KalmanAngleRoll;
  ErrorAnglePitch=DesiredAnglePitch-KalmanAnglePitch;

  pid_equation(ErrorAngleRoll,PAngleRoll,IAngleRoll,DAngleRoll,PrevErrorAngleRoll,PrevItermAngleRoll);
  DesiredRoll=PIDReturn[0];
  PrevErrorAngleRoll=PIDReturn[1];
  PrevItermAngleRoll=PIDReturn[2];

  pid_equation(ErrorAnglePitch,PAnglePitch,IAnglePitch,DAnglePitch,PrevErrorAnglePitch,PrevItermAnglePitch);
  DesiredPitch=PIDReturn[0];
  PrevErrorAnglePitch=PIDReturn[1];
  PrevItermAnglePitch=PIDReturn[2];
  
  ErrorRoll=DesiredRoll-RateRoll;
  ErrorPitch=DesiredPitch-RatePitch;
  ErrorYaw=DesiredYaw-RateYaw;

  pid_equation(ErrorRoll,PRoll,IRoll,DRoll,PrevErrorRoll,PrevItermRoll);
  InputRoll=PIDReturn[0];
  PrevErrorRoll=PIDReturn[1];
  PrevItermRoll=PIDReturn[2];

  pid_equation(ErrorPitch,PPitch,IPitch,DPitch,PrevErrorPitch,PrevItermPitch);
  InputPitch=PIDReturn[0];
  PrevErrorPitch=PIDReturn[1];
  PrevItermPitch=PIDReturn[2];
NEW SKETCH


  pid_equation(ErrorYaw,PYaw,IYaw,DYaw,PrevErrorYaw,PrevIt
NEW SKETCH
ermYaw);
  InputYaw=PIDReturn[0];
  PrevErrorYaw=PIDReturn[1];
  PrevItermYaw=PIDReturn[2];
  

  Serial.print("Pitch: "); Serial.print(InputPitch);
  Serial.print(" Roll: "); Serial.print(InputRoll);
  Serial.print(" Yaw: "); Serial.println(InputYaw);

  if (InputThrottle>1800) {
    InputThrottle = 1800;} //making sure we always have enough rate left in the motor to account for a turn

  MotorInput1=1.024*(InputThrottle-InputRoll+InputPitch-InputYaw);
  MotorInput2=1.024*(InputThrottle+InputRoll+InputPitch+InputYaw);
  MotorInput3=1.024*(InputThrottle+InputRoll-InputPitch-InputYaw);
  MotorInput4=1.024*(InputThrottle-InputRoll-InputPitch+InputYaw);
  

  
    //Capping out the max power to the motors so we dont smoke them
  if (MotorInput1>=2000) {
    MotorInput1=1999;
  }

  if (MotorInput2>=2000) {
    MotorInput2=1999;
  }

    if (MotorInput3>=2000) {
    MotorInput3=1999;
  }

    if (MotorInput4>=2000) {
    MotorInput4=1999;
  }

  //Always keeping the motors at 20% during flight to sustain itself

  if (MotorInput1<ThrottleI){
    MotorInput1=ThrottleI; 
  }

    if (MotorInput2<ThrottleI){
    MotorInput2=ThrottleI; 
  }

    if (MotorInput3<ThrottleI){
    MotorInput3=ThrottleI; 
  }

    if (MotorInput4<ThrottleI){
    MotorInput4=ThrottleI; 
  }

  if (ReceiverValue[0]<950){
    MotorInput1=ThrottleCO;//CO=Cut Off
    MotorInput2=ThrottleCO;
    MotorInput3=ThrottleCO;
    MotorInput4=ThrottleCO;
    reset_pid();
  }
  
  printReceiverValues(); 
  analogWrite(1,MotorInput1);
  analogWrite(2,MotorInput2);
  analogWrite(3,MotorInput3);
  analogWrite(4,MotorInput4);
  
  battery_voltage(); //Get the current battery voltage
  CurrentConsumed=(Current*(1000/3600)*0.004) + CurrentConsumed;//Total current consumed during the flight in mAh
  BatteryRemaining=(BatteryAtStart-CurrentConsumed)/BatteryDefault*100;// Calcullating percenet difference im mAh 
  if(BatteryRemaining<30) {
    digitalWrite(5,HIGH);}
  else {
    digitalWrite(5,LOW);} 

  while(micros()-LoopTimer<4000);
  LoopTimer=micros(); 
}
