#define REMOTEXY_MODE__WIFI_POINT
#include <WiFi.h>
#include <RemoteXY.h>
#include <Arduino.h>
#include <Wire.h>
#define MPU6050_ADDR 0x68
#define SDA_PIN 21  
#define SCL_PIN 22    

// -------------------- PIN DEFINITIONS --------------------
#define LED2 2
#define PIN_EMERGENCY_BTN 2
#define MOTOR1_PWM_PIN 13 // ESC signal wire pin
#define MOTOR2_PWM_PIN 12
#define MOTOR3_PWM_PIN 14
#define MOTOR4_PWM_PIN 27

// -------------------- RemoteXY WiFi Settings --------------------
#define REMOTEXY_WIFI_SSID "ESP-DRONE_4A90"
#define REMOTEXY_WIFI_PASSWORD "12345678"
#define REMOTEXY_SERVER_PORT 2390
#define REMOTEXY_ACCESS_PASSWORD "12345678"

// -------------------- RemoteXY GUI Configuration --------------------
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =   // 78 bytes
  { 255,6,0,4,0,71,0,19,0,0,0,69,83,80,45,68,82,79,78,69,
  95,52,65,57,48,0,31,1,200,84,1,1,5,0,5,19,8,60,60,32,
  2,26,31,5,136,7,60,60,32,2,26,31,1,96,58,21,21,0,36,31,
  0,4,3,11,12,63,0,2,26,67,173,72,24,10,78,2,26,2 };

struct {
  int8_t joystick_01_x; // Yaw
  int8_t joystick_01_y;
  int8_t joystick_02_x; // Roll
  int8_t joystick_02_y; // Pitch
  uint8_t Emergency_btn;
  int8_t throttle; // 0 to 100
  float battery;
  uint8_t connect_flag;
} RemoteXY;
#pragma pack(pop)

// -------------------- PWM Setup for Motor ESC --------------------
const int motorPwmChannel = 0;     // PWM channel (0–15)
const int pwmFreq = 50;            // ESCs work with 50Hz
const int pwmResolution = 12;      // 16-bit resolution (0–65535)
const int minPulse = 1000;         // microseconds
const int maxPulse = 2000;         // microseconds

// Store joystick to PWM µs values
float ReceiverValue[4] = {0, 0, 0, 0}; // Roll, Pitch, Throttle, Yaw


// Declaring PID variables
float AccXCalibration,AccYCalibration,AccZCalibration;//added by kshitiz
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float PIDReturn[]= {0,0,0};
float PRateRoll = 1.16; float PRatePitch = PRateRoll; float PRateYaw = 0;
float IRateRoll = 0.005; float IRatePitch = IRateRoll; float IRateYaw = 0;
float DRateRoll = 0.0065; float DRatePitch = DRateRoll; float DRateYaw = 0;

float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
unsigned long LoopTimer;

// Following Code is for battery percentage//
const int BATTERY_PIN = 35; // ADC pin connected to voltage divider
const float R1 = 10000.0; // Resistor values in ohms
const float R2 = 2200.0;
const float ADC_MAX = 4095.0; // Maximum ADC value (12-bit resolution)
const float VOLTAGE_REF = 3.3; // Reference voltage for ADC

//-----------Imu sensor Code-----------//
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNum;

float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
float Kalman1DOutput[]={0,0};

float DesiredAngleRoll,DesiredAnglePitch;
float ErrorAngleRoll,ErrorAnglePitch;

float PrevErrorAngleRoll,PrevErrorAnglePitch;
float PrevItermAngleRoll,PrevItermAnglePitch;

float PAngleRoll=2; float PAnglePitch= PAngleRoll;
float IAngleRoll=0; float IAnglePitch=IAngleRoll;
float DAngleRoll=0; float DAnglePitch= DAngleRoll;

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState+0.004*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}
void gyro_signals(void) {
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
  AccX=(float)AccXLSB/4096;
  AccY=(float)AccYLSB/4096;
  AccZ=(float)AccZLSB/4096-0.17;
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
}
// PID function
void pid_equation(float Error, float P, float I, float D, 
  float PrevError, float PrevIterm) {
    float Pterm = P * Error;
    float Iterm = PrevIterm + I * (Error + PrevError) * 0.004 / 2;
    if(Iterm > 400) Iterm = 400;
    else if (Iterm < -400) Iterm = -400;
    float Dterm = D * (Error - PrevError) / 0.004; // freq of loop is 400us
    float PIDOutput = Pterm + Iterm + Dterm;
    if(PIDOutput > 400) PIDOutput = 400;
    else if (PIDOutput < -400) PIDOutput = -400;
    PIDReturn[0] = PIDOutput;
    PIDReturn[1] = Error;
    PIDReturn[2] = Iterm;
}
// Reset Variable function 
void reset_pid(void) {
  PrevErrorRateRoll = 0; 
  PrevErrorRatePitch = 0;
  PrevErrorRateYaw = 0;
  PrevItermRateRoll = 0;
  PrevItermRatePitch = 0;
  PrevItermRateYaw = 0;

  PrevErrorAngleRoll=0;
  PrevErrorAnglePitch=0;
  PrevItermAngleRoll=0;
  PrevItermAnglePitch=0;
}

void setup() {
  Serial.begin(115200);
  pinMode(LED2, OUTPUT);
  pinMode(PIN_EMERGENCY_BTN, OUTPUT);

  // Init RemoteXY
  RemoteXY_Init();
  digitalWrite(LED2, HIGH); // Startup LED

  // Init PWM for ESC - keeping original ledcAttach approach
  ledcAttach(MOTOR1_PWM_PIN, pwmFreq, pwmResolution);
  analogWrite(MOTOR1_PWM_PIN, 205); // Assuming analogWrite works in this ESP32 environment
  ledcAttach(MOTOR2_PWM_PIN, pwmFreq, pwmResolution);
  analogWrite(MOTOR2_PWM_PIN, 205);
  ledcAttach(MOTOR3_PWM_PIN, pwmFreq, pwmResolution);
  analogWrite(MOTOR3_PWM_PIN, 205);
  ledcAttach(MOTOR4_PWM_PIN, pwmFreq, pwmResolution);
  analogWrite(MOTOR4_PWM_PIN, 205);

  Serial.println("System Ready. Waiting for throttle...");
  // Initialize IMU
  Wire.begin(SDA_PIN, SCL_PIN, 400000);

  delay(250);

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);  // Wake up MPU6050
  Wire.write(0x00);
  Wire.endTransmission();

  // Verify MPU6050 connection
  Wire.beginTransmission(MPU6050_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("MPU6050 not found!");
    while (1);
  }

  // // Gyro Calibration
  // for (RateCalibrationNum = 0; RateCalibrationNum < 2000; RateCalibrationNum++) {
  //   gyro_signals();
  //   RateCalibrationRoll += RateRoll;
  //   RateCalibrationPitch += RatePitch;
  //   RateCalibrationYaw += RateYaw;
  //   delay(1);
  // }

  // RateCalibrationRoll /= 2000;
  // RateCalibrationPitch /= 2000;
  // RateCalibrationYaw /= 2000;
   
   RateCalibrationRoll=-2.3;
   RateCalibrationPitch=-1.72;
   RateCalibrationYaw=-0.3;
   AccXCalibration=0.02;
   AccYCalibration=0.01;
   AccZCalibration=0.18;
  LoopTimer = micros();
}

void loop() {
  //kalman filter code
  gyro_signals();
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  AccX -= AccXCalibration ; //newly added by kshitiz
  AccY -= AccYCalibration ;
  AccZ -= AccZCalibration;
  
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0]; 
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
  
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0]; 
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];
  
  Serial.print("Roll Angle [°] ");
  Serial.print(KalmanAngleRoll);
  Serial.print(" Pitch Angle [°] ");
  Serial.println(KalmanAnglePitch);
  
  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();
  
  // Code for battery percentage
  int adcValue = analogRead(BATTERY_PIN); // Read ADC value
  float vOut = (adcValue / ADC_MAX) * VOLTAGE_REF; // Convert ADC value to voltage
  float vIn = vOut * (R1 + R2) / R2; // Calculate actual battery voltage
  RemoteXY.battery = vIn; // Update battery voltage in RemoteXY
  Serial.print("Battery Voltage: ");
  Serial.print(vIn);
  Serial.println(" V");

  RemoteXY_Handler();

  // Convert joystick to 1000–2000 µs
  ReceiverValue[0] = map(RemoteXY.joystick_02_x, -100, 100, 1000, 2000); // Roll
  ReceiverValue[1] = map(RemoteXY.joystick_02_y, -100, 100, 1000, 2000); // Pitch
  ReceiverValue[2] = map(RemoteXY.throttle, 0, 100, 1000, 2000);         // Throttle
  ReceiverValue[3] = map(RemoteXY.joystick_01_x, -100, 100, 1000, 2000); // Yaw

  // Added conversion from joystick to desired angles
  DesiredAngleRoll = 0.10 * (ReceiverValue[0] - 1500);  // Scale for angle control
  DesiredAnglePitch = 0.10 * (ReceiverValue[1] - 1500); // Scale for angle control
  InputThrottle = ReceiverValue[2];
  DesiredRateYaw = 0.15 * (ReceiverValue[3] - 1500);

  ErrorAngleRoll = DesiredAngleRoll - KalmanAngleRoll;
  ErrorAnglePitch = DesiredAnglePitch - KalmanAnglePitch;
  
  pid_equation(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll, PrevItermAngleRoll);
  DesiredRateRoll = PIDReturn[0];
  PrevErrorAngleRoll = PIDReturn[1];
  PrevItermAngleRoll = PIDReturn[2];

  pid_equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorAnglePitch, PrevItermAnglePitch);
  DesiredRatePitch = PIDReturn[0];
  PrevErrorAnglePitch = PIDReturn[1];
  PrevItermAnglePitch = PIDReturn[2];

  //for inner loop controller
  ErrorRateRoll = DesiredRateRoll - RateRoll; // Calculating the error for the pid calculations
  ErrorRatePitch = DesiredRatePitch - RatePitch;
  ErrorRateYaw = DesiredRateYaw - RateYaw;

  pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
  InputRoll = PIDReturn[0];
  PrevItermRateRoll = PIDReturn[2];
  PrevErrorRateRoll = PIDReturn[1];
  
  pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
  InputPitch = PIDReturn[0];
  PrevErrorRatePitch = PIDReturn[1];
  PrevItermRatePitch = PIDReturn[2];
  
  pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
  InputYaw = PIDReturn[0];
  PrevErrorRateYaw = PIDReturn[1];
  PrevItermRateYaw = PIDReturn[2];

  if(InputThrottle > 1800) // limiting to 80%
    InputThrottle = 1800;

  MotorInput1 = 1.024 * (InputThrottle - InputRoll - InputPitch - InputYaw); // using quadcopter dynamics equation
  MotorInput2 = 1.024 * (InputThrottle - InputRoll + InputPitch + InputYaw);
  MotorInput3 = 1.024 * (InputThrottle + InputRoll + InputPitch - InputYaw);
  MotorInput4 = 1.024 * (InputThrottle + InputRoll - InputPitch + InputYaw);
 
  if(MotorInput1 > 2000) MotorInput1 = 1999;
  if(MotorInput2 > 2000) MotorInput2 = 1999;
  if(MotorInput3 > 2000) MotorInput3 = 1999;
  if(MotorInput4 > 2000) MotorInput4 = 1999;

  int ThrottleIdle = 1180;
  if(MotorInput1 < ThrottleIdle) MotorInput1 = ThrottleIdle; // keeping the drone minimally 18% power during flight
  if(MotorInput2 < ThrottleIdle) MotorInput2 = ThrottleIdle;
  if(MotorInput3 < ThrottleIdle) MotorInput3 = ThrottleIdle;
  if(MotorInput4 < ThrottleIdle) MotorInput4 = ThrottleIdle;

  int ThrottleCutOff = 1000;
  if(ReceiverValue[2] < 1050) {  // Turning off the motors
    MotorInput1 = ThrottleCutOff;
    MotorInput2 = ThrottleCutOff;
    MotorInput3 = ThrottleCutOff;
    MotorInput4 = ThrottleCutOff;
    reset_pid();
  }
  
  analogWrite(MOTOR1_PWM_PIN, map(MotorInput1, 1000, 2000, 205, 410));
  analogWrite(MOTOR2_PWM_PIN, map(MotorInput2, 1000, 2000, 205, 410));
  analogWrite(MOTOR3_PWM_PIN, map(MotorInput3, 1000, 2000, 205, 410));
  analogWrite(MOTOR4_PWM_PIN, map(MotorInput4, 1000, 2000, 205, 410));

  while(micros() - LoopTimer < 400); // finish the 400 us / 2500 Hz control loop
  LoopTimer = micros();
}