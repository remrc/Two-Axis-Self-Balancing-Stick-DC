#include <Wire.h>

#define MPU6050       0x68    // Device address
#define ACCEL_CONFIG  0x1C    // Accelerometer configuration address
#define GYRO_CONFIG   0x1B    // Gyro configuration address
#define PWR_MGMT_1    0x6B
#define PWR_MGMT_2    0x6C

#define   sensorX     A3
#define   sensorY     A2
#define   _2PI        6.28318530718f

#define   InA_m1      8       // INA right motor pin 
#define   InB_m1      10      // INB right motor pin
#define   m1PWM       5       // PWM right motor pin

#define   InA_m2      3       // INA left motor pin
#define   InB_m2      4       // INB left motor pin
#define   m2PWM       6       // PWM left motor pin

#define   BUZZER     12
#define   VBAT       A7

#define   accSens   0             // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
#define   gyroSens  1             // 0 = 250rad/s, 1 = 500rad/s, 2 1000rad/s, 3 = 2000rad/s
#define   Gyro_amount 0.996       // percent of gyro in complementary filter

float K1Gain = 170.0; 
float K2Gain = 22.0;  
float K3Gain = 13.0;  
float K4Gain = 0.04;  
float loop_time = 10; 

int pwmX, pwmY;
int32_t motor_speed_X, motor_speed_Y; 
float motor_speed_enc_X, motor_speed_enc_Y; 
uint32_t timer;
long currentT, previousT_1, previousT_2 = 0; 
int16_t AcX, AcY, AcZ, GyY, GyZ, gyroY, gyroZ;

float angle_prev_X = 0.0f;        // result of last call to getSensorAngle(), used for full rotations and velocity
float velocity_X = 0.0f;
long angle_prev_ts_X = 0;         // timestamp of last call to getAngle, used for velocity
float vel_angle_prev_X = 0.0f;    // angle at last call to getVelocity, used for velocity
long vel_angle_prev_ts_X = 0;     // last velocity calculation timestamp
int32_t full_rotations_X = 0;     // full rotation tracking
int32_t vel_full_rotations_X = 0; // previous full rotation value for velocity calculation

float angle_prev_Y = 0.0f;        // result of last call to getSensorAngle(), used for full rotations and velocity
float velocity_Y = 0.0f;
long angle_prev_ts_Y = 0;         // timestamp of last call to getAngle, used for velocity
float vel_angle_prev_Y = 0.0f;    // angle at last call to getVelocity, used for velocity
long vel_angle_prev_ts_Y = 0;     // last velocity calculation timestamp
int32_t full_rotations_Y = 0;     // full rotation tracking
int32_t vel_full_rotations_Y = 0; // previous full rotation value for velocity calculation

int max_raw_count = 1; 
int min_raw_count = 1023;

//IMU offset values
int16_t  AcX_offset = 1520;
int16_t  AcY_offset = -400;
int16_t  AcZ_offset = 2100;
int16_t  GyZ_offset = 0;
int32_t  GyZ_offset_sum = 0;
int16_t  GyY_offset = 0;
int32_t  GyY_offset_sum = 0;

float alpha = 0.4; // low pass 
float gyroYfilt, gyroZfilt;

float robot_angleX, robot_angleY;
float Acc_angleX, Acc_angleY;

bool vertical = false;      // is the robot vertical

uint8_t i2cData[14]; // Buffer for I2C data

void setup() {
  Serial.begin(115200);
  pinMode(InA_m1, OUTPUT); 
  pinMode(InB_m1, OUTPUT); 
  pinMode(InA_m2, OUTPUT); 
  pinMode(InB_m2, OUTPUT); 
  digitalWrite(BUZZER, LOW);
  pinMode(BUZZER, OUTPUT);
  delay(2000);
  digitalWrite(BUZZER, HIGH);
  delay(70);
  digitalWrite(BUZZER, LOW);
  angle_setup();
}

void loop() {
  currentT = millis();
  if (currentT - previousT_1 >= loop_time) {
    Tuning(); 
    angle_calc();
    if (vertical) {
      gyroZ = GyZ / 131.0; // Convert to deg/s
      gyroY = GyY / 131.0; // Convert to deg/s
      gyroYfilt = alpha * gyroY + (1 - alpha) * gyroYfilt;
      gyroZfilt = alpha * gyroZ + (1 - alpha) * gyroZfilt;
      pwmX = constrain(K1Gain * robot_angleX + K2Gain * gyroZfilt + K3Gain * motor_speed_enc_X + K4Gain * motor_speed_X, -255, 255);
      pwmY = constrain(K1Gain * robot_angleY + K2Gain * gyroYfilt + K3Gain * motor_speed_enc_Y + K4Gain * motor_speed_Y, -255, 255);
      motor_speed_enc_X = -getVelocity_X();
      motor_speed_enc_Y = getVelocity_Y();
      MotorX_control(pwmX);
      MotorY_control(pwmY);
      motor_speed_X += motor_speed_enc_X;
      motor_speed_Y += motor_speed_enc_Y;
    } else {
      MotorX_control(0);
      MotorY_control(0);
      motor_speed_X = 0;
      motor_speed_Y = 0;
    }
    previousT_1 = currentT;
  }
  if (currentT - previousT_2 >= 1000) {
    battVoltage((double)analogRead(VBAT) / 74); 
    previousT_2 = currentT;
  }
}

