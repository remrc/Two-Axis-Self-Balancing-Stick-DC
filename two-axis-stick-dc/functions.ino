void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

float getSensorAngle_X() {
  int cpr = max_raw_count - min_raw_count;
  int raw_count = analogRead(sensorX);  
  return ((float) (raw_count) / (float)cpr) * _2PI;
}

float getVelocity_X() {
  float val = getSensorAngle_X();
  angle_prev_ts_X = micros();
  float d_angle = val - angle_prev_X;
  // if overflow happened track it as full rotation
  if (abs(d_angle) > (0.8f * _2PI)) full_rotations_X += (d_angle > 0) ? -1 : 1; 
  angle_prev_X = val;

  // calculate sample time
  float Ts = (angle_prev_ts_X - vel_angle_prev_ts_X)*1e-6;
  
  velocity_X = ((float)(full_rotations_X - vel_full_rotations_X) * _2PI + (angle_prev_X - vel_angle_prev_X) ) / Ts;
  vel_angle_prev_X = angle_prev_X;
  vel_full_rotations_X = full_rotations_X;
  vel_angle_prev_ts_X = angle_prev_ts_X;
  return velocity_X;
}

float getSensorAngle_Y() {
  int cpr = max_raw_count - min_raw_count;
  int raw_count = analogRead(sensorY);  
  return ((float) (raw_count) / (float)cpr) * _2PI;
}

float getVelocity_Y() {
  float val = getSensorAngle_Y();
  angle_prev_ts_Y = micros();
  float d_angle = val - angle_prev_Y;
  // if overflow happened track it as full rotation
  if (abs(d_angle) > (0.8f * _2PI)) full_rotations_Y += (d_angle > 0) ? -1 : 1; 
  angle_prev_Y = val;
    
  // calculate sample time
  float Ts = (angle_prev_ts_Y - vel_angle_prev_ts_Y)*1e-6;

  velocity_Y = ((float)(full_rotations_Y - vel_full_rotations_Y) * _2PI + (angle_prev_Y - vel_angle_prev_Y) ) / Ts;
  vel_angle_prev_Y = angle_prev_Y;
  vel_full_rotations_Y = full_rotations_Y;
  vel_angle_prev_ts_Y = angle_prev_ts_Y;
  return velocity_Y;
}

void angle_setup() {
  Wire.begin();
  delay (100);
  writeTo(MPU6050, PWR_MGMT_1, 0);
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3); // Specifying output scaling of accelerometer
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3); // Specifying output scaling of gyroscope
  delay (100);

  for (int i = 0; i < 1024; i++) {
    angle_calc();
    GyY_offset_sum += GyY;
    GyZ_offset_sum += GyZ;
    delay(5);
  }
  GyY_offset = GyY_offset_sum >> 10;
  Serial.print("GyY offset value = "); Serial.println(GyY_offset);
  GyZ_offset = GyZ_offset_sum >> 10;
  Serial.print("GyZ offset value = "); Serial.println(GyZ_offset);
  
  digitalWrite(BUZZER, HIGH);
  delay(70);
  digitalWrite(BUZZER, LOW);
  delay(80);
  digitalWrite(BUZZER, HIGH);
  delay(70);
  digitalWrite(BUZZER, LOW);
}

void angle_calc() {
  
  // read raw accel/gyro measurements from device
  Wire.beginTransmission(MPU6050);
  Wire.write(0x45);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true);  
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);                  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true);
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  AcXc = AcX - offsets.X;
  AcYc = AcY - offsets.Y;  
  AcZc = AcZ - offsets.Z;
  GyZ -= GyZ_offset;
  GyY -= GyY_offset;

  robot_angleX += GyZ * loop_time / 1000 / 65.536;
  Acc_angleX = atan2(AcYc, -AcXc) * 57.2958;        // angle from acc. values  * 57.2958 (deg/rad)
  robot_angleX = robot_angleX * Gyro_amount + Acc_angleX * (1.0 - Gyro_amount);
  robot_angleY += GyY * loop_time / 1000 / 65.536; 
  Acc_angleY = -atan2(AcZc, -AcXc) * 57.2958;       //angle from acc. values  * 57.2958 (deg/rad)
  robot_angleY = robot_angleY * Gyro_amount + Acc_angleY * (1.0 - Gyro_amount);
  //check if robot is vertical
  if (abs(robot_angleX) > 6 || abs(robot_angleY) > 6) vertical = false;
  if (abs(robot_angleX) < 0.3 && abs(robot_angleY) < 0.3) vertical = true;
}

void battVoltage(double voltage) {
  //Serial.print("batt: "); Serial.println(voltage); //debug
  if ((voltage > 5.5 && voltage < 6.4) || (voltage > 8.8 && voltage < 9.5)) {   // 2S and 3S
    digitalWrite(BUZZER, HIGH);
  } else {
    digitalWrite(BUZZER, LOW);
  }
}

void MotorX_control(int pwm) { 
    if (pwm > 0) {
      digitalWrite(InA_m1, LOW);                        
      digitalWrite(InB_m1, HIGH);
    } else {
      digitalWrite(InA_m1, HIGH);                       
      digitalWrite(InB_m1, LOW);
    } 
    analogWrite(m1PWM, abs(pwm)); 
}

void MotorY_control(int pwm) { 
    if (pwm > 0) {
      digitalWrite(InA_m2, LOW);                        
      digitalWrite(InB_m2, HIGH);
    } else {
      digitalWrite(InA_m2, HIGH);                       
      digitalWrite(InB_m2, LOW);
    } 
    analogWrite(m2PWM, abs(pwm)); 
}


int Tuning() {
  if (!Serial.available())  return 0;
  delay(2);
  char param = Serial.read();               // get parameter byte
  if (!Serial.available()) return 0;
  char cmd = Serial.read();                 // get command byte
  Serial.flush();
  switch (param) {
    case 'p':
      if (cmd == '+')    K1Gain += 1;
      if (cmd == '-')    K1Gain -= 1;
      printValues();
      break;
    case 'i':
      if (cmd == '+')    K2Gain += 0.2;
      if (cmd == '-')    K2Gain -= 0.2;
      printValues();
      break;
    case 'a':
      if (cmd == '+')    K3Gain += 1;
      if (cmd == '-')    K3Gain -= 1;
      printValues();
      break;    
    case 's':
      if (cmd == '+')    K4Gain += 0.005;
      if (cmd == '-')    K4Gain -= 0.005;
      printValues();
      break;
    case 'c':
      if (cmd == '+' && !calibrating) {
        calibrating = true;
         Serial.println("calibrating on");
      }
      if (cmd == '-' && calibrating)  {
        Serial.println("calibrating off");
        Serial.print("X: "); Serial.print(AcX + 16384); Serial.print(" Y: "); Serial.print(AcY);  Serial.print(" Z: "); Serial.println(AcZ);
        if (abs(AcZ) < 5000 && abs(AcY) < 5000) {
		      offsets.ID = 77;
          offsets.X = AcX + 16384;
          offsets.Y = AcY;
          offsets.Z = AcZ;
          digitalWrite(BUZZER, HIGH);
          delay(70);
          digitalWrite(BUZZER, LOW);
          EEPROM.put(0, offsets);
		      calibrating = false;
		      calibrated = true;
        } else {
          Serial.println("The angles are wrong!!!");
          digitalWrite(BUZZER, HIGH);
          delay(50);
          digitalWrite(BUZZER, LOW);
          delay(70);
          digitalWrite(BUZZER, HIGH);
          delay(50);
          digitalWrite(BUZZER, LOW);
        }
      }
      break;          
  }
}

void printValues() {
  Serial.print("K1: "); Serial.print(K1Gain);
  Serial.print(" K2: "); Serial.print(K2Gain);
  Serial.print(" K3: "); Serial.print(K3Gain, 3);
  Serial.print(" K4: "); Serial.println(K4Gain, 3);
}

