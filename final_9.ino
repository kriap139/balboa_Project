#include <Balboa32U4.h>
#include <Wire.h>
#include <LSM6.h>
LSM6 imu;
Balboa32U4Motors motors;
Balboa32U4Encoders encoders;

//PID parameters
const float kp = 6;
const double ki = 1.5;
const double kd = 0.04;

const float MILLI_DEGREES_TO_DEGREES = 0.001;
const int16_t MOTOR_SPEED_LIMIT = 300;
const int16_t MOTOR_SPEED_MIN_LIMIT = 60;
const int16_t MOTOR_SPEED_MIN_LIMIT_NEG = -60;

const uint8_t UPDATE_TIME_MS = 10;
const uint8_t CALIBRATION_ITERATIONS = 100;

int32_t gYZero;
int32_t angle;      // millidegrees
int32_t angleRate; // degrees/s
int16_t motorSpeed;

double angleError = 0;
double prevError= 0;
double accError = 0;

// to remove
int32_t gyroAngleACC = 0;
int32_t accelAngleACC = 0;
// /to remove

void balanceSetup();

void setup()
{
  ledYellow(0);
  ledRed(1);
  balanceSetup();
  ledRed(0);
  motors.allowTurbo(true);
}

void balanceSetup()
{
  // Initialize IMU.
  Wire.begin();
  if (!imu.init())
  {
    while(true)
    {
      Serial.println("Failed to detect and initialize IMU!");
      delay(200);
    }
  }
  imu.enableDefault();
  imu.writeReg(LSM6::CTRL2_G, 0b01011000); // 208 Hz, 1000 deg/s

  // Wait for IMU readings to stabilize.
  delay(1000);

  // Calibrate the gyro.
  int32_t total = 0;
  for (int i = 0; i < CALIBRATION_ITERATIONS; i++)
  {
    imu.read();
    total += imu.g.y;
    delay(1);
  }

  gYZero = total / CALIBRATION_ITERATIONS;
}

void checkMotorMinMaxLimits()
{
  if ((motorSpeed > (-MOTOR_SPEED_MIN_LIMIT)) && motorSpeed < 0) 
  {
      motorSpeed = MOTOR_SPEED_MIN_LIMIT_NEG;
  }
  else if (motorSpeed < MOTOR_SPEED_MIN_LIMIT && motorSpeed > 0) 
  {
      motorSpeed = MOTOR_SPEED_MIN_LIMIT;
  }

  if (motorSpeed > MOTOR_SPEED_LIMIT)
  {
    motorSpeed = MOTOR_SPEED_LIMIT;
  }
  if (motorSpeed < -MOTOR_SPEED_LIMIT)
  {
    motorSpeed = -MOTOR_SPEED_LIMIT;
  }
}

void setMotorSpeeds()
{
  double angleT = angle;

  if(angleT < 70 and angleT > -70)
    motors.setSpeeds(motorSpeed,motorSpeed);
  else {
    motors.setSpeeds(0,0);
    accError = 0;
    angleError = 0;
  }
}

void calculatePID()
{
  double proportional = angleError * kp;
  double integral =  accError * ki;
  double derivative = -1*(kd*angleRate);

  motorSpeed =-1*(proportional + integral + derivative);
}

void readSensorsACC()
{
  imu.read();
  //PS: We accumelate sensor values (+=), for the avg filter
  
  // Convert from full-scale 1000 deg/s to deg/s.
  angleRate += (imu.g.y - gYZero) / 29;

  // to remove
  int32_t gyroAngle = (angleRate * UPDATE_TIME_MS);
  int32_t accelAngle = (atan2(imu.a.z, imu.a.x) * 57296);
  gyroAngleACC += gyroAngle;
  accelAngleACC += accelAngle;
  angle += gyroAngle + accelAngle;
  // /to remove

  // angle += (angleRate * UPDATE_TIME_MS) + (atan2(imu.a.z, imu.a.x) * 57296);
}

void loop()
{
  static uint8_t avgFilterCount = 0;
  static uint16_t lastMillis;
  
  readSensorsACC();
  avgFilterCount++;

  uint16_t ms = millis();
 
  // Perform the balance updates at 100 Hz.
  if ((uint16_t)(ms - lastMillis) > UPDATE_TIME_MS) 
  { 
      lastMillis = ms;

      angle = (angle/avgFilterCount) * MILLI_DEGREES_TO_DEGREES;
      angleRate /= avgFilterCount;
      
      angleError = 9 - angle;
      accError += angleError;
      prevError = angleError;

      calculatePID();
      checkMotorMinMaxLimits();
      setMotorSpeeds();
      
      //Serial.println(angle);
      //Serial.print(" ");
      //Serial.println(angleRate);

      // to remove
      Serial.print(gyroAngleACC/avgFilterCount * MILLI_DEGREES_TO_DEGREES);
      Serial.print(" ");
      Serial.print(accelAngleACC/avgFilterCount * MILLI_DEGREES_TO_DEGREES); 
      Serial.print(" ");
      Serial.println(angle); 
      gyroAngleACC = 0;
      accelAngleACC = 0;
      // /to remove
     
      avgFilterCount = 0;
      angle = 0;
      angleRate = 0;
  }
}
