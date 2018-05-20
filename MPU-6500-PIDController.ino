#include <LiquidCrystal.h>
#include <Wire.h>
#include <Servo.h>

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
Servo rightMotor;
Servo leftMotor;

float PIDCommand, commandLeft, commandRight, epsilon, previous_epsilon;
float PIDCommandProportionalValue = 0;
float PIDCommandIntegralValue = 0;
float PIDCommandDerivativeValue = 0;

float samplingTime, time, previousTime;

double proportionalConstant = 2.38;
double integralConstant = 0.0022;
double derivativeConstant = 0.32;

double throttle = 1300;
float reference = 0.0;

int gyroX, gyroY, gyroZ;
long accelerationXAxis, accelerationYAxis, accelerationZAxis, acc_total_vector;
int temperature;
long gyroCalibrationXAxis, gyroCalibrationYAxis, gyroCalibrationZAxis;
long loop_timer;
int lcdLoopCounter;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angleRollBuffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angleRollOutput;

void setup() {
  Wire.begin();
  pinMode(13, OUTPUT);
  lcd.begin(16, 2);
  rightMotor.attach(9);
  leftMotor.attach(10);

  lcd.begin(16, 2);
  leftMotor.writeMicroseconds(1000);
  rightMotor.writeMicroseconds(1000);
  initializeMPU6500();
  initializeAccelerometer();
  initializeGyroscope();

  digitalWrite(13, HIGH);

  lcd.begin(16, 2);

  lcd.setCursor(0, 0);
  lcd.print(" 1 DOF BALANCE");
  lcd.setCursor(0, 1);
  lcd.print("     V1.0");

  delay(1500);
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("Calibrating gyro");
  lcd.setCursor(0, 1);
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++) {
    if (cal_int % 125 == 0)lcd.print(".");
    read_mpu_6050_data();
    gyroCalibrationXAxis += gyroX;
    gyroCalibrationYAxis += gyroY;
    gyroCalibrationZAxis += gyroZ;
    delay(3);
  }
  
  gyroCalibrationXAxis /= 2000;
  gyroCalibrationYAxis /= 2000;
  gyroCalibrationZAxis /= 2000;

  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("Roll :");

  digitalWrite(13, LOW);
  loop_timer = micros();
}

void loop() {
  previousTime = time;  
  time = millis();  
  samplingTime = (time - previousTime) / 1000;

  read_mpu_6050_data();

  gyroX -= gyroCalibrationXAxis;
  gyroY -= gyroCalibrationYAxis;
  gyroZ -= gyroCalibrationZAxis;
  
  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  
  angle_pitch += gyroX * 0.0000611;
  angle_roll += gyroY * 0.0000611;

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  
  angle_pitch += angle_roll * sin(gyroZ * 0.000001066);
  angle_roll -= angle_pitch * sin(gyroZ * 0.000001066);

  //Accelerometer angle calculations
  acc_total_vector = sqrt((accelerationXAxis * accelerationXAxis) + (accelerationYAxis * accelerationYAxis) + (accelerationZAxis * accelerationZAxis));
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)accelerationYAxis / acc_total_vector) * 57.296;
  angle_roll_acc = asin((float)accelerationXAxis / acc_total_vector) * -57.296;


  angle_pitch_acc -= 0.0;
  angle_roll_acc -= 0.0;
  if (set_gyro_angles) {
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;
  }
  else {
    angle_pitch = angle_pitch_acc;
    angle_roll = angle_roll_acc;
    set_gyro_angles = true;
  }

  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;
  angleRollOutput = angleRollOutput * 0.9 + angle_roll * 0.1;
  epsilon = angleRollOutput - reference;
  PIDCommandProportionalValue = proportionalConstant * epsilon;

  if (-2.5 < epsilon < 2.5)
  {
    PIDCommandIntegralValue = PIDCommandIntegralValue + (integralConstant * epsilon);
  }

  PIDCommandDerivativeValue = derivativeConstant * ((epsilon - previous_epsilon) / samplingTime);

  PIDCommand = PIDCommandProportionalValue + PIDCommandIntegralValue + PIDCommandDerivativeValue;

  if (PIDCommand < -1000)
  {
    PIDCommand = -1000;
  }
  if (PIDCommand > 1000)
  {
    PIDCommand = 1000;
  }

  commandLeft = throttle + PIDCommand;
  commandRight = throttle - PIDCommand;


  if (commandRight < 1000)
  {
    commandRight = 1000;
  }
  if (commandRight > 2000)
  {
    commandRight = 2000;
  }

  if (commandLeft < 1000)
  {
    commandLeft = 1000;
  }
  if (commandLeft > 2000)
  {
    commandLeft = 2000;
  }

  leftMotor.writeMicroseconds(commandLeft);
  rightMotor.writeMicroseconds(commandRight);

  previous_epsilon = epsilon; //Remember to store the previous epsilon.

  write_LCD();

  while (micros() - loop_timer < 4000);
  loop_timer = micros();
}

void read_mpu_6050_data() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14);
  while (Wire.available() < 14);
  accelerationXAxis = Wire.read() << 8 | Wire.read();
  accelerationYAxis = Wire.read() << 8 | Wire.read();
  accelerationZAxis = Wire.read() << 8 | Wire.read();
  temperature = Wire.read() << 8 | Wire.read();
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();
}

void write_LCD() {
  if (lcdLoopCounter == 7)lcdLoopCounter = 0;
  lcdLoopCounter ++;
  if (lcdLoopCounter == 1) {
    angleRollBuffer = angleRollOutput * 10;
    lcd.setCursor(6, 0);
  }
  if (lcdLoopCounter == 2) {
    if (angleRollBuffer < 0)lcd.print("-");
    else lcd.print("+");
  }
  if (lcdLoopCounter == 3)lcd.print(abs(angleRollBuffer) / 1000);
  if (lcdLoopCounter == 4)lcd.print((abs(angleRollBuffer) / 100) % 10);
  if (lcdLoopCounter == 5)lcd.print((abs(angleRollBuffer) / 10) % 10);
  if (lcdLoopCounter == 6)lcd.print(".");
  if (lcdLoopCounter == 7)lcd.print(abs(angleRollBuffer) % 10);
}

void initializeMPU6500()
{
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

}
void initializeAccelerometer()
{
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
}

void initializeGyroscope()
{
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
}