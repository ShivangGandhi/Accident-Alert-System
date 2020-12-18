#include <Wire.h>
#include <SoftwareSerial.h>
#include "MPU6050_6Axis_MotionApps20.h"
SoftwareSerial BTserial(3, 2); //   TX|RX
#define button 9
#define shock_pin 5
static boolean flag = true;
MPU6050 mpu;
// MPU6050 Slave Device Address
const uint8_t MPU6050SlaveAddress = 0x68;

// Select SDA and SCL pins for I2C communication
const uint8_t scl = A5;
const uint8_t sda = A4;

// sensitivity scale factor respective to full scale setting provided in datasheet
const uint16_t AccelScaleFactor = 16384;
const uint16_t GyroScaleFactor = 131;

// MPU6050 few configuration register addresses
const uint8_t MPU6050_REGISTER_SMPLRT_DIV   =  0x19;
const uint8_t MPU6050_REGISTER_USER_CTRL    =  0x6A;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1   =  0x6B;
const uint8_t MPU6050_REGISTER_PWR_MGMT_2   =  0x6C;
const uint8_t MPU6050_REGISTER_CONFIG       =  0x1A;
const uint8_t MPU6050_REGISTER_GYRO_CONFIG  =  0x1B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG =  0x1C;
const uint8_t MPU6050_REGISTER_FIFO_EN      =  0x23;
const uint8_t MPU6050_REGISTER_INT_ENABLE   =  0x38;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H =  0x3B;
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET  = 0x68;

int16_t AccelX, AccelY, AccelZ, Temperature, GyroX, GyroY, GyroZ;

int buttonState = 0;
static boolean flag_button=false;
float time=0.0;
static boolean acci_detec=false;
float x=0.0;

void setup() {
  Serial.begin(9600);
  BTserial.begin(9600);
  Wire.begin();
  // sgps.begin(9600);
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
  pinMode(command,OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(button, INPUT);
  pinMode(shock_pin, INPUT); 
}

long TP_init() {
  delay(10);
  long measurement = pulseIn (shock_pin, HIGH); //wait for the pin to get HIGH and returns measurement
  return measurement;
}

void loop() 
{
  if(flag_button)
  {
     buttonState = digitalRead(button);
  }
  
  if (flag)
  {
    double Ax, Ay, Az, T, Gx, Gy, Gz, shock;

    Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);

    //divide each with their sensitivity scale factor
    Ax = (double)AccelX / AccelScaleFactor;
    Ay = (double)AccelY / AccelScaleFactor;
    Az = (double)AccelZ / AccelScaleFactor;
    T = (double)Temperature / 340 + 36.53; //temperature formula
    Gx = (double)GyroX / GyroScaleFactor;
    Gy = (double)GyroY / GyroScaleFactor;
    Gz = (double)GyroZ / GyroScaleFactor;
    shock = TP_init();

    Serial.print("Ax: "); Serial.print(Ax);
    Serial.print(" Ay: "); Serial.print(Ay);
    Serial.print(" Az: "); Serial.print(Az);
    Serial.print(" T: "); Serial.print(T);
    Serial.print(" Gx: "); Serial.print(Gx);
    Serial.print(" Gy: "); Serial.print(Gy);
    Serial.print(" Gz: "); Serial.print(Gz);
    Serial.print(" Shock: "); Serial.println(shock);
    
    if ((Gy >= 10.0||Gy<=-10.0||Gx>=10.0||Gx<=-10 || shock>8000 || Ax>0.25 || Ax<-0.25 || Ay>0.25 || Ay<-0.20) && shock>100)
    {
      Serial.println("ANGLE CHANGE IN Z DIRECTION IS SUDDEN!!!!");
      BTserial.println("1");
      x=millis();
      flag_button=true;
      acci_detec=true;
      flag=false;
    }
  }

  if(acci_detec)
  {
    time=millis()-x;
    if (buttonState == 1 && time <= 15000)
    {
      flag = true;
      acci_detec = false;
      flag_button=false;
      digitalWrite(command, LOW);
      BTserial.println("0");
      buttonState = 0;
    }
    
    if(time>15000)
    {
      flag=false;
       digitalWrite(command, HIGH);        
       Serial.println(F("Accident Ho gaya !!!"));
       BTserial.println("11");
       exit(0);
    }
  }  
  delay(100);
}

void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}

// read all 14 register
void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t)14);
  AccelX = (((int16_t)Wire.read() << 8) | Wire.read());
  AccelY = (((int16_t)Wire.read() << 8) | Wire.read());
  AccelZ = (((int16_t)Wire.read() << 8) | Wire.read());
  Temperature = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroX = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroY = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroZ = (((int16_t)Wire.read() << 8) | Wire.read());
}

//configure MPU6050
void MPU6050_Init() {
  delay(150);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_CONFIG, 0x00);//set +/-250 degree/second full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x00);// set +/- 2g full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, 0x00);
}
