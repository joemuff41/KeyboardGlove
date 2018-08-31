// MPU-6050
// By Andrew Christovich
// August 5, 2018
// Public Domain
#include<Wire.h>
#include <math.h>

#define M_PI 3.14159265359

const int MPU_addr=0x68;                    //I2C address of the MPU-6050
const int NUM_MPUS = 3;                     //Number of accelerometers
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;        //Current raw readings from MPU
const int DELAY = 1;                       //Delay between readings of MPU (milliseconds)

int currentMPU;                             //The MPU currently set low
float gyroVal[3][NUM_MPUS];                 //Gyro values
float accVal[3][NUM_MPUS];                  //Accelerometer values

float test;

const int ADO_PINS[NUM_MPUS] = {9, 12, 7};     //Define the ADO pins for each MPU

union floating {
  float val;
  unsigned char b[4];
} byteArray;

void setup(){

  test = 0;

  currentMPU = 0;

  //Initialize the gyro and accelerometer arrays
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < NUM_MPUS; j++) {
      gyroVal[i][j] = 0;
      accVal[i][j] = 0;

      pinMode(ADO_PINS[j], OUTPUT);       //Make the ADO pins outputs
      digitalWrite(ADO_PINS[j], HIGH);    //Set all the ADO pins to high
      
    }
  }

  Wire.begin();

  //Initial startup on MPU's
  for (int i = 0; i < NUM_MPUS; i++) {

    switchAcc(i);
    
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0);     // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
  
    Wire.beginTransmission(MPU_addr); //I2C address of the MPU
    Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4)
    Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s
    Wire.endTransmission();
  
    Wire.beginTransmission(MPU_addr); //I2C address of the MPU
    Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5)
    Wire.write(0b00000000); //Setting the accel to +/- 2g
    Wire.endTransmission();
  }

  Serial.begin(115200);
}

/*
 * Make the given MPU active
 * @param mpu     The mpu to make active
 */
void switchAcc(int mpu) {
  
  digitalWrite(ADO_PINS[currentMPU], HIGH);   //Set the current MPU to high
  digitalWrite(ADO_PINS[mpu], LOW);           //Set the new MPU to low
  
  currentMPU = mpu;                           //Update the current MPU to the new one
}

/*
 * Take the measurements from the gyro
 */
void recordGyro() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(MPU_addr,6); //Request Gyro Registers (43 - 48)
  
  
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

/*
 * Take the measurements from the accelerometer
 */
void recordAccelerometer() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(MPU_addr,6); //Request accelerometer Registers (3B - 40)
  
  
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
}

void loop(){
  int8_t i;
  char mpu;

  for (i = 0; i < NUM_MPUS; i++) {
    switchAcc(i);
    
    recordGyro();
    recordAccelerometer();
  
    processGyroData(i);
    processAccData(i);


    /*
    Serial.print('a'); Serial.print(i); Serial.print('_'); Serial.print(gyroVal[0][i]); Serial.print('_');
    Serial.print('b'); Serial.print(i); Serial.print('_'); Serial.print(gyroVal[1][i]); Serial.print('_');
    Serial.print('c'); Serial.print(i); Serial.print('_'); Serial.print(gyroVal[2][i]); Serial.print('_');

    Serial.print('x'); Serial.print(i); Serial.print('_'); Serial.print(accVal[0][i]); Serial.print('_');   
    Serial.print('y'); Serial.print(i); Serial.print('_'); Serial.print(accVal[1][i]); Serial.print('_');    
    Serial.print('z'); Serial.print(i); Serial.print('_'); Serial.print(accVal[2][i]); Serial.print('_'); 
    Serial.print(test);
    */
    
    mpu = (i + 48); 
    byteArray.val = gyroVal[0][i];
    Serial.write('a'); Serial.write(mpu); Serial.write(byteArray.b, 4);
    byteArray.val = gyroVal[1][i];
    Serial.write('b'); Serial.write(mpu); Serial.write(byteArray.b, 4);
    byteArray.val = gyroVal[2][i];
    Serial.write('c'); Serial.write(mpu); Serial.write(byteArray.b, 4);

    byteArray.val = accVal[0][i];
    Serial.write('d'); Serial.write(mpu); Serial.write(byteArray.b, 4);
    byteArray.val = accVal[1][i];
    Serial.write('e'); Serial.write(mpu); Serial.write(byteArray.b, 4);
    byteArray.val = accVal[2][i];
    Serial.write('f'); Serial.write(mpu); Serial.write(byteArray.b, 4);
    
  }

  //Serial.println("");

  test += DELAY;
  delay(DELAY);
}

/*
 * Convert to deg/sec
 */
void processGyroData(int acc) {

  
  gyroVal[0][acc] = (((float)GyX) / 131.0);
  gyroVal[1][acc] = (((float)GyY) / 131.0);
  gyroVal[2][acc] = (((float)GyZ) / 131.0);

}

/*
 * Convert the accelerometer data to g-force
 */
void processAccData(int acc) {
  accVal[0][acc] = ((float)AcX) / 16384.0;
  accVal[1][acc] = ((float)AcY) / 16384.0;
  accVal[2][acc] = ((float)AcZ) / 16384.0;

}

