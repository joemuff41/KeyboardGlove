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
const int DELAY = 10;                       //Delay between readings of MPU (milliseconds)
const int CALIBRATION_TIME = 5;             //Time to throw away (seconds)

int currentMPU;                             //The MPU currently set low

float gyroOffsets[3][NUM_MPUS];             //Offsets in x, y, and z for each MPU to account for gyro noise

const int ADO_PINS[NUM_MPUS] = {9, 12, 7};     //Define the ADO pins for each MPU

float theta[3][NUM_MPUS];                   //The angle each MPU is at
                                            /*      MPU 1   MPU2    MPU3    ...
                                             * x  [                             ]
                                             * y  [                             ]
                                             * z  [                             ]
                                             */
float deltaTheta[3][NUM_MPUS];              //Change in theta as measured by gyro
float gForce[3][NUM_MPUS];
float accTilt[NUM_MPUS];
float accTiltOffsets[NUM_MPUS];
float peak;
float accPeak;
float valley;
float lastTheta;
int countRepeats;
int state;                          //Value of 0 is static, 1 is rising, 2 is falling
int accState;
boolean change;                     //True if going from falling to rising or falling to static
boolean accChange;                  //
int s;
int sequence;                       //Set to 1 if going from rising to steady, set to 2 if falling and it is 1 (supposed to catch a peak)
float steadyChange;                 //Keep track of how much angle changes over the course of being "steady" to categorize as an actual rise or fall that takes a while
float totalAcc[NUM_MPUS];           //Total magnitude of acceleration
float accSumSquare[NUM_MPUS];       //Sum of square of accelerations
float accSumSquareChange[NUM_MPUS]; //Change from last iteration
float accOffsets[NUM_MPUS];
int countSinceLastGyroLetter;
int countSinceLastAccLetter;
char lastAccLetter;
float calibrationPeakGyro;          //Used to determine set points for calibration of letters for gyro
float calibrationPeakAcc;           //Used to determine set points for calibration of letters for accelerometer
float setPointsGyro[2];             //Set points for between 'c' and 'd' [1], and between 'd' and 'e' [0]
float setPointsAcc[2];              //Set points for between 'c' and 'd' [1], and between 'd' and 'e' [0]
char calibrationLetters[2];         //Used to calibrate gyro and acclerometer keyboards

void setup(){

  currentMPU = 0;
  peak = 0;
  valley = 0;
  lastTheta = 0;
  countRepeats = 0;
  state = 0;
  s = 0;
  sequence = 0;
  steadyChange = 0;
  
  //Initialize the starting angles, gyro offsets, g-forces, and accelerometer tilt
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < NUM_MPUS; j++) {
      theta[i][j] = 0;
      deltaTheta[i][j] = 0;
      gyroOffsets[i][j] = 0;
      gForce[i][j] = 0;
      accTilt[j] = 0;
      totalAcc[j] = 0;

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

  calibrate();
  learnSetPoints();
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
 * Establish gyro offsets for each MPU to account for gyro noise
 */
void calibrate() {

  int calTime = 3000;                   //Time to take readings over (milliseconds)
  Serial.println("");
  Serial.print("Calibrating...");

  //Throw away beginning data
  for (int i = CALIBRATION_TIME ; i > 0; i--) {
    Serial.print(i + calTime / 1000); Serial.print("...");
    //Throw away 20 readings each second
    for (int j = 0; j < 20; j++) {
      for (int k = 0; k < NUM_MPUS; k++) {
        switchAcc(k);

        Wire.beginTransmission(MPU_addr);
        Wire.write(0x3B);
        Wire.endTransmission();
        Wire.requestFrom(MPU_addr,6); //Request accelerometer Registers (3B - 40)
        Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
        Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
        Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
        
        Wire.beginTransmission(MPU_addr);
        Wire.write(0x43);
        Wire.endTransmission();
        Wire.requestFrom(MPU_addr,6); //Request Gyro Registers (43 - 48)
        Wire.read()<<8|Wire.read();
        Wire.read()<<8|Wire.read();
        Wire.read()<<8|Wire.read();
      }

      delay(50);
    }
  }

  int numReadings = 40;                 //Number of calibration readings to average per second

  float sumGyroReadings[3][NUM_MPUS];       //Sum of the gyro readings over the time period
  float sumAccReadings[NUM_MPUS];
  
  //Initialize the sumReadings array
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < NUM_MPUS; j++) {
      sumGyroReadings[i][j] = 0;
      sumAccReadings[j] = 0;
    }
  }

  int calDelay = 1000/numReadings;         //Delay to use between readings during the calibration time period
  float accX = 0;
  float accY = 0;
  float accZ = 0;

  //Get data over the calibration time
  for (int i = (calTime / 1000); i > 0; i--) {
    Serial.print(i); Serial.print("...");
    
    for (int j = 0; j < numReadings; j++) {
      for (int k = 0; k < NUM_MPUS; k++) {

        switchAcc(k);

        //Accelerometer
        Wire.beginTransmission(MPU_addr);
        Wire.write(0x3B);
        Wire.endTransmission();
        Wire.requestFrom(MPU_addr,6); //Request accelerometer Registers (3B - 40)


        accX = ((float)(Wire.read()<<8|Wire.read())) / 16384.0;  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
        accY = ((float)(Wire.read()<<8|Wire.read())) / 16384.0;  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
        accZ = ((float)(Wire.read()<<8|Wire.read())) / 16384.0;  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
        accOffsets[k] += sqrt(accX * accX + accY * accY + accZ * accZ); 
        sumAccReadings[k] += atan2(accY, accZ) * (180 / M_PI);

        //Gyro
        Wire.beginTransmission(MPU_addr);
        Wire.write(0x43);
        Wire.endTransmission();
        Wire.requestFrom(MPU_addr,6); //Request Gyro Registers (43 - 48)

        sumGyroReadings[0][k] += ((float)(Wire.read()<<8|Wire.read())) / 131.0;
        sumGyroReadings[1][k] += ((float)(Wire.read()<<8|Wire.read())) / 131.0;
        sumGyroReadings[2][k] += ((float)(Wire.read()<<8|Wire.read())) / 131.0;
      }
      delay(calDelay);
    }
  }

  Serial.println("");

  for (int i = 0; i < NUM_MPUS; i++) {
    //Average the readings over the calibration time

    accOffsets[i] = accOffsets[i] / ((float) (numReadings*(calTime / 1000)));
    accTiltOffsets[i] = sumAccReadings[i] / ((float) (numReadings*(calTime / 1000)));
    
    gyroOffsets[0][i] = sumGyroReadings[0][i] / ((float) (numReadings*(calTime / 1000)));
    gyroOffsets[1][i] = sumGyroReadings[1][i] / ((float) (numReadings*(calTime / 1000)));
    gyroOffsets[2][i] = sumGyroReadings[2][i] / ((float) (numReadings*(calTime / 1000)));

    //Serial.print("Accelerometer tilt offset "); Serial.print(i); Serial.print(" = "); Serial.println(accTiltOffsets[i]);
  }

}

void learnSetPoints() {
  boolean done = false;
  int gyroLetters = 0;
  int accLetters = 0;
  float letterPeaksGyro[3] = {0, 0, 0};
  float letterPeaksAcc[3] = {0, 0, 0};
  int numPoints = 3;
  
  Serial.print("Type d "); Serial.print(numPoints); Serial.println(" times");

  while (!done) {
    calibrationLoopBody();
    if (gyroLetters < 3 && calibrationLetters[0] != 0) {
      gyroLetters++;
      letterPeaksGyro[1] += calibrationPeakGyro;
    }
    if (accLetters < 3 && calibrationLetters[1] != 0) {
      accLetters++;
      letterPeaksAcc[1] += calibrationPeakAcc;
    }

    if (gyroLetters >= 3 && accLetters >= 3) {
      done = true;
      letterPeaksGyro[1] = letterPeaksGyro[1] / 3;
      letterPeaksAcc[1] = letterPeaksAcc[1] / 3;
    }
  }

  gyroLetters = 0;
  accLetters = 0;
  done = false;
  Serial.print("Type e "); Serial.print(numPoints); Serial.println(" times");
  while (!done) {
    calibrationLoopBody();
    if (gyroLetters < 3 && calibrationLetters[0] != 0) {
      gyroLetters++;
      letterPeaksGyro[0] += calibrationPeakGyro;
    }
    if (accLetters < 3 && calibrationLetters[1] != 0) {
      accLetters++;
      letterPeaksAcc[0] += calibrationPeakAcc;
    }

    if (gyroLetters >= 3 && accLetters >= 3) {
      done = true;
      letterPeaksGyro[0] = letterPeaksGyro[0] / 3;
      letterPeaksAcc[0] = letterPeaksAcc[0] / 3;
    }
  }
  
  gyroLetters = 0;
  accLetters = 0;
  done = false;
  Serial.print("Type c "); Serial.print(numPoints); Serial.println(" times");
  while (!done) {
    calibrationLoopBody();
    if (gyroLetters < 3 && calibrationLetters[0] != 0) {
      gyroLetters++;
      letterPeaksGyro[2] += calibrationPeakGyro;
    }
    if (accLetters < 3 && calibrationLetters[1] != 0) {
      accLetters++;
      letterPeaksAcc[2] += calibrationPeakAcc;
    }

    if (gyroLetters >= 3 && accLetters >= 3) {
      done = true;
      letterPeaksGyro[2] = letterPeaksGyro[2] / 3;
      letterPeaksAcc[2] = letterPeaksAcc[2] / 3;
    }
  }

  setPointsGyro[0] = (letterPeaksGyro[0] + letterPeaksGyro[1]) * 0.42;
  setPointsGyro[1] = (letterPeaksGyro[1] + letterPeaksGyro[2]) * 0.58;

  setPointsAcc[0] = (letterPeaksAcc[0] + letterPeaksAcc[1]) * 0.42;
  setPointsAcc[1] = (letterPeaksAcc[1] + letterPeaksAcc[2]) * 0.58;
}

void calibrationLoopBody() {
  for (int i = 2; i < NUM_MPUS; i++) {
    switchAcc(i);
    
    recordGyro();
    recordAccelerometer();
  
    processGyroData(i);
    processAccData(i);

    sumSquareAcc(i);
    complementaryFilter(i);


    calibrationLetters[0] = analyzeKeyboard(true);
    calibrationLetters[1] = analyzeKeyboardAcc(true);

  }
  delay(DELAY);
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

void sumSquareAcc(int acc) {
  totalAcc[acc] = (sqrt((gForce[0][acc] * gForce[0][acc]) + (gForce[1][acc] * gForce[1][acc]) + (gForce[2][acc] * gForce[2][acc])));

  accSumSquareChange[acc] = accSumSquare[acc];
  accSumSquare[acc] += ((totalAcc[acc] - accOffsets[acc]) * (totalAcc[acc] - accOffsets[acc]) );
  accSumSquareChange[acc] = accSumSquare[acc] - accSumSquareChange[acc];
  
}

/*
 * Apply a complementary filter to stabilize the angle
 */
void complementaryFilter(int acc) {
  
  if (totalAcc[acc] > 0.5 && totalAcc[acc] < 2.0) {
    theta[0][acc] = 0.97 * (theta[0][acc] + deltaTheta[0][acc]) + 0.03*(accTilt[acc]);
    theta[1][acc] = 1.00 * (theta[1][acc] + deltaTheta[1][acc]);
    theta[2][acc] = 1.00 * (theta[2][acc] + deltaTheta[2][acc]);
  }
  else {
    theta[0][acc] = (theta[0][acc] + deltaTheta[0][acc]);
    theta[1][acc] = (theta[1][acc] + deltaTheta[1][acc]);
    theta[2][acc] = (theta[2][acc] + deltaTheta[2][acc]);
  }
}

void keyboardRising() {
  steadyChange = 0;
  //peak = theta[0][2];
  countRepeats = 0;

  if (state != 2) {
    //change = true;
    lastTheta = theta[0][2];
  }
  state = 1;
}

void keyboardFalling() {
  steadyChange = 0;
  countRepeats = 0;

  //If there was a quick rise in angle then dropping down to the actual peak (noise), make the peak the next value
  //This also should be a letter indication
  if (state == 1) {
    //change = true;
    peak = theta[0][2];
  }
  else if (state == 2) {
    //Only set the valley if it was previously dropping
    valley = theta[0][2];
  }
  if (sequence == 1) {
    sequence = 2;
  }
  lastTheta = theta[0][2];
  state = 2;  
}

char analyzeKeyboardAcc(boolean isCalibrating) {
  char letter = 0;

  if (accSumSquareChange[2] < 0.1) {
    if (accState == 1){
      countRepeats++;
      if (countRepeats == 3) {
        accChange = true;
        accState = 0;
        countRepeats = 0;
      }
    }
  }
  else {
    accPeak = theta[0][2];
    accState = 1;
    countRepeats = 0;
  }

  if (accChange) {

    if (isCalibrating) {
      calibratePeakAcc();
      letter = "!";
    }
    else{
      if (accPeak > setPointsAcc[1]) {
        letter = 'C';
      }
      else if (accPeak > setPointsAcc[0]) {
        letter = 'D';
      }
      else {
        letter = 'E';
      }
    }
    
    accChange = false;
  }
  else {

  }
  
  return letter;
}

char analyzeKeyboard(boolean isCalibrating) {
  
  char letter = 0;

  
  change = false;

  //Rising
  if ((theta[0][2] - lastTheta) > 0.5) {
    keyboardRising();
  }
  //Falling
  else if ((lastTheta - theta[0][2]) > 0.5) {
    keyboardFalling();
  }
  else {
    steadyChange += theta[0][2] - lastTheta;
    if (steadyChange > 0.5){
      keyboardRising();
    }
    else if (steadyChange < -0.5) {
      keyboardFalling();
    }
    else {
    
      //If going from falling to static, 
      if (state == 2) {

        lastTheta = theta[0][2];
      }

      //If going from rising to static, set the next value to the peak
      //This is a letter indication
      else if (state == 1) {
        change = true;
        peak = theta[0][2];
        lastTheta = theta[0][2];
        sequence = 1;
      }
    }
    state = 0;
  }

  if (sequence == 2) {
    sequence = 0;
    
    if ((peak - valley) > 2.5) {

      if (state == 1) {
        valley = lastTheta;
      }
      if (isCalibrating){
        calibratePeakGyro();
        letter = '!';
      }
      else {
        if (peak > setPointsGyro[1]) {
          letter = 'c';
        }
        else if (peak > setPointsGyro[0]) {
          letter = 'd';
        }
        else {
          letter = 'e';
        }
      }
  
      valley = peak;
    }
    
  }
  
  return letter;

}

void calibratePeakGyro() {
  calibrationPeakGyro = peak;
}

void calibratePeakAcc() {
  calibrationPeakAcc = accPeak;
}

void loop(){

  for (int i = 2; i < NUM_MPUS; i++) {
    switchAcc(i);
    
    recordGyro();
    recordAccelerometer();
  
    processGyroData(i);
    processAccData(i);

    sumSquareAcc(i);
    complementaryFilter(i);
    /*
    theta[0][i] += deltaTheta[0][i];
    theta[1][i] += deltaTheta[1][i];
    theta[2][i] += deltaTheta[2][i];
    */

    //totalAcc = 10*(sqrt((gForce[0][i] * gForce[0][i]) + (gForce[1][i] * gForce[1][i]) + (gForce[2][i] * gForce[2][i])));

    char letter = analyzeKeyboard(false);
    char letterAcc = analyzeKeyboardAcc(false);

    if (letter != 0 && countSinceLastAccLetter > 10) {
      Serial.print(letter);
      countSinceLastGyroLetter = 0;
    }
    else {
      countSinceLastGyroLetter++;
    }
    
    if (letterAcc != 0 && countSinceLastGyroLetter > 30) {
      if (!(countSinceLastAccLetter < 40 && letterAcc == lastAccLetter)) {
          //Serial.print(countSinceLastGyroLetter);
          Serial.print(letterAcc);
          countSinceLastAccLetter = 0;
          lastAccLetter = letterAcc;
      }
    }
    else {
      countSinceLastAccLetter++;
    }

    
    //Serial.print(s); Serial.print("\t");
    /*
    Serial.print(theta[0][i]); Serial.print("\t");
    Serial.print(accSumSquare[i]); Serial.print("\t");
    Serial.print(accSumSquareChange[i]); Serial.print("\t");
    Serial.print(peak); Serial.print("\t");
    */
    /*
    Serial.print(peak); Serial.print("\t");
    Serial.print(valley); Serial.print("\t");
    Serial.print(state);  Serial.print("\t");
    Serial.print(sequence); Serial.print("\t");
    */
    
    if (letter != 0) {
      //Serial.print(letter); //Serial.print("\t");
    }
    
    if (letter != 0) {
      //Serial.print("\n\n\n"); 
      //Serial.print(letter); 
      //Serial.print("\n\n\n");
    }
    //Serial.print(theta[1][i]); Serial.print("\t");
    //Serial.print(theta[2][i]); Serial.print("\t");
    //Serial.print(accTilt[i]); Serial.print("\t");
    //Serial.print("AcX = "); Serial.print(AcX); Serial.print("\t");
    //Serial.print("AcY = "); Serial.print(AcY); Serial.print("\t");
    //Serial.print("AcZ = "); Serial.print(AcZ); Serial.print("\t");
    //Serial.print(totalAcc); Serial.print("\t");
    //Serial.print(theta[1]); Serial.print("\t");
    //Serial.println(theta[2]);

  }

  //jSerial.println("");

  s += DELAY;
  delay(DELAY);
}

/*
 * Integrate the gyro rotational speed over the delay to find the change in angle.
 * Incorporate the calibration offsets.
 */
void processGyroData(int acc) {

  deltaTheta[0][acc] = ((((float)GyX) / 131.0) - gyroOffsets[0][acc]) * (((float)DELAY)/1000);
  deltaTheta[1][acc] = ((((float)GyY) / 131.0) - gyroOffsets[1][acc]) * (((float)DELAY)/1000);
  deltaTheta[2][acc] = ((((float)GyZ) / 131.0) - gyroOffsets[2][acc]) * (((float)DELAY)/1000);

  //Serial.print("Gyro Offset X: "); Serial.println(gyroOffsets[0]);

}

/*
 * Convert the accelerometer data to g-force
 */
void processAccData(int acc) {
  gForce[0][acc] = ((float)AcX) / 16384.0;
  gForce[1][acc] = ((float)AcY) / 16384.0;
  gForce[2][acc] = ((float)AcZ) / 16384.0;

  //float test = gForce[1][acc]*gForce[1][acc];
  //Serial.print("Test: "); Serial.println(test);
  accTilt[acc] = 1*(atan2(gForce[1][acc], gForce[2][acc]) * (180 / M_PI)- accTiltOffsets[acc]);

}

