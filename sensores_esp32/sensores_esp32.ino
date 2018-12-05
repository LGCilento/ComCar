#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>

#define LIMITEACC 2
#define LIMITEGIR 100

LiquidCrystal_I2C  lcd(0x27,2,1,0,4,5,6,7); // 0x27 is the I2C bus address for an unmodified module

long Tmp;
float AcX, AcY, AcZ, varAcX = 0, varAcY = 0, varAcZ = 0;
float vecAcX[10], vecAcY[10], vecAcZ[10];
float rotX, rotY, rotZ;
int magX, magY, magZ;
const int MPU = 0x68;
char k = 0;
int acidente = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  /*      Chamada de configuração do MPU6050  ACC-GIR*/
  Wire.begin();
  setupMPU();
  
  Serial.print("HMC5833L COMPASS SENSOR BEGIN");
  Serial.println();
  setupHMC();

  lcd.setBacklightPin(3,POSITIVE);
  lcd.setBacklight(HIGH); // NOTE: You can turn the backlight off by setting it to LOW instead of HIGH
  lcd.begin(16, 2);
  lcd.clear();
}

void loop() {
  if (!acidente) {
    acelerometro_giroscopio(); //Obtém valores do Acc, Gir e Tmp
    readHMC();
    //acidente_acelerometro();  //Detecta acidente a partir do Acc, Gir e Tmp
  }
  printLCD();
  delay(500);
}


/*=======================================================================================================================================
  ||                                                                                                                                   ||
  ||                           Config. MPU, range dos sensores e endereços necessários para acesso                                     ||
  ||                                                                                                                                   ||
  =====================================================================================================================================*/
void setupMPU() {
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4)
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5)
  Wire.write(0b00000000); //Setting the accel to +/- 2g //Bit 3 e 4
  Wire.endTransmission();
}

/*
 * Configure HMC to continuously measure
 */
void setupHMC(){
  Wire.beginTransmission(0b0001101); //start talking
//  Wire.write(0x09); // Set the Register
//  Wire.write(0x01); // Tell the HMC5883 to Continuously Measure
//  Wire.endTransmission();

  Wire.beginTransmission(0b0001101); //start talking
  // Tell the HMC5883 to Continuously Measure
  Wire.write(0x0B); // Set the Register
  Wire.write(0x01); // Define Set/Reset period
  Wire.write(0x09); // Set the Register
  Wire.write(0x1D); // 
  Wire.endTransmission();
}

/*=======================================================================================================================================
  ||                                                                                                                                   ||
  ||                           Máximo de um intervalo do acelerômetro em determinado eixo                                              ||
  ||                                                                                                                                   ||
  =====================================================================================================================================*/
float getMaxAcc(float vecAcc[]) {
  float maxAcc = -5;
  for (int i = 0; i < 10; i++)
    if (vecAcc[i % 10] > maxAcc) {
      maxAcc = vecAcc[i % 10];
    }
  return maxAcc;
}
/*=======================================================================================================================================
  ||                                                                                                                                   ||
  ||                           Mínimo de um intervalo do acelerômetro em determinado eixo                                              ||
  ||                                                                                                                                   ||
  =====================================================================================================================================*/
float getMinAcc(float vecAcc[]) {
  float minAcc = 5;
  for (int i = 0; i < 10; i++)
    if (vecAcc[i % 10] < minAcc) {
      minAcc = vecAcc[i % 10];
    }
  return minAcc;
}

/*=======================================================================================================================================
  ||                                                                                                                                   ||
  ||                                                  Leitura dos sensores ACC, GIR, TMP                                               ||
  ||                                                                                                                                   ||
  =====================================================================================================================================*/
void acelerometro_giroscopio()
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  //Solicita os dados do sensor
  Wire.requestFrom(MPU, 14, true);
  //Armazena o valor dos sensores nas variaveis correspondentes
  AcX = Wire.read() << 8 | Wire.read(); //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  rotX = Wire.read() << 8 | Wire.read(); //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  rotY = Wire.read() << 8 | Wire.read(); //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  rotZ = Wire.read() << 8 | Wire.read(); //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  processAccGir();
}

/*=======================================================================================================================================
  ||                                                                                                                                   ||
  ||                                                  Alteração do range dos sensores                                                  ||
  ||                                                                                                                                   ||
  =====================================================================================================================================*/
void processAccGir() {
  AcX = AcX / 16384.0;
  AcY = AcY / 16384.0;
  AcZ = AcZ / 16384.0;
  Serial.print("AcX="); Serial.print(AcX);
  Serial.print(" AcY="); Serial.print(AcY);
  Serial.print(" AcZ="); Serial.print(AcZ);
  Serial.println();
  
  Tmp = (Tmp / 340) + 36.53; //Fórmula para temperatura do acelerometro
  
  rotX = rotX / 131.0;
  rotY = rotY / 131.0;
  rotZ = rotZ / 131.0;
  Serial.print("rotX="); Serial.print(rotX);
  Serial.print(" rotY="); Serial.print(rotY);
  Serial.print(" rotZ="); Serial.print(rotZ);
  Serial.println();

  // if ((abs(rotX)) > LIMITEGIR || (abs(rotY)) > LIMITEGIR || (abs(rotZ)) > LIMITEGIR) {

  //   acidente = 1;
  // }
  
  delay(500);

}

/*
 * Read HMC
 */
void readHMC(){
  //Read the data.. 2 bytes for each axis
  Wire.requestFrom(0b0001101, 6);
  
  magX = Wire.read()<<8; //MSB  x 
  magX |= Wire.read(); //LSB  x
  magZ = Wire.read()<<8; //MSB  z
  magZ |= Wire.read(); //LSB z
  magY = Wire.read()<<8; //MSB y
  magY |= Wire.read(); //LSB y
}
/*=======================================================================================================================================
  ||                                                                                                                                   ||
  ||                               Detecção de acidente a partir da variação do ACC em um determinado intervalo                        ||
  ||                                                                                                                                   ||
  =====================================================================================================================================*/
void acidente_acelerometro() {
  vecAcX[k] = AcX;
  vecAcY[k] = AcY;
  vecAcZ[k] = AcZ;
  if (k == 10) {
    varAcX = (getMaxAcc(vecAcX) - getMinAcc(vecAcX)); //Pd ser colocado direto do IF abaixo, mas sera mantido para melhorar legibilidade
    varAcY = (getMaxAcc(vecAcY) - getMinAcc(vecAcY));
    varAcZ = (getMaxAcc(vecAcZ) - getMinAcc(vecAcZ));

    //Serial.println(varAcX);
    if (varAcX > LIMITEACC && varAcY > LIMITEACC && varAcZ > LIMITEACC) {
      acidente = 1;
    }
    k = 0;

  }
  else {
    k++;
  }
}

/*
 * Print measures in LCD
 */
void printLCD(){
  lcd.setCursor(0,0);
  lcd.print("Ac ");
  lcd.print((int)AcX);
  lcd.print(" ");
  lcd.print((int)AcY);
  lcd.print(" ");
  lcd.print((int)AcZ);
  lcd.print(" Gr ");
  lcd.print((int)rotX);
  
  lcd.setCursor(0,1);
  lcd.print((int)rotY);
  lcd.print(" ");
  lcd.print((int)rotZ);
  lcd.print(" Mg ");
  lcd.print(magX);
  lcd.print(" ");
  lcd.print(magY);
  lcd.print(" ");
  lcd.print(magZ);
}
