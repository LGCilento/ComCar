#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>

#define LIMITEACC 2
#define LIMITEGIR 100
#define declinationAngle 0.3846
#define leftEncoder 19
#define rightEncoder 18

LiquidCrystal_I2C  lcd(0x27,2,1,0,4,5,6,7); // 0x27 is the I2C bus address for an unmodified module

// WIFI
const char* ssid     = "comcar";     // your network SSID (name of wifi network)
const char* password = "comcar2018"; // your network password

const char*  server = "192.168.43.202";  // Server URL
const uint16_t port = 23;
WiFiClient client;

long Tmp;
float AcX, AcY, AcZ;
float vecAcX[10], vecAcY[10], vecAcZ[10];
float rotX, rotY, rotZ;
float bearing;
int leftTurns, rightTurns;
const int MPU = 0x68;
char k = 0;
int acidente = 0;

uint8_t i = 0;
char buff[50];

void setup() {
  Serial.begin(115200);
  pinMode(leftEncoder,INPUT);
  pinMode(rightEncoder,INPUT);
    Wire.begin();
    setupMPU(); // Chamada de configuração do MPU6050  ACC-GIR
  setupHMC_continous();
    
    WiFi.begin(ssid, password);
  
    // attempt to connect to Wifi network:
    while (WiFi.status() != WL_CONNECTED) {
      Serial.print(".");
      // wait 10 miliseconds for re-trying
      delay(50);
    }

      lcd.setBacklightPin(3,POSITIVE);
      lcd.setBacklight(HIGH); // NOTE: You can turn the backlight off by setting it to LOW instead of HIGH
      lcd.begin(16, 2);
      lcd.clear();

  leftTurns = 0;
  rightTurns = 0;
}

void loop() {
  unsigned long tempo;
  int leftEnc, rightEnc;

  if (!acidente) {
    acelerometro_giroscopio(); //Obtém valores do Acc, Gir e Tmp
    readHMC();
    // acidente_acelerometro();  //Detecta acidente a partir do Acc, Gir e Tmp
  }

  tempo = millis();
  while(millis()-tempo < 100){
    leftEnc = digitalRead(leftEncoder);
    rightEnc = digitalRead(rightEncoder);
    
    if(leftEnc==HIGH && rightEnc==HIGH){
      leftTurns++;
      rightTurns++;
    } else if(leftEnc==HIGH){
      leftTurns++;
    } else if(rightEnc==HIGH){
      rightTurns++;
    }
  }
    
  if (!client.connect(server, port)){
    Serial.println("Connection failed!");
  }
  else {
    Serial.println("Connected to server!");
    if(acidente){
      
    }
    else{
      sprintf(buff,"COM-0001N%2.2f%2.2f%2.2f%3.1f%d%d",AcX,AcY,AcZ,bearing,leftTurns/2,rightTurns/2);
      client.write(buff);
      Serial.println(buff);

      leftTurns = 0;
      rightTurns = 0;
      delay(50);
      client.stop();
    }
  }
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
void setupHMC_continous(){
  // Tell the HMC5883 to Continuously Measure
  Wire.beginTransmission(0b0001101); //start talking
  Wire.write(0x0B); // Set the Register
  Wire.write(0x01); // Define Set/Reset period
  Wire.endTransmission();
  Wire.beginTransmission(0b0001101);
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
//  Serial.print("AcX="); Serial.print(AcX);
//  Serial.print(" AcY="); Serial.print(AcY);
//  Serial.print(" AcZ="); Serial.print(AcZ);
//  Serial.println();
  
  Tmp = (Tmp / 340) + 36.53; //Fórmula para temperatura do acelerometro
  
  rotX = rotX / 131.0;
  rotY = rotY / 131.0;
  rotZ = rotZ / 131.0;
//  Serial.print("rotX="); Serial.print(rotX);
//  Serial.print(" rotY="); Serial.print(rotY);
//  Serial.print(" rotZ="); Serial.print(rotZ);
//  Serial.println();

  if ((abs(rotX)) > LIMITEGIR || (abs(rotY)) > LIMITEGIR || (abs(rotZ)) > LIMITEGIR) {
    acidente = 1;
  }

}

/*
 * Read HMC
 */
void readHMC(){
  int16_t magX, magY, magZ;
  float heading, headingDegrees;
  Wire.beginTransmission(0b0001101);
  Wire.write(0x00);
  Wire.endTransmission();
  //Read the data.. 2 bytes for each axis
  Wire.beginTransmission(0b0001101);
  Wire.requestFrom(0b0001101, 6);
  magX = Wire.read(); //LSB x
  magX |= (int)Wire.read()<<8; //MSB x 
  magY = Wire.read(); //LSB y
  magY |= (int)Wire.read()<<8; //MSB y
  magZ = Wire.read(); //LSB z
  magZ |= (int)Wire.read()<<8; //MSB z
  Wire.endTransmission();

  heading = atan2(magY, magX);
  heading += declinationAngle;
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  headingDegrees = heading * 180/PI;

//  Serial.print("MagX="); Serial.print(magX);
//  Serial.print(" MagY="); Serial.print(magY);
//  Serial.print(" MagZ="); Serial.print(magZ);
//  Serial.println();
//  Serial.print("Head="); Serial.print(heading);
//  Serial.print(" Head_deg="); Serial.print(headingDegrees);
//  Serial.println();
  bearingDegrees(headingDegrees);
//  Serial.print("Bearing="); Serial.print(bearing);
//  Serial.println();
}

// Calculate bearing from heading.
// relative to a magnetometer mounted with 'Y' axis pointing in the direction to be measured
// and with 'X' pointing to the right from the perspective of the operator facing in the +Y direction
// bearing 0 = Y pointing North, 90 = Y pointing E, 180 = Y pointing S, 270 = Y pointing W
void bearingDegrees(float headingDeg) {
   bearing = headingDeg - 90;
   if (bearing < 0) bearing += 360;
}

/*=======================================================================================================================================
  ||                                                                                                                                   ||
  ||                               Detecção de acidente a partir da variação do ACC em um determinado intervalo                        ||
  ||                                                                                                                                   ||
  =====================================================================================================================================*/
void acidente_acelerometro() {
  float varAcX = 0, varAcY = 0, varAcZ = 0;
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
 * Print messages in LCD
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
  lcd.print(" B ");
  lcd.print(bearing);
//  Serial.println("Terminou LCD");
}
