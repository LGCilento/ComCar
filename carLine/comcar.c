 /*DECLARAÇÃO DE VARIAVEIS*/
#define H_Bridge_In1 11
#define H_Bridge_In2 10
#define H_Bridge_In3 9
#define H_Bridge_In4 8

#define Line_Sensor_In1 7
#define Line_Sensor_In2 4
#define Line_Sensor_In3 3
#define Line_Sensor_In4 2

#define Motor_Right_PWM 6  
#define Motor_Left_PWM 5

int Voltage = 9 ;
bool Right, Right2, Left, Left2;
int veloc0,veloc1, veloc3 = 0;
 
void setup() {
  Serial.begin(9600);
  pinMode(H_Bridge_In1, OUTPUT);
  pinMode(H_Bridge_In2, OUTPUT);
  pinMode(H_Bridge_In3, OUTPUT);
  pinMode(H_Bridge_In4, OUTPUT);
  
  pinMode(Line_Sensor_In1, INPUT);
  pinMode(Line_Sensor_In2, INPUT);
  pinMode(Line_Sensor_In3, INPUT);
  pinMode(Line_Sensor_In4, INPUT);
  
  pinMode(Motor_Right_PWM, OUTPUT);
  pinMode(Motor_Left_PWM, OUTPUT);
  veloc3 = 1275/Voltage;
  veloc1 = veloc3/3;
  
}

void loop() {
   //Define o sentido de rotação dos motores
  digitalWrite(H_Bridge_In1, HIGH);
  digitalWrite(H_Bridge_In2, LOW);
  digitalWrite(H_Bridge_In3, HIGH);
  digitalWrite(H_Bridge_In4, LOW);
  
  //Leituras dos Sensores
  Right2 = digitalRead(Line_Sensor_In1);
  Right = digitalRead(Line_Sensor_In2);
  Left = digitalRead(Line_Sensor_In3);
  Left2 = digitalRead(Line_Sensor_In4);

  Serial.print(Right2);
  Serial.print("||");
  Serial.print(Right);
  Serial.print("||");
  Serial.print(Left);
  Serial.print("||");
  Serial.println(Left2);
  delay(200);
 
  //Rodando os motores dependendo das leituras
  if(Right ==false && Left == false){
    analogWrite(Motor_Right_PWM,veloc3);
    analogWrite(Motor_Left_PWM, veloc3);
  }
  if(Right == true){
      analogWrite(Motor_Right_PWM,veloc1);
  }
  if(Right2 == true){
      analogWrite(Motor_Right_PWM,veloc0);
  }
  }
  if(Left == true){
     analogWrite(Motor_Left_PWM, veloc1);
  }
  if(Left2 == true){
      analogWrite(Motor_Left_PWM, veloc0);
  }
  if(Right == true && Right2 == true && Left == true && Left2 == true){
  analogWrite(Motor_Right_PWM, veloc0);
  analogWrite(Motor_Left_PWM, veloc0);
  }
}
