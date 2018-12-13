 /*DECLARAÇÃO DE VARIAVEIS*/
#define H_Bridge_In1 11
#define H_Bridge_In2 10
#define H_Bridge_In3 9
#define H_Bridge_In4 8

#define Line_Sensor_In1 A0
#define Line_Sensor_In2 A1
#define Line_Sensor_In3 A2

#define led_L 2  
#define led_R 3


#define Motor_Right_PWM 6  
#define Motor_Left_PWM 5

#define My_Accident 12
#define Other_Accident 13

#define FOLLOW_LINE 0
#define CHANGE_LINE 1
#define FULL_STOP 2

float Voltage = 8.2 ;
float PercentageR = 0.7;
float PercentageL = 0.9;
bool Right, Center, Left;
int veloc0, velocR,velocL, MaxSpeed= 0;
bool i_am_on_the_right = true;
int Status = FOLLOW_LINE;
int counter =0 ;
bool curving = true;
 
void setup() {
  Serial.begin(9600);
  pinMode(H_Bridge_In1, OUTPUT);
  pinMode(H_Bridge_In2, OUTPUT);
  pinMode(H_Bridge_In3, OUTPUT);
  pinMode(H_Bridge_In4, OUTPUT);

  pinMode(Line_Sensor_In1, INPUT);
  pinMode(Line_Sensor_In2, INPUT);
  pinMode(Line_Sensor_In3, INPUT);
  
  
  pinMode(Motor_Right_PWM, OUTPUT);
  pinMode(Motor_Left_PWM, OUTPUT);

  pinMode(led_R, OUTPUT);
  pinMode(led_L, OUTPUT);

  pinMode(My_Accident, INPUT);
  pinMode(Other_Accident, INPUT);

   //Define o sentido de rotação dos motores
  digitalWrite(H_Bridge_In1, HIGH);
  digitalWrite(H_Bridge_In2, LOW);
  digitalWrite(H_Bridge_In3, HIGH);
  digitalWrite(H_Bridge_In4, LOW);

  MaxSpeed = 1275/Voltage;
  velocR = PercentageR * MaxSpeed;
  velocL = PercentageL * MaxSpeed; 
}

void change_Line(){
}


void follow_Line(){
   //Leituras dos Sensores
  Right = digitalRead(Line_Sensor_In1);
  Center = digitalRead(Line_Sensor_In2);
  Left = digitalRead(Line_Sensor_In3);

  //Rodando os motores dependendo das leituras
  if(curving  && Center == true){
    analogWrite(Motor_Right_PWM,velocR);
    digitalWrite(led_R,HIGH);
    analogWrite(Motor_Left_PWM, velocL);
    digitalWrite(led_L,HIGH);
    curving = false;
  
  }
  else if(Right == true){
    analogWrite(Motor_Right_PWM, veloc0);digitalWrite(led_R,LOW);
    analogWrite(Motor_Left_PWM, velocL);digitalWrite(led_L,HIGH);
    curving = true;
    //while(digitalRead(Line_Sensor_In2) == false);
    delay(50);
  }
  
  else if(Left == true){
    analogWrite(Motor_Left_PWM, veloc0); digitalWrite(led_L,LOW);
    analogWrite(Motor_Right_PWM, velocR);digitalWrite(led_R,HIGH);
    curving = true;
    //while(digitalRead(Line_Sensor_In2) == false);
    delay(50);
  }
}

void loop() {
  switch(Status){
    case FOLLOW_LINE:
      follow_Line();
      //if ( digitalRead(My_Accident) == true ){
      //  Status = FULL_STOP;
      //} else 
     // if ( digitalRead(Other_Accident) == true ){
       // Status = CHANGE_LINE;
    //  }

      if(counter ==  40){
        Status = CHANGE_LINE;
      }
      break;
      
    case CHANGE_LINE:
    //Serial.println("ENTERED  CHANGE_LINE");
      change_Line();
      i_am_on_the_right = !i_am_on_the_right;
      Status = FOLLOW_LINE;
      break;
      
    case FULL_STOP:
    //Serial.println("ENTERED  FFULL_STOP");
      velocR= velocL=0;
      break;
  }
  //counter++;
}
