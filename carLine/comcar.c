 /*DECLARAÇÃO DE VARIAVEIS*/
#define H_Bridge_In1 11
#define H_Bridge_In2 10
#define H_Bridge_In3 9
#define H_Bridge_In4 8

#define Line_Sensor_In1 A0
#define Line_Sensor_In2 A1
#define Line_Sensor_In3 A2
#define Line_Sensor_In4 A3


#define Motor_Right_PWM 6  
#define Motor_Left_PWM 5

#define My_Accident 12
#define Other_Accident 13

#define FOLLOW_LINE 0
#define CHANGE_LINE 1
#define FULL_STOP 2

int Voltage = 9 ;
float Percentage = 0.55 ;
bool Right, Left, Right2, Left2;
int veloc0, veloc1, MaxSpeed= 0;
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
  pinMode(Line_Sensor_In4, INPUT);
  
  
  pinMode(Motor_Right_PWM, OUTPUT);
  pinMode(Motor_Left_PWM, OUTPUT);


  pinMode(My_Accident, INPUT);
  pinMode(Other_Accident, INPUT);

   //Define o sentido de rotação dos motores
  digitalWrite(H_Bridge_In1, HIGH);
  digitalWrite(H_Bridge_In2, LOW);
  digitalWrite(H_Bridge_In3, LOW);
  digitalWrite(H_Bridge_In4, HIGH);

  MaxSpeed = 1275/Voltage;
  veloc1 = Percentage * MaxSpeed; 
}

void change_Line(){
  
  bool exited_line = false;
  bool found_new_line= false;
  bool ajusted = false;

  // --- Exit line
  if(i_am_on_the_right){
    //Turn to Left
    analogWrite(Motor_Left_PWM, veloc0);
    while( digitalRead(Line_Sensor_In4)== false); //wait until edge sensor takes the line
    while( digitalRead(Line_Sensor_In4)== true); //wait until edge sensor exit the line
    analogWrite(Motor_Left_PWM, veloc1); //go foward to get car out of the line
  } else{ //I am on the Left
    //Turn to the Right
    analogWrite(Motor_Right_PWM, veloc0);
    while( digitalRead(Line_Sensor_In1)== false); //wait until edge sensor takes the line
    while( digitalRead(Line_Sensor_In1)== true); //wait until edge sensor exit the line
    analogWrite(Motor_Right_PWM, veloc1); //go foward to get car out of the line
  }  

  //----Search  New Line-----------------
  
  while( !found_new_line ){
    Right = digitalRead(Line_Sensor_In2);
    Left = digitalRead(Line_Sensor_In3);
    Right2 = digitalRead(Line_Sensor_In1);
    Left2 = digitalRead(Line_Sensor_In4);

    if(Right2 == true || Left2 == true || Right == true || Left == true ){
        found_new_line = true;
    }
  }

  //---Ajust itself in New Line----------
    Right = digitalRead(Line_Sensor_In2);
    Left = digitalRead(Line_Sensor_In3);
    Right2 = digitalRead(Line_Sensor_In1);
    Left2 = digitalRead(Line_Sensor_In4);

  //Rodando os motores dependendo das leituras
  if( Right ==false && Left == false && Right2 ==false && Left2 == false){
    ajusted = true;
  } else if( ( Right2 == true && Left2 == true) || (Right == true && Left == true  ) ){ // perpendicular to th line
    if( i_am_on_the_right){ //I AM GONNA AJUST ON THE LEFT LINE
      analogWrite(Motor_Right_PWM, veloc0); //turn right
      while(digitalRead(Line_Sensor_In3) == true); //wait until it exits the line
      while(digitalRead(Line_Sensor_In3) == false); //wait until it re-enters the line
     analogWrite(Motor_Left_PWM, veloc0); //make a little curve to the left
     analogWrite(Motor_Right_PWM, veloc0); 
     while(digitalRead(Line_Sensor_In3) == true); // wait until enters correctly on the line
     
    } else{ //I AM GONNA AJUST ON THE RIGHT LINE
      analogWrite(Motor_Left_PWM, veloc0); //turn left
      while(digitalRead(Line_Sensor_In2) == true); //wait until it exits the line
      while(digitalRead(Line_Sensor_In2) == false); //wait until it re-enters the line
     analogWrite(Motor_Right_PWM, veloc0); //make a little curve to the left
     analogWrite(Motor_Left_PWM, veloc0); 
     while(digitalRead(Line_Sensor_In2) == true); // wait until enters correctly on the line
    }
    analogWrite(Motor_Left_PWM, veloc0);
    analogWrite(Motor_Right_PWM, veloc0);
    ajusted = true;
    
  } else if(Right == true || Right2 == true){
    analogWrite(Motor_Left_PWM, veloc1);
     analogWrite(Motor_Right_PWM, veloc1); // go foward
     while( digitalRead(Line_Sensor_In2) ==  true) ; //wait inner sensor enter line
     while( digitalRead(Line_Sensor_In2) ==  false); // wait inner sensor exit line
    analogWrite(Motor_Left_PWM, veloc0); // turn left
    while( digitalRead(Line_Sensor_In3) ==  true); //until corrects itself in the line
    analogWrite(Motor_Left_PWM, veloc0);
    analogWrite(Motor_Right_PWM, veloc0);
    ajusted = true;
  }

  else if(Left == true || Left2 == true){
     analogWrite(Motor_Left_PWM, veloc1);
     analogWrite(Motor_Right_PWM, veloc1); // go foward
     while( digitalRead(Line_Sensor_In3) ==  true) ; //wait inner sensor enter line
     while( digitalRead(Line_Sensor_In3) ==  false); // wait inner sensor exit line
    analogWrite(Motor_Right_PWM, veloc0); // turn right
    while( digitalRead(Line_Sensor_In2) ==  true); //until corrects itself in the line
    analogWrite(Motor_Left_PWM, veloc0);
    analogWrite(Motor_Right_PWM, veloc0);
    ajusted = true;
  }  
}


void follow_Line(){
   //Leituras dos Sensores
  Right = digitalRead(Line_Sensor_In2);
  Left = digitalRead(Line_Sensor_In3);
  Right2 = digitalRead(Line_Sensor_In1);
  Left2 = digitalRead(Line_Sensor_In4);

  //Rodando os motores dependendo das leituras
  if(curving  && Right ==false && Left == false && Right2 ==false && Left2 == false){
    analogWrite(Motor_Right_PWM,veloc1);
    analogWrite(Motor_Left_PWM, veloc1);
    curving = false;
   
  } else if(Right == true){
    analogWrite(Motor_Right_PWM, veloc0);
    curving = true;
    if(Right2 == true) delay(1000);
    else delay(20);
    
  } else if(Right2 == true){
    analogWrite(Motor_Right_PWM, veloc0);
    curving = true;
    delay(40);
  }
  
  else if(Left == true){
    analogWrite(Motor_Left_PWM, veloc0);
    curving = true;
    if(Left2 == true) delay(1000);
    else delay(40);
  }

  else if(Left2 == true){
    analogWrite(Motor_Left_PWM, veloc0);
    curving = true;
    delay(20);
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
      veloc1=0;
      break;
  }
  //counter++;
}
