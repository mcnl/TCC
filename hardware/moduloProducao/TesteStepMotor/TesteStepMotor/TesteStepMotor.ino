#include <Stepper.h> 
 
const int stepsPerRevolution = 500; 
  
//Inicializa a biblioteca utilizando as portas de 8 a 11 para 
//ligacao ao motor 
Stepper myStepper_vertical(stepsPerRevolution, 26,33,25,32); 
Stepper myStepper_horizontal(stepsPerRevolution, 23,18,19,5);
int angleofSun   = 30;
int currentAngle = 0;
int contator = 0;
void setup() 
{ 
 //Determina a velocidade inicial do motor 
 Serial.begin(9600);
 myStepper_vertical.setSpeed(30);
 myStepper_horizontal.setSpeed(5);
} 
  
void loop() 
{ 
contator +=1;
myStepper_horizontal.step(2048);
Serial.println(contator);
Serial.println("Continuar?");
Serial.read();
}