#include <Stepper.h> 
 
const int stepsPerRevolution = 500; 
  
//Inicializa a biblioteca utilizando as portas de 8 a 11 para 
//ligacao ao motor 
Stepper myStepper(stepsPerRevolution, 26,33,25,32); 
int angleofSun   = 108;
int currentAngle = 0;
void setup() 
{ 
 //Determina a velocidade inicial do motor 
 Serial.begin(9600);
 myStepper.setSpeed(60);
} 
  
void loop() 
{ 
  
 //Gira o motor no sentido horario a 90 graus
 while(angleofSun>currentAngle)
 {
 Serial.print("Angulo menor, reajuste   ");
 Serial.println(currentAngle);
 myStepper.step(-2048); 
 currentAngle += random(5);
 delay(500);
 }
  
 //Gira o motor no sentido anti-horario a 120 graus
 while(angleofSun<currentAngle)
 {
  Serial.print("Angulo maior, reajuste   ");
 Serial.println(currentAngle);
 myStepper.step(2048);
 currentAngle -= random(5);
 delay(500);
 }
 Serial.println("Angulo perfeito!!!!  ");
 Serial.println(currentAngle);

 delay(2000); 
}
