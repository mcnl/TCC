#include <Stepper.h> 
 
const int stepsPerRevolution = 500; 
  
//Inicializa a biblioteca utilizando as portas de 8 a 11 para 
//ligacao ao motor 
Stepper myStepper_vertical(stepsPerRevolution, 26,33,25,32); 
Stepper myStepper_horizontal(stepsPerRevolution, 23,18,19,5); //13 12 14 27
int angleofSun   = 108;
int currentAngle = 0;
void setup() 
{ 
 //Determina a velocidade inicial do motor 
 Serial.begin(9600);
 myStepper_vertical.setSpeed(60);
 myStepper_horizontal.setSpeed(60);
} 
  
void loop() 
{ 
  
 //Gira o motor no sentido horario a 90 graus
 while(angleofSun>currentAngle)
 {
 Serial.print("Angulo vertical menor, reajuste   ");
 Serial.println(currentAngle);
 myStepper_vertical.step(-2048); 
 currentAngle += random(10);
 delay(500);
 }
  
 //Gira o motor no sentido anti-horario a 120 graus
 while(angleofSun<currentAngle)
 {
  Serial.print("Angulo vertical maior, reajuste   ");
 Serial.println(currentAngle);
 myStepper_vertical.step(2048);
 currentAngle -= random(10);
 delay(500);
 }


currentAngle = 0;

  while(angleofSun>currentAngle)
 {
 Serial.print("Angulo horizontal menor, reajuste   ");
 Serial.println(currentAngle);
 myStepper_horizontal.step(-2048); 
 currentAngle += random(10);
 delay(500);
 }
  
 //Gira o motor no sentido anti-horario a 120 graus
 while(angleofSun<currentAngle)
 {
  Serial.print("Angulo horizontal maior, reajuste   ");
 Serial.println(currentAngle);
 myStepper_horizontal.step(2048);
 currentAngle -= random(10);
 delay(500);
 }
 Serial.println("Angulo perfeito!!!!  ");
 Serial.println(currentAngle);

 delay(2000); 
}
