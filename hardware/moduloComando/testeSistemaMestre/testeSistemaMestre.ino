#include "SoftwareSerial.h" // Inclui a biblioteca SoftwareSerial
#define pinoRX 9
#define pinoTX 8
#define Wake   10
// Cria uma serial em software 
SoftwareSerial blackBoardSlave(pinoRX, pinoTX); // (RX, TX)

char comandos[4] = {'L','B','E','S'};

float responsee;

float comandoSunnySpot(char comando){
  float resultadoComando; 
  if(comando == 'S'){
    
    digitalWrite(Wake,HIGH);
    delay(1000);
    digitalWrite(Wake,LOW);

    return 1;
     
  } 
  
  blackBoardSlave.print(comando);
  
  while(blackBoardSlave.available()<=0){}
 
  if(blackBoardSlave.available()>0){

    resultadoComando = blackBoardSlave.parseFloat();
    
    return resultadoComando;
  
  }
   
}


void setup() {
  // inicia a serial em software com uma taxa de 9600 bit/s
  blackBoardSlave.begin(9600);
  pinMode(Wake,OUTPUT);
  Serial.begin(9600);
}

void loop() {

   for(int i=0; i<4; i++){
      Serial.print("Responsta para comando ");
      Serial.print(comandos[i]);
      Serial.print(" é ");
      responsee = comandoSunnySpot(comandos[i]);
      Serial.println(responsee);
      delay(5000);
   } 
}
