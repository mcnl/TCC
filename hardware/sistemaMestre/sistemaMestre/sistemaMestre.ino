#include "SoftwareSerial.h" // Inclui a biblioteca SoftwareSerial
#define pinoRX 0
#define pinoTX 1
#define Wake   6
#include <LiquidCrystal.h>

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
 
//Define os pinos que serão utilizados para ligação ao display
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
 
void setup()
{
  blackBoardSlave.begin(9600);
  pinMode(Wake,OUTPUT);
  pinMode(10, INPUT); 
  pinMode(9, INPUT);
  pinMode(8, INPUT);
  pinMode(7, INPUT); 
  lcd.begin(16, 2);
}
 
void loop()
{
  int s = digitalRead(10);
  int e = digitalRead(9);
  int b = digitalRead(8);
  int l = digitalRead(7);
  
  if(s==1){
    lcd.clear();
    //Posiciona o cursor na coluna 3, linha 0;
    lcd.setCursor(0, 1);
    //Envia o texto entre aspas para o LCD
    lcd.print("ModoEconomia OFF");
    lcd.setCursor(0, 0);
    lcd.print("Comando Enviado!");
    delay(3000);
    lcd.clear();
    responsee = comandoSunnySpot('S');
    lcd.setCursor(0, 0);
    //Envia o texto entre aspas para o LCD
    lcd.print("Resposta:");
    lcd.setCursor(0, 1);
    lcd.print(responsee);
    delay(3000);
    lcd.clear();
    
  }
  else if(e==1){
    lcd.clear();
    //Posiciona o cursor na coluna 3, linha 0;
    lcd.setCursor(0, 1);
    //Envia o texto entre aspas para o LCD
    lcd.print("Modo Economia ON");
    lcd.setCursor(0, 0);
    lcd.print("Comando Enviado!");
    delay(3000);
    lcd.clear();
    responsee = comandoSunnySpot('E');
    lcd.setCursor(0, 0);
    //Envia o texto entre aspas para o LCD
    lcd.print("Resposta:");
    lcd.setCursor(0, 1);
    lcd.print(responsee);
    delay(3000);
    lcd.clear();
     
  }
  else if(b==1){
    lcd.clear();
    //Posiciona o cursor na coluna 3, linha 0;
    lcd.setCursor(0, 1);
    //Envia o texto entre aspas para o LCD
    lcd.print("Perguntar Nivel!");
    lcd.setCursor(0, 0);
    lcd.print("Comando Enviado!");
    delay(3000);
    lcd.clear();
    responsee = comandoSunnySpot('B');
    lcd.setCursor(0, 0);
    //Envia o texto entre aspas para o LCD
    lcd.print("Resposta:");
    lcd.setCursor(0, 1);
    lcd.print(responsee);
    delay(3000);
    lcd.clear();
    
  }
  else if(l==1){
    lcd.clear();
    //Posiciona o cursor na coluna 3, linha 0;
    lcd.setCursor(0, 1);
    //Envia o texto entre aspas para o LCD
    lcd.print("Perguntar Brilho");
    lcd.setCursor(0, 0);
    lcd.print("Comando Enviado!");
    delay(3000);
    lcd.clear();
    responsee = comandoSunnySpot('L');
    lcd.setCursor(0, 0);
    //Envia o texto entre aspas para o LCD
    lcd.print("Resposta:");
    lcd.setCursor(0, 1);
    lcd.print(responsee);
    delay(3000);
    lcd.clear();
  
  }
}
