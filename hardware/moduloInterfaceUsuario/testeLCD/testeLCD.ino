/*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com  
*********/

#include <LiquidCrystal_I2C.h>

// set the LCD number of columns and rows
int lcdColumns = 16;
int lcdRows = 2;
std::string ola = "Ola usuario(a)!";
std::string apresentacao = "Ola usuario(a)! Eu me chamo Sunny Spots :)";
// set LCD address, number of columns and rows
// if you don't know your display address, run an I2C scanner sketch
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);  

void setup(){
  // initialize LCD
  lcd.init();
  // turn on LCD backlight                      
  lcd.backlight();
}

void loop(){
  printLCD(50,2000,apresentacao);
}

void printLCD(int velocidadeLetras, int velocidadeApagarTela, std::string mensagem){
  lcd.clear(); 
  lcd.setCursor(0,0);
  for(int i = 0; i< mensagem.size();i++){
    if(i%32==0 && i>0){
      delay(velocidadeApagarTela);
      lcd.clear();
      lcd.setCursor(0, 0);
      delay(velocidadeApagarTela);
    }
    else if(i%16==0 && i>0){
      lcd.setCursor(0, 1);
    }
    lcd.print(apresentacao[i]);
    delay(velocidadeLetras);
  }
  delay(velocidadeApagarTela);
}
