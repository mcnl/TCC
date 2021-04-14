#include "SoftwareSerial.h" // Inclui a biblioteca SoftwareSerial

// Cria uma serial em software 
SoftwareSerial blackBoardMaster(15,4); // (RX, TX)

void setup() {

  blackBoardMaster.begin(9600);
  Serial.begin(9600);
}

void loop() {

  if (blackBoardMaster.available() > 0){
  
    char received = blackBoardMaster.read();

    Serial.print("Comando chegou: ");
    Serial.println(received);
    
    if (received == 'L'){//Retorna luminosidade
      blackBoardMaster.print(7.0);
    }
    else if (received == 'E'){//Retorna True(1), e coloca o sistema em modo de economia
      blackBoardMaster.print(1.0);
    }
    else if (received == 'B'){//retone nivel da bateria
      blackBoardMaster.print(-7.0);
    }
    else{//Retorna código de erro, no nosso caso '!'
      blackBoardMaster.print(0.0);
    }
  }
}
