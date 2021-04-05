
const int pinoTensao = 27;

int entradaAnalogicaTensao = 0;
double tensao;

void setup() {
  Serial.begin(9600);
  delay(1000);
}

void loop() {
  // Reading potentiometer value
  entradaAnalogicaTensao = analogRead(pinoTensao);
  tensao = 0.000806*potValue+1.6;
  Serial.print(entradaAnalogicaTensao);
  Serial.print(" = ");
  Serial.print(tensao);
  
  Serial.print(" - ");
  if(tensao<3){
     Serial.println("0%");
  }
  else{
    Serial.print((tensao-3.0)*83.333);
    Serial.println("% ");
  }
  delay(500);
}
