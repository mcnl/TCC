
const int pinoTensao = 39;

int entradaAnalogicaTensao = 0;
double tensao;

void setup() {
  Serial.begin(9600);
  delay(1000);
}

void loop() {
  float somatorio = 0;
  float nivelCargaAtual;
  int nivelCarga;
  for(int i=0;i<1000;i++){
    entradaAnalogicaTensao = analogRead(pinoTensao);
    tensao = (3.3 * entradaAnalogicaTensao)/4095;
    tensao = tensao * (12.7)/2.7;
    if(tensao<3){
       nivelCargaAtual = 0;
    }
    else{
      nivelCargaAtual = 100*(tensao - 3)/12;
    }
    somatorio += nivelCargaAtual;
  }
  nivelCarga = somatorio/1000;
  Serial.println(nivelCarga);
}
