int statusBotao=2;   // to store on or off value
void setup()
{
  Serial.begin(9600);
  pinMode(16,INPUT);
}
void loop()
{
  
  esperaPeloOn();

}

void esperaPeloOn(){
  statusBotao = digitalRead(16);
  delay(2000);
  Serial.println("Esperando para ser ligado!");
  while(statusBotao==0){
    statusBotao=digitalRead(16);
  }
  Serial.println("Foi ligado!");
}
