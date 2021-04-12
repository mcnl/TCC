int d=2;   // to store on or off value
void setup()
{
  Serial.begin(9600);
  pinMode(16,INPUT);
}
void loop()
{
  d=digitalRead(16);
  if(d==0){
    Serial.println("Não foi ligado!");
  }
  else{
    Serial.println("Ta ligado!");
  }
}
