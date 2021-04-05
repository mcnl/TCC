#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  120   

RTC_DATA_ATTR int bootCount = 0;

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

const int pinoTensao = 27;
int entradaAnalogicaTensao = 0;
double tensao;

int nivelCarga;

void setup() {
  Serial.begin(9600);
  delay(1000);
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));
  print_wakeup_reason();
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
  " Seconds");
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_2,1); //1 = High, 0 = Low
}

void loop() {
  entradaAnalogicaTensao = analogRead(pinoTensao);
  tensao = 0.000806 * entradaAnalogicaTensao + 1.6;
  
  if(tensao<3){
     nivelCarga = 0;
  }
  else{
    nivelCarga = (tensao-3.0)*83.333;
  }
  
  Serial.print((tensao-3.0)*83.333);
  Serial.println("% ");
  
  if(nivelCarga<20){
      Serial.println("Nivel de carga baixo, entrando em modo de economia");
      Serial.flush(); 
      esp_deep_sleep_start();
  }
  
  delay(500);

}
