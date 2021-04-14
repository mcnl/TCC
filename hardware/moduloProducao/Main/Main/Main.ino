//Modulo de Producao - Código Principal
//Escrito e projetado por Matheus Casa Nova da Luz
#define uS_TO_S_FACTOR 1000000ULL  
#define TIME_TO_SLEEP  5   

#include <Wire.h>

#include <Stepper.h> 

#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#include <virtuabotixRTC.h> 

#include <WiFi.h>
#include <IOXhop_FirebaseESP32.h>

// Set these to run example.
#define FIREBASE_HOST "sunny-spots-757b9.firebaseio.com/"
#define FIREBASE_AUTH "7OcygkhYndryXxWsa8i2QrGoZxqgvEPEkpzxT9Q9"
#define WIFI_SSID "NET_2GDF6789"
#define WIFI_PASSWORD "B3DF6789"

#include <LiquidCrystal_I2C.h>
#define lcdColumns 16
#define lcdRows 2
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);  

//Variaveis do Modulo de Produção
//Sensor de Angulagem
#define MPU_addr 0x68
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
 
#define minVal 265
#define maxVal 402

 
//Motores de Passo
#define stepsPerRevolution  500
Stepper myStepper_vertical(stepsPerRevolution, 26,33,25,32); 
Stepper myStepper_horizontal(stepsPerRevolution, 23,18,19,5);
//GPS
#define RXPin 34
#define TXPin 35
#define GPSBaud 4800
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
//Data, GPS e Angulo
//Segundos, Minutos, Horas, Dia, Mes, Ano
RTC_DATA_ATTR uint16_t dataAtual[6]      = {0,0,11,19,3,2021};
RTC_DATA_ATTR float gpsAtual[]    = {-8.15762000,-34.91477200};
float ultimaElevacao = 90.0;
float ultimoAzimuth  = 0.0;
float angulosPlacaSolar[3] = {0,0,0};
float filterAngulosPlacaSolar[3] = {0,0,0};
RTC_DATA_ATTR int ultimaMovimentacao = 10;
uint8_t retries = 0;
//RTC
RTC_DATA_ATTR virtuabotixRTC myRTC(14, 12, 13);
//Comando
SoftwareSerial sistemaMestre(15,4);
//LDR
uint8_t brilho;
//Parte do modulo de Gerenciamento de Bateria
//Variaveis do Sleep
//Botao ON
uint8_t statusBotao=2; 

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : break;
    case ESP_SLEEP_WAKEUP_EXT1 : sistemaMestre.print(1); break;
    case ESP_SLEEP_WAKEUP_TIMER : mainGerenciamentodeBateria(); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : break;
    case ESP_SLEEP_WAKEUP_ULP : break;
    default : break;
  }
}
//Variaveis de medição da Bateria
#define pinoTensao 39
uint16_t entradaAnalogicaTensao = 0;
uint8_t tensao;
uint8_t nivelCarga; 

RTC_DATA_ATTR bool setupFeito = false;


void setup() {
  delay(10000);
  // Setup Inicial 
  setupLCD();
  setupGerenciamentoBateria();
  setupModuloProducao();
  setupComando();
  dataHoraAtual();
  setupFirebase();
  if(!setupFeito){
    //finalSetup();  
  }
}

void loop() {
  
  mainprocessamentodeComando();
  
  calcularBrilho();
  
  obterAnguloAtual();
  
  
  char resultadoGerenciamentodeBateria = mainGerenciamentodeBateria();
  if(resultadoGerenciamentodeBateria == '2'){
      nivelCarga = -1;
  }
  enviarInformacaoFirebase(gpsAtual[0], gpsAtual[1], angulosPlacaSolar[1], angulosPlacaSolar[0], nivelCarga, brilho);
  
  if(statusBotao != digitalRead(16)){
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Carga     Brilho");
    lcd.setCursor(0,1);
    lcd.print(nivelCarga);
    lcd.setCursor(13,1);
    lcd.print(brilho);
    delay(10000);
    lcd.noBacklight();
    statusBotao = digitalRead(16);
  }
  
  if((abs(ultimaMovimentacao - dataAtual[2]) == 1)){
    char result = mainmoduloProducao(); 
    if     (result == '1')printLCD(100,3000,"Erro mecanico!");
    else if(result == '4')printLCD(100,3000,"Não foi possivel posicionar a placa :/");
    else printLCD(100,3000,"Placa Posicionada");
  } 
  delay(1000);
}


void finalSetup(){
  printLCD(100,3000,"Ola usuario(a)! Eu me chamo Sunny Spots! :D");
  printLCD(100,2000,"Eu sou uma estacao de producao de energia solar");
  printLCD(100,2000,"Por favor nao conecte nada, ok?");
  printLCD(100,2000,"E nem coloque o ligar, para ligado ;)");
  printLCD(100,2000,"Por favor, posiciona o painel para o norte?");
  printLCD(100,2000,"É só virar ele até o LDR estar voltado para o norte");
  printLCD(100,2000,"Quando tiver pronto, liga e desliga o botao OFF");
  esperaPeloOn();
  esperaPeloOff();
  printLCD(100,2000,"Perfeito :D");
  printLCD(100,2000,"Me da um instante pra pegar nossa localizacao e que dia e hora eh hoje! ;)");
  while(retries<3 && !posicaoGpsAtual()){
    retries++;
    printLCD(100,3000,"Tentativa " + (retries));
    delay(1000);
  }
  if(retries==3){
    printLCD(100,3000,"Meu sensor GPS não conseguiu pegar os dados necessários para seguir a execução :(");
    printLCD(100,3000,"Mas eu posso tentar pegar as informações pelo firebase :D");

    printLCD(100,3000,"Verifique as informações lá!");
    printLCD(100,3000,"Se estiver tudo de acordo, aperte o botao ON!");

    esperaPeloOn();
    
    dataAtual[0] = pegarDatadoFirebase("segundo");
    dataAtual[1] = pegarDatadoFirebase("minuto");
    dataAtual[2] = pegarDatadoFirebase("hora");
    dataAtual[3] = pegarDatadoFirebase("dia");
    dataAtual[4] = pegarDatadoFirebase("mes");
    dataAtual[5] = pegarDatadoFirebase("ano");

    gpsAtual[0] = pegarLocaldoFirebase("latitude");
    gpsAtual[1] = pegarLocaldoFirebase("longitude");
    
    printLCD(100,3000,"Consegui o que a gente precisava no Firebase :D");
    printLCD(100,3000,"Latitude: ");
    printLCDDados(100,3000,gpsAtual[0]);
    printLCD(100,3000,"Longitude: ");
    printLCDDados(100,3000,gpsAtual[1]);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(dataAtual[2]);
    lcd.print(":");
    lcd.print(dataAtual[1]);
    lcd.setCursor(0,1);
    lcd.print(dataAtual[3]);
    lcd.print("/");
    lcd.print(dataAtual[4]);
    lcd.print("/");
    lcd.print(dataAtual[5]);
    setupGPSRTC();
    
  }
  else{
    printLCD(100,3000,"Consegui o que a gente precisava :D");
    printLCD(100,3000,"Latitude: ");
    printLCDDados(100,3000,gpsAtual[0]);
    printLCD(100,3000,"Longitude: ");
    printLCDDados(100,3000,gpsAtual[1]);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(dataAtual[2]);
    lcd.print(":");
    lcd.print(dataAtual[1]);
    lcd.setCursor(0,1);
    lcd.print(dataAtual[3]);
    lcd.print("/");
    lcd.print(dataAtual[4]);
    lcd.print("/");
    lcd.print(dataAtual[5]);
    delay(3000);
    printLCD(100,3000,"Atualizando Firebase!");
    
    setarDatadoFirebase("segundo", dataAtual[0]);
    setarDatadoFirebase("minuto",  dataAtual[1]);
    setarDatadoFirebase("hora",    dataAtual[2]);
    setarDatadoFirebase("dia",     dataAtual[3]);
    setarDatadoFirebase("mes",     dataAtual[4]);
    setarDatadoFirebase("ano",     dataAtual[5]);

    setarLocaldoFirebase("latitude", gpsAtual[0]);
    setarLocaldoFirebase("longitude", gpsAtual[1]);
    
    printLCD(100,3000,"Firebase Atualizado!");
    setupGPSRTC();
    printLCD(100,3000,"RTC Atualizado :D");
    printLCD(100,3000,"Agora por favor, ligue o interruptor ON");
  
    esperaPeloOn();
  }
  printLCD(100,3000,"Iniciando Producao :D");
  setupFeito = true;
  ultimaMovimentacao = dataAtual[2] - 1;
  delay(5000);  
  lcd.noBacklight();
}

//Funcoes divididas____________________________________________________________________________________________________
void modoEconomia(bool ligar){
  if(ligar){
    printLCD(100,3000,"Entrando em Modo Economia");
    esp_deep_sleep_start();
  }
}


//INICIO WIFI - FIrebase____________________________________________________________________________________________________

void setupFirebase(){
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  printLCD(100,3000,"Conectando com Wifi");
  while (WiFi.status() != WL_CONNECTED) {
    printLCD(50,1000,"....");
    delay(500);
  }
  printLCD(100,3000,"Conexao adquirida! :D");
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(WiFi.localIP());

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
}

bool enviarInformacaoFirebase(float lat, float lng, float horizontal, float vertical, int nivelBateria, int luminosidade){
    bool n = 0;
    Firebase.setFloat("/device-1/local/latitude", lat);
    if (Firebase.failed()) {
        return false;
    }
    Firebase.setFloat("/device-1/local/longitude", lng);
    if (Firebase.failed()) {
        return false;
    }
    Firebase.setInt("/device-1/leitura/carga", nivelBateria);
    if (Firebase.failed()) {
        return false;
    }
    Firebase.setInt("/device-1/leitura/luminosidade", luminosidade);
    if (Firebase.failed()) {
        return false;
    }
    Firebase.setInt("/device-1/leitura/anguloPainel/elevacao", vertical);
    if (Firebase.failed()) {
        return false;
    }
    Firebase.setInt("/device-1/leitura/anguloPainel/azimute", horizontal);
    if (Firebase.failed()) {
        return false;
    }
    String name = Firebase.pushInt("/device-1/logs/", n++);
    if (Firebase.failed()) {
        return false;
    }
    Firebase.setInt("/device-1/logs/"+name+"/carga", nivelBateria);
    if (Firebase.failed()) {
        return false;
    }
    Firebase.setInt("/device-1/logs/"+name+"/luminosidade", luminosidade);
    if (Firebase.failed()) {
        return false;
    }
    Firebase.setInt("/device-1/logs/"+name+"/anguloPainel/elevacao", vertical);
    if (Firebase.failed()) {
        return false;
    }
    Firebase.setInt("/device-1/logs/"+name+"/anguloPainel/azimute", horizontal);
    if (Firebase.failed()) {
        return false;
    }
    return true;
}

uint16_t pegarDatadoFirebase(String unidadeTempo){
  uint16_t resposta = Firebase.getInt("/device-1/data/"+unidadeTempo);
  if (Firebase.failed()) {
        return -1;
  }
  return resposta;
}
float pegarLocaldoFirebase(String unidadeLocal){
  float resposta = Firebase.getFloat("/device-1/local/"+unidadeLocal);
  if (Firebase.failed()) {
        return -1259;
  }
  return resposta;
}
bool setarDatadoFirebase(String unidadeTempo, int tempo){
  Firebase.setInt("/device-1/data/"+unidadeTempo,tempo);
  if (Firebase.failed()) {
        return false;
  }
  return true;
}
bool setarLocaldoFirebase(String unidadeLocal, float coordenada){
  Firebase.setFloat("/device-1/local/"+unidadeLocal,coordenada);
  if (Firebase.failed()) {
        return false;
  }
  return true;
}




//FIM WIFI-FIREBASE____________________________________________________________________________________________________




//LCD INICIO____________________________________________________________________________________________________

void setupLCD(){
  lcd.init();                     
  lcd.backlight();
}

void printLCD(uint16_t velocidadeLetras, uint16_t velocidadeApagarTela, std::string mensagem){
  lcd.clear(); 
  lcd.setCursor(0,0);
  for(uint8_t i = 0; i< mensagem.size();i++){
    if(i%32==0 && i>0){
      delay(velocidadeApagarTela/2);
      lcd.clear();
      lcd.setCursor(0, 0);
      delay(velocidadeApagarTela);
    }
    else if(i%16==0 && i>0){
      lcd.setCursor(0, 1);
    }
    lcd.print(mensagem[i]);
    delay(velocidadeLetras);
  }
  delay(velocidadeApagarTela/2);
}

void printLCDDados(uint16_t velocidadeLetras, uint16_t velocidadeApagarTela, float mensagem){ 
  lcd.setCursor(0,1);
  lcd.print(mensagem);
  delay(velocidadeApagarTela/2);
}


//LCD FIM____________________________________________________________________________________________________

//BUTTON INICIO____________________________________________________________________________________________________
void setupButton(){
  pinMode(16,INPUT);
}

void esperaPeloOn(){
  statusBotao = digitalRead(16);
  delay(2000);
  while(statusBotao==0){
    statusBotao=digitalRead(16);
  }
}

void esperaPeloOff(){
  statusBotao = digitalRead(16);
  delay(2000);
  while(statusBotao==1){
    statusBotao=digitalRead(16);
  }
}


//BUTTON FIM____________________________________________________________________________________________________



//INICIO Comando____________________________________________________________________________________________________

  void setupComando(){
      sistemaMestre.begin(9600);
  }

  void mainprocessamentodeComando(){//nivelCarga
    
    if (sistemaMestre.available() > 0){
      char received = sistemaMestre.read();
  
       printLCD(100,3000,"Comando recebido do Mestre!:D");
      
      if (received == 'L'){//Retorna luminosidade
        sistemaMestre.print(brilho);//TODO Kuminosidade
      }
      else if (received == 'E'){//Retorna True(1), e coloca o sistema em modo de economia
        sistemaMestre.print(1.0);
        modoEconomia(true);
      }
      else if (received == 'B'){//retone nivel da bateria
        sistemaMestre.print(nivelCarga);
      }
      else{//Retorna código de erro, no nosso caso '!'
        sistemaMestre.print(-225.0);
      }
    }
  
  }

//FIM Comando____________________________________________________________________________________________________


//INICIO LDR - Sensoriamento____________________________________________________________________________________________________

void calcularBrilho(){
  uint16_t somatorio = 0;
  uint16_t valorAnalogico;
  uint8_t brilhoAtual;
  for(uint16_t i=0;i<1000;i++){
      valorAnalogico = analogRead(36);
      brilhoAtual = map(valorAnalogico, 0, 4095, 0, 100);
      somatorio += brilhoAtual;
  }  
  brilho = somatorio/1000;
}

//FIM LDR - Sensoriamento____________________________________________________________________________________________________

//INICIO Gerenciamento de Bateria____________________________________________________________________________________________________

void setupGerenciamentoBateria(){
  print_wakeup_reason();
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_2,1); 
}

char mainGerenciamentodeBateria(){
  if(entradaAnalogicaTensao<0 || entradaAnalogicaTensao>4095)return '2';
  calculaNiveldeCarga();
  if(nivelCarga<20){
    modoEconomia(true);
    return '1';
  }    
  return '0';
}
void calculaNiveldeCarga(){
  uint16_t somatorio = 0;
  uint8_t nivelCargaAtual;
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
  
}

//FIM Gerenciamento de Bateria____________________________________________________________________________________________________


// Inicio MODULO DE PRODUCAO____________________________________________________________________________________________________
void setupGPSRTC(){
  myRTC.setDS1302Time(dataAtual[0], dataAtual[1], dataAtual[2], 2,dataAtual[3], dataAtual[4], dataAtual[5]);
}


void setupModuloProducao(){
  
  //Sensor de Angulagem
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  //Motores de Passo
  myStepper_vertical.setSpeed(30);
  myStepper_horizontal.setSpeed(1);
  //GPS
  ss.begin(GPSBaud);
}

char mainmoduloProducao(){
  if(!obterAnguloAtual()){
    return '1';
  }
  else{
    posicaoSolarAtual();
    bool error = posicionarPlaca();
    if(!error) return '4';
    return '0';
  }
}


bool dataHoraAtual(){
  myRTC.updateTime();
  dataAtual[0] = myRTC.seconds;
  dataAtual[1] = myRTC.minutes;
  dataAtual[2] = myRTC.hours;
  dataAtual[3] = myRTC.dayofmonth;
  dataAtual[4] = myRTC.month;
  dataAtual[5] = myRTC.year;

  if(dataAtual[5]!=2021) return false;
  
  return true;
}

bool obterAnguloAtual(){
  
  for(uint16_t i = 0; i<1000 ; i++){
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);
    
    AcX=Wire.read()<<8|Wire.read();
    AcY=Wire.read()<<8|Wire.read();
    AcZ=Wire.read()<<8|Wire.read();
  
    float out[3] = {AcX, AcY, AcZ};
      
    uint16_t xAng = map(out[0],minVal,maxVal,0,360);
    uint16_t yAng = map(out[1],minVal,maxVal,0,360);
    uint16_t zAng = map(out[2],minVal,maxVal,0,360);
    
    angulosPlacaSolar[0] = RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
    angulosPlacaSolar[1] = RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
    angulosPlacaSolar[2] = RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
    
    filterAngulosPlacaSolar[0] += angulosPlacaSolar[0];
    filterAngulosPlacaSolar[1] += angulosPlacaSolar[1];
    filterAngulosPlacaSolar[2] += angulosPlacaSolar[2];

  }
  
  angulosPlacaSolar[0] = (90 * abs(cos(filterAngulosPlacaSolar[0]/1000*PI/180)));
  angulosPlacaSolar[1] = filterAngulosPlacaSolar[1]/1000;
  angulosPlacaSolar[2] = filterAngulosPlacaSolar[2]/1000;
  
  //Equacao para adequar angulo a o espaco de angulo de elevacao
  angulosPlacaSolar[1] = ((((angulosPlacaSolar[0]/90)*angulosPlacaSolar[2] + (1-angulosPlacaSolar[0]/90)*angulosPlacaSolar[1]))*2.25);

  filterAngulosPlacaSolar[0] = 0;
  filterAngulosPlacaSolar[1] = 0;
  filterAngulosPlacaSolar[2] = 0;

  if(angulosPlacaSolar[2]==225) return false;
  return true;
}

bool posicaoGpsAtual(){
  while (ss.available() > 0){
    if (gps.encode(ss.read())){
      if (gps.location.isValid())
      {
        gpsAtual[0] = gps.location.lat();
        gpsAtual[1] = gps.location.lng();
      }
      else
      {
        return false;
      }
      if (gps.date.isValid())
      {
        dataAtual[3] = gps.date.month();
        dataAtual[4] = gps.date.day();
        dataAtual[5] = gps.date.year();
      }
      else
      {
        return false;
      }
      if (gps.time.isValid())
      {
        dataAtual[2] = gps.time.hour();
        dataAtual[1] = gps.time.minute();
        dataAtual[0] = gps.time.second();
        return true;
      }
      else
      {
        return false;
      }
    }
  }
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    return false;
  }
}
void moverMotorAzimuth(unsigned short passos){
  myStepper_horizontal.step(passos);
}
void moverMotorElevacao(unsigned short passos){
  myStepper_vertical.step(passos);
}
bool posicionarPlaca(){
  
  bool perfectPositioned = false;
  printLCD(100,3000,"Posicionando Placa");
  
  while(!perfectPositioned){
    bool error = obterAnguloAtual();
    
    if(!error) return false;
    
    float diferencaElevacao = angulosPlacaSolar[0] - ultimaElevacao;
    float diferencaAzimute  = angulosPlacaSolar[1] - ultimoAzimuth;
    
    if(diferencaElevacao > 5){
      moverMotorElevacao(-3000);
    }
    else if(diferencaAzimute > 5){
      moverMotorAzimuth(-10000);
    }
    else if(diferencaElevacao < -5){
      moverMotorElevacao(+3000);
    }
    else if(diferencaAzimute < -5){
     moverMotorAzimuth(+10000);
    }
    else{
      perfectPositioned = true;      
    }
    delay(500);
  }
  
  ultimaMovimentacao = dataAtual[2];
  return true;
}

//posicaoSolarAtual ----------------------------
#define rad   PI / 180.0
#define e    rad * 23.4397

float roundRadians(double beta){
	double trash;
	if(beta>2*PI){
		beta = beta/2*PI;
		beta = modf(beta, &trash);
	}
	return beta;
}
float dateToMiliseconds(int sec,int minute,int hour,int day, int month, int year){
	struct tm date = {
		.tm_sec=sec,
		.tm_min=minute,
		.tm_hour=hour,
		.tm_mday=day,
		.tm_mon=month,
		.tm_year=year - 1900
	};
	return ((float)mktime(&date))*1000.0;
}
float ToJulian(float date){
	return (date - dateToMiliseconds(1,0,0,1,1,1970)) / (1000.0 * 60.0 * 60.0 * 24.0) - 0.5 + 2440588.0;
}
float ToDays(float date){
	return ToJulian(date) - 2451545.0;
}
float rightAscencion(float d){
	float m = rad * (357.5291 + 0.98560028 * d);
	float c = rad * (1.9148 * sin(m) + 0.02 * sin(2 * m) + 0.0003 * sin(3 * m));
	float p = rad * 102.9372; 
	float l = m + c + p + PI;
	float ra = atan2((sin(l) * cos(e) - tan(0) * sin(e)),cos(l));
	
	return ra;
}
float declination(float d){
	float m = rad * (357.5291 + 0.98560028 * d);
	float c = rad * (1.9148 * sin(m) + 0.02 * sin(2 * m) + 0.0003 * sin(3 * m));
	float p = rad * 102.9372; 
	float l = m + c + p + PI;
	float dec = asin(sin(0) * cos(e) + cos(0) * sin(e) * sin(l));
	return dec;
}
float sideralTime(float d, float lw){
	return rad * (280.16 + 360.9856235 * d) - lw;
}
float azimuth(float H, float phi, float dec){
	return PI + atan2(sin(H), (cos(H) * sin(phi) - tan(dec) * cos(phi)));
}
float altitude(float H, float phi, float dec){
	return PI + asin(roundRadians(sin(phi) * sin(dec) + cos(phi) * cos(dec) * cos(H)));
}
float* getposition(float date, float lat, float lng){
	float lw = rad * (-lng);
	float phi = rad * lat;
	float d = ToDays(date);
	float RA = rightAscencion(d);
	float dec = declination(d);
	float H = sideralTime(d,lw) - RA;

	float az = azimuth(H,phi,dec);
	float alt = altitude(H,phi,dec);

	static float result[2];
	result[0] = 360 - (az)*180.0/PI;
	result[1] = (alt)*180.0/PI - 180;
	return result;
} 
void posicaoSolarAtual(){
  float *ret;
	float now;
	
  now = (float)dateToMiliseconds(dataAtual[0],dataAtual[1],(24 - dataAtual[2])+3,dataAtual[3],dataAtual[4],dataAtual[5]); 
	ret = getposition(now,gpsAtual[0],gpsAtual[1]);

  ultimaElevacao = *(ret + 1);
  ultimoAzimuth  = *(ret + 0);
}
//posicaoSolarAtualFim -------------------------

//FIM MODULO DE PRODUCAO____________________________________________________________________________________________________
