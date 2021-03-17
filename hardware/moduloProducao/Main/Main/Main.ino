//Modulo de Producao - Código Principal
//Escrito e projetado por Matheus Casa Nova da Luz
#include <math.h>
#include <stdlib.h>
#include <time.h>

#include <MPU6050_tockn.h>
#include <Wire.h>

#include <Stepper.h> 

#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#include <virtuabotixRTC.h> 

//Variaveis do Modulo de Produção
MPU6050 sensordeAngulagem(Wire); //Sensor de Angulagem
//Motores de Passo
const int stepsPerRevolution = 500; 
Stepper myStepper_vertical(stepsPerRevolution, 26,33,25,32); 
Stepper myStepper_horizontal(stepsPerRevolution, 23,18,19,5);
//GPS
static const int RXPin = 34, TXPin = 35;
static const uint32_t GPSBaud = 4800;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
//Data, GPS e Angulo
//Segundos, Minutos, Horas, Dia, Mes, Ano
int dataAtual[6]      = {0,0,12,19,3,2021};
double gpsAtual[]    = {-8.15762000,-34.91477200};
double ultimaElevacao = 90.0;
double ultimoAzimuth  = 0.0;
float angulosPlacaSolar[3] = {0,0,0};
//RTC
virtuabotixRTC myRTC(14, 12, 13);

void setup() {
  // Setup Inicial 
  Serial.begin(9600);
  setupModuloProducao();
}

void loop() {
  // put your main code here, to run repeatedly:
  char result = mainmoduloProducao();
  delay(5000);
}


void setupModuloProducao(){
  
  //Sensor de Angulagem
  Wire.begin();
  sensordeAngulagem.begin();
  sensordeAngulagem.calcGyroOffsets(true);
  //Motores de Passo
  myStepper_vertical.setSpeed(60);
  myStepper_horizontal.setSpeed(60);
  //GPS
  ss.begin(GPSBaud);
  //RTC (segundos, minutos, hora, dia da semana, dia do mes, mes, ano)
  myRTC.setDS1302Time(dataAtual[0], dataAtual[1], dataAtual[2], 2,dataAtual[3], dataAtual[4], dataAtual[5]);
}

char mainmoduloProducao(){
  if(!obterAnguloAtual()){
    return '1';
  }
  else if(!posicaoGpsAtual()){
    posicionarPlaca();
    return '2';
  }
  else if(!dataHoraAtual()){
    posicionarPlaca();
    return '3';
  }
  else{
    posicaoSolarAtual();
    printaAngulo();
    posicionarPlaca();
    return '0';
  }
}


bool dataHoraAtual(){

  myRTC.updateTime(); 
  printaTempo();//DEBUG - Remove after complete
  dataAtual[0] = myRTC.seconds;
  dataAtual[1] = myRTC.minutes;
  dataAtual[2] = myRTC.hours;
  dataAtual[3] = myRTC.dayofmonth;
  dataAtual[4] = myRTC.month;
  dataAtual[5] = myRTC.year;
  
  return true;
}
void printaTempo(){
  Serial.println("Pegando Data");
  Serial.print("Data : ");
  Serial.print(", ");
  Serial.print(myRTC.dayofmonth);
  Serial.print("/");
  Serial.print(myRTC.month);
  Serial.print("/");
  Serial.print(myRTC.year);
  Serial.print("  ");
  Serial.print("Hora : ");
  
  if (myRTC.hours < 10)
  {
    Serial.print("0");
  }

  Serial.print(myRTC.hours);
  Serial.print(":");
  if (myRTC.minutes < 10)
  {
    Serial.print("0");
  }

  Serial.print(myRTC.minutes);
  Serial.print(":");
  if (myRTC.seconds < 10)
  {
    Serial.print("0");
  }

  Serial.println(myRTC.seconds);

}
bool obterAnguloAtual(){
  sensordeAngulagem.update();
  angulosPlacaSolar[0] = sensordeAngulagem.getAngleX();
  angulosPlacaSolar[1] = sensordeAngulagem.getAngleY();
  angulosPlacaSolar[2] = sensordeAngulagem.getAngleZ();
  return true;
}
void printaAngulo(){
  Serial.println("Pegando Angulo Atual");
  Serial.print("Angulo Eixo X: ");
  Serial.print(sensordeAngulagem.getAngleX());
  Serial.print(" Angulo Eixo Y: ");
  Serial.println(sensordeAngulagem.getAngleY());
}
bool posicaoGpsAtual(){
  return true;//just for DEBUG - Delete this line after
  while (ss.available() > 0){
    if (gps.encode(ss.read())){
      if (gps.location.isValid())
      {
        gpsAtual[0] = gps.location.lat();
        gpsAtual[1] = gps.location.lng();
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
void moverMotorAzimuth(int passos){
  myStepper_horizontal.step(passos);
}
void moverMotorElevacao(int passos){
  myStepper_vertical.step(passos);
}
void posicionarPlaca(){
  bool perfectPositioned = false;
  Serial.println("Posicionando a placa...");
  while(!perfectPositioned){
    obterAnguloAtual();
    //TODO valores a serem calibrados
    printaAngulo();
    Serial.print("\nDiferença de Elevacao: ");
    Serial.print(angulosPlacaSolar[0] - ultimaElevacao);
    Serial.print(" Diferença de Azimuth: ");
    Serial.println(angulosPlacaSolar[1] - ultimoAzimuth);
    
    if((angulosPlacaSolar[0] - ultimaElevacao) > 5){
      moverMotorElevacao(1024);
    }
    else if((angulosPlacaSolar[1] - ultimoAzimuth) > 5){
      moverMotorAzimuth(1024);
    }
    else if((angulosPlacaSolar[0] - ultimaElevacao) < -5){
      moverMotorElevacao(-1024);
    }
    else if((angulosPlacaSolar[1] - ultimoAzimuth) < -5){
      moverMotorAzimuth(-1024);
    }
    else{
      perfectPositioned = true;      
    }
  }
  Serial.println("Placa posicionada adequadamente!");
}

//posicaoSolarAtual ----------------------------
const double rad =  PI / 180.0;
const double e   =  rad * 23.4397;

double roundRadians(double beta){
	double trash;
	if(beta>2*PI){
		beta = beta/2*PI;
		beta = modf(beta, &trash);
	}
	return beta;
}
double dateToMiliseconds(int sec,int minute,int hour,int day, int month, int year){
	struct tm date = {
		.tm_sec=sec,
		.tm_min=minute,
		.tm_hour=hour,
		.tm_mday=day,
		.tm_mon=month,
		.tm_year=year - 1900
	};
	return ((double)mktime(&date))*1000.0;
}
double ToJulian(double date){
	return (date - dateToMiliseconds(1,0,0,1,1,1970)) / (1000.0 * 60.0 * 60.0 * 24.0) - 0.5 + 2440588.0;
}
double ToDays(double date){
	return ToJulian(date) - 2451545.0;
}
double rightAscencion(double d){
	double m = rad * (357.5291 + 0.98560028 * d);
	double c = rad * (1.9148 * sin(m) + 0.02 * sin(2 * m) + 0.0003 * sin(3 * m));
	double p = rad * 102.9372; 
	double l = m + c + p + PI;
	double ra = atan2((sin(l) * cos(e) - tan(0) * sin(e)),cos(l));
	
	return ra;
}
double declination(double d){
	double m = rad * (357.5291 + 0.98560028 * d);
	double c = rad * (1.9148 * sin(m) + 0.02 * sin(2 * m) + 0.0003 * sin(3 * m));
	double p = rad * 102.9372; 
	double l = m + c + p + PI;
	double dec = asin(sin(0) * cos(e) + cos(0) * sin(e) * sin(l));
	return dec;
}
double sideralTime(double d, double lw){
	return rad * (280.16 + 360.9856235 * d) - lw;
}
double azimuth(double H, double phi, double dec){
	return PI + atan2(sin(H), (cos(H) * sin(phi) - tan(dec) * cos(phi)));
}
double altitude(double H, double phi, double dec){
	return PI + asin(roundRadians(sin(phi) * sin(dec) + cos(phi) * cos(dec) * cos(H)));
}
double* getposition(double date, double lat, double lng){
	double lw = rad * (-lng);
	double phi = rad * lat;
	double d = ToDays(date);
	double RA = rightAscencion(d);
	double dec = declination(d);
	double H = sideralTime(d,lw) - RA;

	double az = azimuth(H,phi,dec);
	double alt = altitude(H,phi,dec);

	static double result[2];
	result[0] = 360 - (az)*180.0/PI;
	result[1] = (alt)*180.0/PI - 180;
	return result;
} 
void posicaoSolarAtual(){
  double *ret;
	double now;
	
  now = (double)dateToMiliseconds(dataAtual[0],dataAtual[1],(24 - dataAtual[2])+3,dataAtual[3],dataAtual[4],dataAtual[5]); 
	ret = getposition(now,gpsAtual[0],gpsAtual[1]);

  ultimaElevacao = *(ret + 1);
  ultimoAzimuth  = *(ret + 0);
  printaposicaoSolarAtual();//DEBUG REMOVE AFTER
}
void printaposicaoSolarAtual(){
  Serial.println("Pegando posição do sol: ");
  Serial.print("Elevacao: ");
  Serial.print(ultimaElevacao);
  Serial.print(" Azimute: ");
  Serial.println(ultimoAzimuth);
}
//posicaoSolarAtualFim -------------------------

