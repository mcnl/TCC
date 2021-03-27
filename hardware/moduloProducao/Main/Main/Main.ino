//Modulo de Producao - Código Principal
//Escrito e projetado por Matheus Casa Nova da Luz

#define Nsta 3     // Two state values: pressure, temperature
#define Mobs 3     // Three measurements: baro pressure, baro temperature, LM35 temperature


#include <TinyEKF.h>

#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <Wire.h>

#include <Stepper.h> 

#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#include <virtuabotixRTC.h> 

//Variaveis do Modulo de Produção
//Sensor de Angulagem
const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
 
int minVal=265;
int maxVal=402;

class Fuser : public TinyEKF {

    public:

        Fuser()
        {            
            // We approximate the process noise using a small constant
            this->setQ(0, 0, .0001);
            this->setQ(1, 1, .0001);

            // Same for measurement noise
            this->setR(0, 0, .0001);
            this->setR(1, 1, .0001);
            this->setR(2, 2, .0001);
        }

    protected:

        void model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta])
        {
            // Process model is f(x) = x
            fx[0] = this->x[0];
            fx[1] = this->x[1];

            // So process model Jacobian is identity matrix
            F[0][0] = 1;
            F[1][1] = 1;

            // Measurement function simplifies the relationship between state and sensor readings for convenience.
            // A more realistic measurement function would distinguish between state value and measured value; e.g.:
            //   hx[0] = pow(this->x[0], 1.03);
            //   hx[1] = 1.005 * this->x[1];
            //   hx[2] = .9987 * this->x[1] + .001;
            hx[0] = this->x[0]; // Barometric pressure from previous state
            hx[1] = this->x[1]; // Baro temperature from previous state
            hx[2] = this->x[1]; // LM35 temperature from previous state

            // Jacobian of measurement function
            H[0][0] = 1;        // Barometric pressure from previous state
            H[1][1] = 1 ;       // Baro temperature from previous state
            H[2][1] = 1 ;       // LM35 temperature from previous state
        }
};

 Fuser ekf;
 
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
  //dataHoraAtual();
}

void loop() {
  // put your main code here, to run repeatedly:
  //if(myRTC.hours - dataAtual[2] == 1 || myRTC.hours - dataAtual[2] == -23)
  char result = mainmoduloProducao();
  delay(5000);
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
  myStepper_horizontal.setSpeed(10);
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
  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);
  
  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();

  double out[3] = {AcX, AcY, AcZ};
  ekf.step(out);
    
  int xAng = map(out[0],minVal,maxVal,0,360);
  int yAng = map(out[1],minVal,maxVal,0,360);
  int zAng = map(out[2],minVal,maxVal,0,360);
  
  angulosPlacaSolar[0] = RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
  angulosPlacaSolar[1] = RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
  angulosPlacaSolar[2] = RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);

  
  return true;
}
void printaAngulo(){
  Serial.println("Pegando Angulo Atual");
  Serial.print("Angulo Eixo X: ");
  Serial.print(angulosPlacaSolar[0]);
  Serial.print(" Angulo Eixo Y: ");
  Serial.println(angulosPlacaSolar[1]);
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
    printaAngulo();
    Serial.print("\nDiferença de Elevacao: ");
    Serial.println(angulosPlacaSolar[0] - ultimaElevacao);
    Serial.print(" Diferença de Azimuth: ");
    Serial.println(angulosPlacaSolar[1] - ultimoAzimuth);
    
    if((angulosPlacaSolar[0] - ultimaElevacao) > 5){
      //moverMotorElevacao(3000);
    }
    else if((angulosPlacaSolar[1] - ultimoAzimuth) > 5){
      //moverMotorAzimuth(1000);
    }
    else if((angulosPlacaSolar[0] - ultimaElevacao) < -5){
      //moverMotorElevacao(-3000);
    }
    else if((angulosPlacaSolar[1] - ultimoAzimuth) < -5){
     //moverMotorAzimuth(-1000);
    }
    else{
      perfectPositioned = true;      
    }
    delay(500);
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
