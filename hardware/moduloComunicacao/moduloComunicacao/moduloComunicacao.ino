#include <FirebaseESP32.h>
#include <FirebaseJson.h>
#include <WiFi.h>

//Retirar depois
#include <virtuabotixRTC.h>

virtuabotixRTC myRTC(14, 12, 13);

//

#define FIREBASE_HOST "https://sunny-spots-757b9.firebaseio.com/" //Do not include https:// in FIREBASE_HOST
#define FIREBASE_AUTH "7OcygkhYndryXxWsa8i2QrGoZxqgvEPEkpzxT9Q9"
#define WIFI_SSID "NET_2GDF6789"
#define WIFI_PASSWORD "B3DF6789"
FirebaseData firebaseData;
FirebaseJson json;

String pathSet = "/device-1/reading_now";
String pathPuse = "/device-1/readings";


void setup() {
  Serial.begin(9600);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);
  //Set database read timeout to 1 minute (max 15 minutes)
  Firebase.setReadTimeout(firebaseData, 1000 * 60);
  //tiny, small, medium, large and unlimited.
  //Size and its write timeout e.g. tiny (1s), small (10s), medium (30s) and large (60s).
  Firebase.setwriteSizeLimit(firebaseData, "small");

  
  myRTC.setDS1302Time(00, 58, 23, 2, 17, 11, 2014);

}

char *ssid = "NET_2GDF6789";
char *password = "B3DF6789";

void loop() {
  //sendInformationToFirebase(-8.0, 34.5, 100.0, 50.0, 50, 45);
  //Le as informacoes do CI
  myRTC.updateTime(); 
  //Imprime as informacoes no serial monitor
  Serial.print("Data : ");
  //Chama a rotina que imprime o dia da semana
  imprime_dia_da_semana(myRTC.dayofweek);
  Serial.print(", ");
  Serial.print(myRTC.dayofmonth);
  Serial.print("/");
  Serial.print(myRTC.month);
  Serial.print("/");
  Serial.print(myRTC.year);
  Serial.print("  ");
  Serial.print("Hora : ");
  //Adiciona um 0 caso o valor da hora seja <10
  if (myRTC.hours < 10)
  {
    Serial.print("0");
  }
  Serial.print(myRTC.hours);
  Serial.print(":");
  //Adiciona um 0 caso o valor dos minutos seja <10
  if (myRTC.minutes < 10)
  {
    Serial.print("0");
  }
  Serial.print(myRTC.minutes);
  Serial.print(":");
  //Adiciona um 0 caso o valor dos segundos seja <10
  if (myRTC.seconds < 10)
  {
    Serial.print("0");
  }
  Serial.println(myRTC.seconds);
  delay( 1000);
}
void imprime_dia_da_semana(int dia)
{
  switch (dia)
  {
    case 1:
    Serial.print("Domingo");
    break;
    case 2:
    Serial.print("Segunda");
    break;
    case 3:
    Serial.print("Terca");
    break;
    case 4:
    Serial.print("Quarta");
    break;
    case 5:
    Serial.print("Quinta");
    break;
    case 6:
    Serial.print("Sexta");
    break;
    case 7:
    Serial.print("Sabado");
    break;
  }
}

bool sendInformationToFirebase(double lat, double lng, double horizontal, double vertical, int lvl, int lumus){
  FirebaseJson gps, angleOfSolarPanel;
  gps.clear().add("lat", int(lat));
  gps.add("lng", int(lng));
  angleOfSolarPanel.clear().add("horizontal", int(horizontal));
  angleOfSolarPanel.add("vertical", int(vertical));
  Serial.println("Sending data to Firebase");
  json.clear().add("gps", gps);
  json.add("angleOfSolarPanel", angleOfSolarPanel);
  json.add("battery",int(lvl));
  json.add("luminosity",int(lumus));

  if (!Firebase.setJSON(firebaseData, pathSet, json)){
      Serial.println("FAILED");
      Serial.println("REASON: " + firebaseData.errorReason());
      Serial.println("------------------------------------");
      Serial.println();
      return false;
  }
  if(!Firebase.pushJSON(firebaseData, pathPuse, json)){
      Serial.println("FAILED");
      Serial.println("REASON: " + firebaseData.errorReason());
      Serial.println("------------------------------------");
      Serial.println();
      return false;
  }  
  return true;
}
