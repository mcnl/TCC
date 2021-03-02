#include <FirebaseESP32.h>
#include <FirebaseJson.h>
#include <WiFi.h>

#define FIREBASE_HOST "https://sunny-spots-757b9.firebaseio.com/" //Do not include https:// in FIREBASE_HOST
#define FIREBASE_AUTH "7OcygkhYndryXxWsa8i2QrGoZxqgvEPEkpzxT9Q9"
#define WIFI_SSID "NET_2GDF6789"
#define WIFI_PASSWORD "B3DF6789"
FirebaseData firebaseData;
FirebaseJson json;

String pathSet = "/device-1/readnow";
String pathPuse = "/device-1/reads";


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
}

char *ssid = "NET_2GDF6789";
char *password = "B3DF6789";

void loop() {
  FirebaseJson gps;
  gps.clear().add("0", int(3213.3));
  gps.add("1", int(3213213.45));
  Serial.println("Sending data to Firebase");
  json.clear().add("gps", gps);
  json.add("inclination",int(1));
  json.add("rotacion",int(2));
  json.add("charge",int(3));
  json.add("lums",-9898);
  json.add("power",int(13));

  if (!Firebase.setJSON(firebaseData, pathSet, json)){
      Serial.println("FAILED");
      Serial.println("REASON: " + firebaseData.errorReason());
      Serial.println("------------------------------------");
      Serial.println();
  }
  if(!Firebase.pushJSON(firebaseData, pathPuse, json)){
      Serial.println("FAILED");
      Serial.println("REASON: " + firebaseData.errorReason());
      Serial.println("------------------------------------");
      Serial.println();
  }
}
