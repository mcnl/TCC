//
// Copyright 2015 Google Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

// FirebaseDemo_ESP32 is a sample that demo the different functions
// of the FirebaseArduino API.

#include <WiFi.h>
#include <IOXhop_FirebaseESP32.h>

// Set these to run example.
#define FIREBASE_HOST "sunny-spots-757b9.firebaseio.com/"
#define FIREBASE_AUTH "7OcygkhYndryXxWsa8i2QrGoZxqgvEPEkpzxT9Q9"
#define WIFI_SSID "NET_2GDF6789"
#define WIFI_PASSWORD "B3DF6789"

void setup() {
  Serial.begin(9600);

  // connect to wifi.
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("connected: ");
  Serial.println(WiFi.localIP());
  
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
}

int n = 0;

void loop() {
  
  // get value 
  Serial.print(pegarDatadoFirebase("mes"));
  Serial.print(pegarLocaldoFirebase("latitude"));
}
bool enviarInformacaoFirebase(float lat, float lng, float horizontal, float vertical, int nivelBateria, int luminosidade){
  
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

int pegarDatadoFirebase(String unidadeTempo){
  int resposta = Firebase.getInt("/device-1/data/"+unidadeTempo);
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
