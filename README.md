Voici une description détaillée du code contenu dans le fichier V01_htcc_gps_neo6M_LORA.cpp :

Description du Code
Le fichier V01_htcc_gps_neo6M_LORA.cpp contient le code pour un projet utilisant un module GPS Neo-6M et un module LoRa pour envoyer des données GPS via un réseau LoRaWAN. 


Voici une description des principales fonctions et de leur fonctionnement :

Fonctions Principales
sendShortMsg()

Cette fonction envoie un message court contenant des informations spécifiques, telles que la date actuelle et l'état de certaines broches numériques.
Elle utilise sprintf pour formater les données dans un tampon (txpacket) et Radio.Send pour envoyer le message via LoRa.
```cpp
void sendShortMsg() {
  sprintf(txpacket, "");
  Serial.println("\r\nsending day: " + String(gps.date.day()) + " year " + String(gps.date.year()));
  sprintf(txpacket, "ssa: %d/%d/%d/%d/%d", digitalRead(Vext), gps.date.day(), gps.date.month(), gps.date.year(), digitalRead(UART_RX2));
  Radio.Send((uint8_t *)txpacket, strlen(txpacket)); //send the package out
}
```
sendMsg()

Cette fonction vérifie si la localisation GPS est valide en utilisant gps.location.isValid().
Si la localisation est valide, elle formate les coordonnées GPS (latitude et longitude) dans un tampon (txpacket) et envoie le message via LoRa.
La fonction utilise sprintf pour formater les coordonnées GPS et Radio.Send pour envoyer le message.
```cpp
void sendMsg() {
  if (gps.location.isValid()) {
    sprintf(txpacket, "");
    Serial.println("\r\nsending packet");
    sprintf(txpacket, "lat: %d.%d lng: %d.%d", (int)gps.location.lat(), abs(fracPart(gps.location.lat(), 6)), (int)gps.location.lng(), abs(fracPart(gps.location.lng(), 5)));
    Radio.Send((uint8_t *)txpacket, strlen(txpacket)); //send the package out
  }
}
```
prepareTxFrame(uint8_t port)

Cette fonction prépare la trame de données à envoyer via LoRa.
Elle vérifie si la localisation GPS est valide avant de formater les données.
Les données incluent la tension de la batterie, la latitude et la longitude GPS.
La fonction utilise digitalRead pour lire l'état des broches numériques et gps.encode pour encoder les données GPS.
```cpp
void prepareTxFrame(uint8_t port) {
  if (gps.location.isValid()) {
    appDataSize = 6; // nombre total d'octets de la trame envoyée
    appData[0] = (uint8_t)(batteryVoltage >> 8);
    appData[1] = (uint8_t)batteryVoltage;
    appData[2] = (uint8_t)(gps.location.lat());
    appData[3] = (uint8_t)(abs(fracPart(gps.location.lat(), 6)));
    appData[4] = (uint8_t)((int)gps.location.lng());
    appData[5] = (uint8_t)(abs(fracPart(gps.location.lng(), 5)));
  } else {
    Serial.println("GPS location not valid");
  }
}
```
Boucle Principale

La boucle principale du programme lit les données GPS à partir du port série (Serial1).
Si des données sont disponibles, elles sont lues et encodées en utilisant gps.encode.
Si les données GPS sont valides, la fonction sendMsg est appelée pour envoyer les coordonnées GPS.
```cpp
void loop() {
  sendShortMsg();
  Serial.println(digitalRead(UART_RX2));
  delay(1000);
  while (Serial1.available() > 0) {
    if (gps.encode(Serial1.read())) {
      sendMsg();
      return;
    }
  }
}
```


Conclusion
Le fichier V01_htcc_gps_neo6M_LORA.cpp implémente un système qui lit les données GPS à partir d'un module Neo-6M et envoie ces données via un réseau LoRaWAN. Les fonctions principales gèrent la lecture des données GPS, la vérification de leur validité, le formatage des messages et l'envoi des données.