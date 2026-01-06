#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <TinyGPS++.h>

#include <cstdint> // Add this include at the top of your file


static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

/*
#define RF_FREQUENCY                                868E6 // Hz

#define TX_OUTPUT_POWER                             20        // dBm

#define LORA_BANDWIDTH                              1         // [0: 125 kHz,
//  1: 250 kHz,
//  2: 500 kHz,
//  3: Reserved]
#define LORA_SPREADING_FACTOR                       11         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
//  2: 4/6,
//  3: 4/7,
//  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false
*/

#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 50 // Define the payload size here

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];
int counter = 0;

static RadioEvents_t RadioEvents;

int16_t rssi, rxSize;

static void smartDelay(unsigned long ms);
void sendShortMsg();
void  DoubleToString( char *str, double double_num, unsigned int len);
void sendMsg();
int fracPart(double val, int n);

void sendShortMsg() {
  sprintf(txpacket, "");
  Serial.println("\r\nsending day: " + String(gps.date.day()) + " year " + String(gps.date.year()));
  Serial.println("lat: " + String(gps.location.lat(), 6) + " lng: " + String(gps.location.lng(), 6));
  sprintf(txpacket, "ssa: %d/%d/%d/%d/%d lat: %d.%d lng: %d.%d", digitalRead(Vext), gps.date.day(), gps.date.month(), gps.date.year(), digitalRead(UART_RX2), (int)gps.location.lat(), abs(fracPart(gps.location.lat(), 6)), (int)gps.location.lng(), abs(fracPart(gps.location.lng(), 6)));
  Radio.Send((uint8_t *)txpacket, strlen(txpacket)); //send the package out
}

/**
    @brief  Double To String
    @param  str: Array or pointer for storing strings
    @param  double_num: Number to be converted
    @param  len: Fractional length to keep
    @retval None
*/
void  DoubleToString( char *str, double double_num, unsigned int len) {
  double fractpart, intpart;
  fractpart = modf(double_num, &intpart);
  fractpart = fractpart * (pow(10, len));
  sprintf(str + strlen(str), "%d", (int)(intpart)); //Integer part
  sprintf(str + strlen(str), ".%d", (int)(fractpart)); //Decimal part
}
int fracPart(double val, int n){
  return (int)((val - (int)(val)) * pow(10, n));
}

void sendMsg() {
  if (gps.location.isValid()) {
    sprintf(txpacket, "");
    Serial.println("\r\nsending packet");
    sprintf(txpacket, "lat: %d.%d lng: %d.%d", (int)gps.location.lat(), abs(fracPart(gps.location.lat(), 6)), (int)gps.location.lng(), abs(fracPart(gps.location.lng(), 6)));
    //sprintf(txpacket,"lng: %d.%d",(int)gps.location.lng(),abs(fracPart(gps.location.lng(),6)));
    Radio.Send( (uint8_t *)txpacket, strlen(txpacket) ); //send the package out
  }
}


static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial1.available())
      gps.encode(Serial1.read());
  } while (millis() - start < ms);
}


void prepareTxFrame(uint8_t port) {
    if (gps.location.isValid()) {
        int32_t lat_enc = gps.location.lat() * 10000;  // Conversion en entier sur 3 octets
        int32_t lon_enc = gps.location.lng() * 10000;

        uint16_t alt_enc = (uint16_t)gps.altitude.meters();
        uint8_t hdop_enc = (uint8_t)(gps.hdop.hdop() * 10);
        uint8_t sats_enc = (uint8_t)gps.satellites.value();

        appDataSize = 10; // 10 octets au total

        // Encodage de la latitude (24 bits)
        appData[0] = (lat_enc >> 16) & 0xFF;
        appData[1] = (lat_enc >> 8) & 0xFF;
        appData[2] = lat_enc & 0xFF;

        // Encodage de la longitude (24 bits)
        appData[3] = (lon_enc >> 16) & 0xFF;
        appData[4] = (lon_enc >> 8) & 0xFF;
        appData[5] = lon_enc & 0xFF;

        // Encodage de l'altitude (16 bits)
        appData[6] = (alt_enc >> 8) & 0xFF;
        appData[7] = alt_enc & 0xFF;

        // Encodage du HDOP et des satellites (8 bits chacun)
        appData[8] = hdop_enc;
        appData[9] = sats_enc;

        Serial.print("Latitude envoyée : "); Serial.println(gps.location.lat(), 6);
        Serial.print("Longitude envoyée : "); Serial.println(gps.location.lng(), 6);
        Serial.print("Altitude envoyée : "); Serial.println(gps.altitude.meters());
        Serial.print("HDOP envoyé : "); Serial.println(gps.hdop.hdop());
        Serial.print("Satellites envoyés : "); Serial.println(gps.satellites.value());
    } else {
        Serial.println("Erreur : Données GPS non valides !");
    }
}

/* OTAA para c'est ce OTAA paramêtre qui est utilisé */

uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
//lora-03 GP
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x05, 0xB1, 0xEA };
uint8_t appKey[] = { 0xDF, 0x6F, 0x66, 0x43, 0x25, 0x79, 0x56, 0x07, 0x11, 0x06, 0x0D, 0x5A, 0x39, 0x4B, 0x9C, 0x36 };
/* ABP para*/
uint8_t nwkSKey[] = { 0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda,0x85 };
uint8_t appSKey[] = { 0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef,0x67 };

uint32_t devAddr =  ( uint32_t )0x007e6ae1;

/*LoraWan channelsmask, default channels 0-7*/ 
uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = LORAWAN_CLASS;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 10000;//900000;//////////////à modifier pour changer cycle d'envoi à TTN  900 000 = 15mn

/*OTAA or ABP*/
bool overTheAirActivation = LORAWAN_NETMODE;

/*ADR enable*/
bool loraWanAdr = LORAWAN_ADR;

/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = LORAWAN_NET_RESERVE;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = LORAWAN_UPLINKMODE;

/* Application port */
uint8_t appPort = 2;
uint8_t confirmedNbTrials = 4;

int maxtry = 50;

/*!
   \brief   Prepares the payload of the frame
*/

void setup() {
  boardInitMcu();
  pinMode(Vext, OUTPUT);
  pinMode(UART_TX2, OUTPUT);
  pinMode(UART_RX2, OUTPUT);
  digitalWrite(Vext, LOW);
  pinMode(UART_RX,OUTPUT);
  digitalWrite(UART_RX,1);
  digitalWrite(UART_TX2, LOW);
  digitalWrite(UART_RX2, LOW);
  delay(500);

  Serial.begin(9600);

  Serial1.begin(GPSBaud);

  rssi = 0;
/*
  Radio.Init( &RadioEvents );
  Radio.SetChannel( RF_FREQUENCY );
  Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                     LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                     LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                     true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );
*/

}

void loop()
{


  //sendShortMsg();
  //Serial.println(digitalRead(UART_RX2));
  //delay(1000);
  while (Serial1.available() > 0) {
    if (gps.encode(Serial1.read())) {
      //sendMsg();
     
  

  switch( deviceState )
    {
      case DEVICE_STATE_INIT:
      {
    #if(LORAWAN_DEVEUI_AUTO)
          LoRaWAN.generateDeveuiByChipID();
    #endif
    #if(AT_SUPPORT)
          getDevParam();
    #endif
        printDevParam();
        LoRaWAN.init(loraWanClass,loraWanRegion);
        deviceState = DEVICE_STATE_JOIN;
        break;
      }
      case DEVICE_STATE_JOIN:
      {
        LoRaWAN.join();
        break;
      }
      case DEVICE_STATE_SEND:
      {
        prepareTxFrame( appPort );
        LoRaWAN.send();
        deviceState = DEVICE_STATE_CYCLE;
        break;
      }
      case DEVICE_STATE_CYCLE:
      {
        // Schedule next packet transmission
        txDutyCycleTime = appTxDutyCycle + randr( 0, APP_TX_DUTYCYCLE_RND );
        LoRaWAN.cycle(txDutyCycleTime);
        deviceState = DEVICE_STATE_SLEEP;
        break;
      }
      case DEVICE_STATE_SLEEP:
      {
        LoRaWAN.sleep();
        break;
      }
      default:
      {
        deviceState = DEVICE_STATE_INIT;
        break;
      }
	}
      return;
    }
  }
  
}


