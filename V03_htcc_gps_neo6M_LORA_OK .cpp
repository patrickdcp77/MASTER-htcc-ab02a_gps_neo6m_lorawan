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

  /*
  //en miliVolts

  //appData[0] = (uint8_t)(batteryVoltage >> 8);//décalage de 8 bit vers la droite, 
  //4803  donne 0001001011000011 sur 16bit donc reste les 8 1ers bits 00010010  soit 12 hex
  //Serial.println(appData[4]);
  
  //appData[1] = (uint8_t)batteryVoltage;
  //4803  donne 0001001011000011 donc si uint_8  on prend donc 8bit de droite 11000011 soit C3 hex
  // quand on concatène 12C3 hex dans un convertisseur hexa vers décimal, cela donne 4803 mV au décodage de la trame LORA
  //Serial.println(appData[5]);

  //calcul pour la température
  //sur lorawan on obtient 00 74 hex
  //on concatène 00 et 74 , on convertit hexa vers décimal et on obtient 116 décimal
  //on reprend la formule de Sylvain  t_byte = (t + 35 )*2  donc partie entière de t=116/2 moins 35 degrés soit 23 degrés
*/




  //la tension batterie
  uint16_t batteryVoltage = getBatteryVoltage();
  Serial.print(batteryVoltage);
  Serial.println( " mV ");


  if (gps.location.isValid()) {
    appDataSize = 10; // nombre total d'octets de la trame envoyée
    // et à changer selon le nombre de balances et capteurs

    appData[0] = (uint8_t)(batteryVoltage >> 8);

    appData[1] = (uint8_t)batteryVoltage;


    char txpacket[50]; // Ensure txpacket is defined with sufficient size

    sprintf(txpacket, "lat: %d.%d lng: %d.%d", 
        (int)gps.location.lat(), 
        abs(fracPart(gps.location.lat(), 6)), 
        (int)gps.location.lng(), 
        abs(fracPart(gps.location.lng(), 6)));

    Serial.println(txpacket);


    appData[2] = (uint8_t)gps.location.lat();
    Serial.print( " lat ");
    Serial.println(appData[2]);
    
    appData[3] = (uint8_t)((abs(fracPart(gps.location.lat(), 6)) )>> 16);
    Serial.print( " lat ");
    Serial.println(appData[3]);
    appData[4] = (uint8_t)((abs(fracPart(gps.location.lat(), 6)) )>> 8);
    Serial.print( " lat ");
    Serial.println(appData[4]);
    appData[5] = (uint8_t)((abs(fracPart(gps.location.lat(), 6)) ));
    Serial.print( " lat ");
    Serial.println(appData[5]);
    //Serial.println( " lat ");
    //Serial.print(appData[3]);

    //appData[4] = (uint8_t)(abs(fracPart(gps.location.lat(), 6)));
    //Serial.println( " lat ");
    //Serial.print(appData[4]);
    
    appData[6] = (uint8_t)(gps.location.lng());
    Serial.print( " lon ");
    Serial.println(appData[6]);

    appData[7] = (uint8_t)((abs(fracPart(gps.location.lng(), 6)) )>> 16);
    Serial.print( " lon ");
    Serial.println(appData[7]);
    appData[8] = (uint8_t)((abs(fracPart(gps.location.lng(), 6)) )>> 8);
    Serial.print( " lon ");
    Serial.println(appData[8]);
    appData[9] = (uint8_t)((abs(fracPart(gps.location.lng(), 6)) ));
    Serial.print( " lon ");
    Serial.println(appData[9]);
    //Serial.println( " lon ");
    //Serial.print(appData[6]);

    //appData[7] = (uint8_t)(abs(fracPart(gps.location.lng(), 6)));
    //Serial.println( " lon ");
    //Serial.print(appData[7]);

  } else {
    Serial.println("GPS location not valid");
  }
}

/* OTAA para c'est ce OTAA paramêtre qui est utilisé */

uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };


//lora-03 GPS patitrck sur ttn citrouille77@
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xEC, 0xA0 };
uint8_t appKey[] = { 0x7D, 0x95, 0x37, 0x4C, 0xCF, 0xD8, 0x7C, 0x19, 0xE5, 0xF6, 0xA1, 0x9D, 0xFA, 0x2A, 0x2D, 0xE2 };


//lora-04 GPS Bernard en prévision de déclaration sur ttn citrouille77@
//uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xEC, 0xA0 };
//uint8_t appKey[] = { 0x7D, 0x95, 0x37, 0x4C, 0xCF, 0xD8, 0x7C, 0x19, 0xE5, 0xF6, 0xA1, 0x9D, 0xFA, 0x2A, 0x2D, 0xE2 };

/* ABP para*/
uint8_t nwkSKey[] = { 0x37, 0xA9, 0x07, 0xF7, 0xDD, 0x11, 0x89, 0x45, 0x67, 0xF2, 0x02, 0xE5, 0xAB, 0x6C, 0xA0, 0x65 };
uint8_t appSKey[] = { 0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef,0x67 };


//lora-09 GPS
//uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x05, 0xB1, 0xEE };
//uint8_t appKey[] = { 0x1C, 0x92, 0x06, 0x95, 0xD4, 0x03, 0xF4, 0x34, 0xC6, 0x60, 0xB8, 0xC4, 0xFD, 0x90, 0xA2, 0x8D };


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


