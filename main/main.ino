#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <SoftwareSerial.h>
#include <ESP32Time.h>
#include <TinyGPSPlus.h>

#include "Wire.h"
#include "DallasTemperature.h"

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>



//----------- Bloco de mapeamento dos pinos -----------
//Pinos Digitais
#define DIGITAL1 47
#define DIGITAL2 48
#define temp_pin 0

//Pinos SD
#define SD_CS    10
#define SD_MOSI  11
#define SD_SCK   12
#define SD_MISO  13

//Pinos SPI (LORA e ADC)
#define VSPI_MISO 42
#define VSPI_MOSI 41
#define VSPI_SCLK 40
#define VSPI_SS   39
#define VSPI_SS2  38

#define RFM_RESET 37
#define RFM_DIO0  7
#define RFM_DIO1  6
#define RFM_DIO2  5


//Pinos I2C (Conversor 4-20mA para 5v e conector I2C para sensores)
#define I2C_SCL  14
#define I2C_SDA  21

//Pinos GPS
#define RXPin  19
#define TXPin  20

//Pinos do RS485
#define RS485_RXPin 18
#define RS485_TXPin 17
#define RS485_DE    31


//tempo de atualização em milisegundos
#define LOG_RATE 10000 
unsigned long LAST_LOG = 0; 

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;


//Variaveis Sensor Temperatura
OneWire oneWire(temp_pin);
DallasTemperature dallasTemperature(&oneWire);

//Variáveis GPS
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);

//Variáveis SD
SPIClass mySPI = SPIClass(HSPI);



//----------- Bloco Atlas ENV50-----------
#include <Ezo_i2c.h>
#include <Ezo_i2c_util.h>

Ezo_board ENV50_PH = Ezo_board(99, "PH");
Ezo_board ENV50_ORP = Ezo_board(62, "ORP");
Ezo_board ENV50_TEMP = Ezo_board(98, "TEMP");
Ezo_board ENV50_DO = Ezo_board(97, "DO");


ESP32Time rtc(-10800);



static const u1_t PROGMEM APPEUI[8] = { 0x1F, 0xBC, 0x05, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 }; //lsb format
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }
static const u1_t PROGMEM DEVEUI[8]  = { 0x4C, 0xBE, 0x05, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 }; // lsb format
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }
static const u1_t PROGMEM APPKEY[16] = { 0x98, 0x50, 0x5E, 0x8C, 0xA5, 0x70, 0x3D, 0x2C, 0xA8, 0xFB, 0x32, 0x43, 0x58, 0xED, 0x72, 0x2F }; //msb format
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }

byte payload[8]; // Enviando um vetor de bytes - Prática 2, parte 2
static uint16_t mydata = 0; // Variável do contador - Prática 2, parte 2
static osjob_t sendjob;






// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = VSPI_SS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = RFM_RESET,
    .dio = {RFM_DIO0, RFM_DIO1, LMIC_UNUSED_PIN},
};

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
	    // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;

        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, payload, sizeof(payload), 0);
        Serial.println("Packet queued " + String(mydata));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}


void setupTTN(){
    SPI.begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, VSPI_SS);
  
    LMIC_selectSubBand(1);
    LMIC_setLinkCheckMode(0);

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}


String readSensors(){
    String dataString = "";
    float pH, DO, ORP, temp;


    pH = readPH_ENV50_i2c();
    //ORP = readORP_ENV50_i2c();
    //DO = readDO_ENV50_i2c();
    //temp = readTEMP_ENV50_i2c();



    dataString += "ph=";
    dataString += pH;
    dataString += ";";
    dataString += "orp=";
    dataString += ORP;
    dataString += ";";
    dataString += "od=";
    dataString += DO;
    dataString += ";";
    dataString += "temp=";
    dataString += temp;
    dataString += ";";




    //pH
    payload[0] = (int)pH;
    payload[1] = (int)((pH-payload[0])*100);
    //DO
    payload[2] = (int)DO;
    payload[3] = (int)((DO-payload[2])*100);
    //ORP
    payload[4] = (int)ORP;
    payload[5] = (int)((ORP-payload[4])*100);
    //temp
    payload[6] = (int)temp;
    payload[7] = (int)((temp-payload[6])*100);

    return dataString;
}



//------------------------------------- BLOCO SD -------------------------------------------
void initSD() {
  mySPI.begin(SD_SCK, SD_MISO, SD_MOSI);
  if(!SD.begin(SD_CS, mySPI, 10000000)){
    Serial.println("Erro ao inicializar o cartão SD");
  }

  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("Nenhum cartão SD foi encontrado");
  }
}



//------------------------------------- BLOCO GPS -------------------------------------------
void initGPS() {
  gpsSerial.begin(GPSBaud);

  //Aguarda o checksum e atualiza a data interna
  Serial.print("Inicializando GPS .");
  while (gps.charsProcessed() < 2000) {
    //Serial.print(gps.failedChecksum());
    Serial.print(".");
    Serial.print(gps.sentencesWithFix());
    smartGPSDelay(1000);
  }

  rtc.setTime(gps.time.second(), gps.time.minute(), gps.time.hour(), gps.date.day(), gps.date.month(), gps.date.year());
  if (gps.date.day() == 0) {
     rtc.setTime(0, 0, 3, 1, 1, 2020);
  } 
    
  Serial.print(" Inicializado com sucesso com ");
  Serial.print(gps.satellites.value());
  Serial.println(" satelites!");
  
  Serial.print(F("Data: "));
  printGPSDateTime(gps.date, gps.time);
  printGPSLocation();
}

static void smartGPSDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read());
      //Serial.println(gpsSerial.read());
    }

  } while (millis() - start < ms);
}

static void printGPSDateTime(TinyGPSDate &d, TinyGPSTime &t) {
  if (!d.isValid()) {
    Serial.print("Data inválida");
  } else {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.day(), d.month(), d.year());
    Serial.print(sz);
  }

  if (!t.isValid()) {
    Serial.print("Hora inválida");
  } else {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }
}

static void printGPSLocation() {
  Serial.print(F("Localização: "));
  if (gps.location.isValid()) {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  } else {
    Serial.println(F("INVÁLIDA "));
  }
}

static String getGPSData() {
  String data = "";
  data += gps.time.hour();
  data += ":";
  data += gps.time.minute();
  data += ":";
  data += gps.time.second();
  data += " ";
  data += gps.date.day();
  data += "/";
  data += gps.date.month();
  data += "/";
  data += gps.date.year();

  return data;
}

String getLocation() {
  String location = "";
  if (gps.location.isValid()) {
    location += "lat=";
    location += String(gps.location.lat(), 6);
    location += ";";
    location += "lng=";
    location += String(gps.location.lng(), 6);
    location += ";";
  }
  return location;
}

//------------------------------------- BLOCO DATA -------------------------------------------
void initRTC() {
  //Seta a data interna para 2023-01-01
  rtc.setTime(0, 0, 0, 1, 1, 2023);
}

String getFileName() {
  String fileName = "/";
  String strDay = "";
  if (rtc.getDay() < 10) {
    strDay += "0";
    strDay += rtc.getDay();
  } else {
    strDay += rtc.getDay();
  }
  fileName += strDay;

  String strMonth = "";
  if ((rtc.getMonth() + 1) < 10) {
    strMonth += "0";
    strMonth += rtc.getMonth() + 1;
  } else {
    strMonth += rtc.getMonth() + 1;
  }

  fileName += strMonth;
  fileName += rtc.getYear();

  fileName += ".txt";
  return fileName;
}

String getFormatedDate() {
  String date = "";

  String strDay = "";
  if (rtc.getDay() < 10) {
    strDay += "0";
    strDay += rtc.getDay();
  } else {
    strDay += rtc.getDay();
  }
  date += strDay;
  date += '/';

  String strMonth = "";
  if ((rtc.getMonth() + 1) < 10) {
    strMonth += "0";
    strMonth += rtc.getMonth() + 1;
  } else {
    strMonth += rtc.getMonth() + 1;
  }
  date += strMonth;
  date += '/';

  date += rtc.getYear();
  date += ' ';
  date += rtc.getHour(true);
  date += ':';
  date += rtc.getMinute();
  date += ':';
  date += rtc.getSecond();
  return date;
}

void printCurrentDate() {
  Serial.print(rtc.getTime("%d/%m/%Y %H:%M:%S"));
}
void printCurrentGPSDate() {
  printGPSDateTime(gps.date, gps.time);
}

time_t syncProvider() {
  return rtc.getEpoch();
}


//------------------------------------- BLOCO ATLAS ENV50 -------------------------------------------
float readPH_ENV50_i2c() {
  ENV50_PH.send_read_cmd();
  //delay(1000);
  smartGPSDelay(0);
  delay(1000);
  ENV50_PH.receive_read_cmd();
  return ENV50_PH.get_last_received_reading();
  //  receive_and_print_reading(PH);
  //  Serial.println();
}

float readORP_ENV50_i2c() {
  ENV50_ORP.send_read_cmd();
  //delay(1000);
  smartGPSDelay(0);
  delay(1000);
  ENV50_ORP.receive_read_cmd();
  return ENV50_ORP.get_last_received_reading();
  //  receive_and_print_reading(ORP);
  //  Serial.println();
}

float readDO_ENV50_i2c() {
  ENV50_DO.send_read_cmd();
  //delay(1000);
  smartGPSDelay(0);
  delay(1000);
  ENV50_DO.receive_read_cmd();
  return ENV50_DO.get_last_received_reading();
  //  receive_and_print_reading(DO);
  //  Serial.println();
}

float readTEMP_ENV50_i2c() {
  ENV50_TEMP.send_read_cmd();
  //delay(1000);
  smartGPSDelay(0);
  delay(1000);
  ENV50_TEMP.receive_read_cmd();
  return ENV50_TEMP.get_last_received_reading();
  //  receive_and_print_reading(TEMP);
  //  Serial.println();
}



void setup() {
    Serial.begin(115200);
    Serial.println(F("Starting"));

    Wire.setPins(I2C_SDA, I2C_SCL);
    Wire.begin();  

    //Inicializa GPS
    initGPS();

    //Inicializa o SD
    initSD();


    setupTTN();

    // dallasTemperature.begin();


}

void loop() {

    if (LAST_LOG <= millis()){

        LAST_LOG = millis() + LOG_RATE;
        printCurrentDate();

        readSensors();
    }

    os_runloop_once();
}

