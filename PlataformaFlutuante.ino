#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <SoftwareSerial.h>
#include <Time.h>
#include <TimeAlarms.h>
#include <ESP32Time.h>

//----------- Bloco de mapeamento dos pinos -----------
//Pinos Digitais
static const int DIGITAL1 = 47;
static const int DIGITAL2 = 48;

//Pinos SD
static const int SD_CS = 10;
static const int SD_MOSI = 11;
static const int SD_SCK = 12;
static const int SD_MISO = 13;

//Pinos SPI (LORA e ADC)
static const int VSPI_MISO = 42;
static const int VSPI_MOSI = 41;
static const int VSPI_SCLK = 40;
static const int VSPI_SS = 39;
static const int VSPI_SS2 = 38;

//Pinos I2C (Conversor 4-20mA para 5v e conector I2C para sensores)
static const int I2C_SCL = 14;
static const int I2C_SDA = 21;

//Pinos GPS
static const int RXPin = 19, TXPin = 20;

//Pinos do RS485
static const int RS485_RXPin = 18, RS485_TXPin = 17;
static const int RS485_DE = 31;

//----------- Bloco Módulos Adicionais-----------
#include <TinyGPSPlus.h>
#include <MCP3208.h>

//Variáveis ADC
MCP3208 adc;

//Variáveis GPS
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);

//----------- Bloco Atlas ENV20-----------
#include "ph_grav.h"
#include "orp_grav.h"
#include "do_grav.h"

//Variáveis ENV20 Gravity
//A0 da placa é o endereço 7 do ADC, o A5 é o endereço =2
static const int ENV20_PH_PIN = 7;
static const int ENV20_ORP_PIN = 6;
static const int ENV20_OD_PIN = 5;
Gravity_pH pH = Gravity_pH(ENV20_PH_PIN);
Gravity_ORP ORP = Gravity_ORP(ENV20_ORP_PIN);
Gravity_DO DO = Gravity_DO(ENV20_OD_PIN);

//----------- Bloco Atlas ENV50-----------
#include <Ezo_i2c.h>
#include <Ezo_i2c_util.h>

Ezo_board ENV50_PH = Ezo_board(99, "PH");
Ezo_board ENV50_ORP = Ezo_board(62, "ORP");
Ezo_board ENV50_TEMP = Ezo_board(98, "TEMP");
Ezo_board ENV50_DO = Ezo_board(97, "DO");

//----------- Bloco de controle -----------
boolean failSD = false;
boolean hasError = false;
boolean alarmed = false;
volatile int mainLoopCounter = 0;

//Offset do RTC com -3 (GMT -3)
ESP32Time rtc(-10800);

//Variavel que controla em quantos minutos deve ser registrado o dado
int INTERVALO_COLETA = 1;
//Variavel que controla a cada quantos registros a informação deve ser enviada pela serial
int INTERVALO_TRANSMISSAO = 1;
//Caso debug ativo, faz com que o intervalo de transmissão seja de segundos e não de minutos (para facilitar o teste)
boolean DEBUG = false;
boolean ATLAS_ENV20 = false;
boolean ATLAS_ENV50 = true;

void setup() {
  Wire.setPins(I2C_SDA, I2C_SCL);
  Wire.begin();   
  Serial.begin(115200);  

  //Inicializa GPS
  initGPS();

  //Inicializa o SD
  initSD();

  //Inicializa o ADC
  initADC();

  //Inicializa os Sensores
  initSensors();

  //Se o SD não falhou, inicializa as rotinas
  if (!failSD) {
    //Sincroniza o metodo se alarme
    setSyncProvider(syncProvider);
    setSyncInterval(600);
    printCurrentDate();
  }
}

void loop() {
  //Se tiver sido alarmado, incrementa 1 segundo. Caso contrario, incrementa o loop.
  if (alarmed) {
    //Necessario para incrementar o contador de segundos do alarme.
    printCurrentDate();
    Serial.println("");
    smartGPSDelay(0);
    Alarm.delay(1000);    
  } else {
    printCurrentDate();
    Serial.println("");
    smartGPSDelay(0);
    delay(1000);    
  }

  if (!alarmed) {
    //Quanto o resto da divisão do minuto pelo intervalo configurado for 0, alarma
    if (DEBUG) {
      if ((rtc.getSecond() % INTERVALO_COLETA) == 0) {
        alarm();
      }
    } else {
      if ((rtc.getMinute() % INTERVALO_COLETA) == 0) {
        alarm();
      } else
        //Se intervalo = 60, testar minuto 0
        if (DEBUG) {
          if ((rtc.getSecond() == 0) && (INTERVALO_COLETA == 60)) {
            alarm();
          }
        } else {
          if ((rtc.getMinute() == 0) && (INTERVALO_COLETA == 60)) {
            alarm();
          }
        }
    }
  }
}

void alarm() {
  alarmed = true;
  Alarm.timerRepeat((INTERVALO_COLETA * 60), mainLoop);

  Serial.println("Sistema alarmado com sucesso");
  Serial.println("");

  //Chama loop principal
  mainLoop();
}

void mainLoop() {
  mainLoopCounter++;

  //Chama a escrita em arquivo e envio, de acordo com a quantidade configurada para ser acumulada antes de enviar.
  if (mainLoopCounter == INTERVALO_TRANSMISSAO) {
    fileLoop();
    mainLoopCounter = 0;
  }

  //Exibe memoria disponivel depois do processo
  Serial.println("Processo de escrita executado com sucesso!");
}

void fileLoop() {
  //Instancia String de dados
  String dataString = "";

  //Gerando arquivo (um por dia)
  String fileName = getFileName();

  File file;
  if (SD.exists(fileName.c_str())) {
    file = SD.open(fileName.c_str(), FILE_APPEND);
    if (DEBUG) {
      Serial.println("file append");
    }

  } else {
    file = SD.open(fileName.c_str(), FILE_WRITE);
    if (DEBUG) {
      Serial.println("file write");
    }
  }

  Serial.print("Abrindo o arquivo: ");
  Serial.print(fileName);
  Serial.print(" ... ");

  //Inicia a linha do arquivo com data, latitude e longitude
  dataString += "dt=";
  dataString += getFormatedDate();
  dataString += ";";
  dataString += getLocation();
  dataString += getSensorsValues();


  //Se o arquivo abrir corretamente, escrever.
  if (file) {
    file.println(dataString);
    file.close();
    Serial.print("Escreveu a leitura:");
    Serial.println(dataString);
  } else {
    //Se o arquivo nao abrir, exibir erro.
    Serial.println("Erro ao abrir o arquivo");
  }
}

//------------------------------------- BLOCO SD -------------------------------------------
void initSD() {
  if (!SD.begin()) {
    Serial.println("Erro ao inicializar o cartão SD");
    failSD = true;
    hasError = true;
  }

  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("Nenhum cartão SD foi encontrado");
    failSD = true;
    hasError = true;
  }
}

//------------------------------------- BLOCO ADC -------------------------------------------
void initADC() {
  adc.begin(VSPI_SCLK, VSPI_MOSI, VSPI_MISO, VSPI_SS2);
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


//------------------------------------- BLOCO SENSORES -------------------------------------------
void initSensors() {
  if (ATLAS_ENV20) {
    initENV20_pH();
    initENV20_ORP();
    initENV20_OD();
  }
  if (ATLAS_ENV50) {
  }
}


String getSensorsValues() {
  String dataString = "";
  if (ATLAS_ENV20) {
    dataString += "ph=";
    dataString += readENV20_pH();
    dataString += ";";
    dataString += "orp=";
    dataString += readENV20_ORP();
    dataString += ";";
    dataString += "od=";
    dataString += readENV20_OD();
    dataString += ";";
    // dataString += "temp=";
    // dataString += readENV20_OD();
    // dataString += ";";
  }

  if (ATLAS_ENV50) {
    dataString += "ph=";
    dataString += readPH_ENV50_i2c();
    dataString += ";";
    dataString += "orp=";
    dataString += readORP_ENV50_i2c();
    dataString += ";";
    dataString += "od=";
    dataString += readDO_ENV50_i2c();
    dataString += ";";
    dataString += "temp=";
    dataString += readTEMP_ENV50_i2c();
    dataString += ";";
  }

  return dataString;
}
//------------------------------------- BLOCO ATLAS ENV20 -------------------------------------------
void initENV20_pH() {
  if (pH.begin()) {
    Serial.println("Erro ao inicializar sensor de PH");
  }
}

void initENV20_ORP() {
  if (pH.begin()) {
    Serial.println("Erro ao inicializar sensor de ORP");
  }
}

void initENV20_OD() {
  if (pH.begin()) {
    Serial.println("Erro ao inicializar sensor de OD");
  }
}

float readENV20_pH() {
  return pH.read_ph(adc.readADC(ENV20_PH_PIN));
}

float readENV20_ORP() {
  return ORP.read_orp(adc.readADC(ENV20_ORP_PIN));
}

float readENV20_OD() {
  return DO.read_do_percentage(adc.readADC(ENV20_OD_PIN));
}


//------------------------------------- BLOCO ATLAS ENV50 -------------------------------------------
float readPH_ENV50_i2c() {
  ENV50_PH.send_read_cmd();
  //delay(1000);
  smartGPSDelay(0);
  Alarm.delay(1000);
  ENV50_PH.receive_read_cmd();
  return ENV50_PH.get_last_received_reading();
  //  receive_and_print_reading(PH);
  //  Serial.println();
}

float readORP_ENV50_i2c() {
  ENV50_ORP.send_read_cmd();
  //delay(1000);
  smartGPSDelay(0);
  Alarm.delay(1000);
  ENV50_ORP.receive_read_cmd();
  return ENV50_ORP.get_last_received_reading();
  //  receive_and_print_reading(ORP);
  //  Serial.println();
}

float readDO_ENV50_i2c() {
  ENV50_DO.send_read_cmd();
  //delay(1000);
  smartGPSDelay(0);
  Alarm.delay(1000);
  ENV50_DO.receive_read_cmd();
  return ENV50_DO.get_last_received_reading();
  //  receive_and_print_reading(DO);
  //  Serial.println();
}

float readTEMP_ENV50_i2c() {
  ENV50_TEMP.send_read_cmd();
  //delay(1000);
  smartGPSDelay(0);
  Alarm.delay(1000);
  ENV50_TEMP.receive_read_cmd();
  return ENV50_TEMP.get_last_received_reading();
  //  receive_and_print_reading(TEMP);
  //  Serial.println();
}
