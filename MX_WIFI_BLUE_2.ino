#include <SPI.h>
#include <WiFi.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

bool deviceConnected = false;  //controle de dispositivo conectado

const char* ssid = "MX007";
const char* password = "72507766CC";
//const char* ssid = "Hacker I";
//const char* password = "#MaxLider16052710#";

WiFiServer server(80);

//SERIAL RELAY
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
Adafruit_MCP23X17 mcp;

//DEFINE TIMER
volatile int Time1;
volatile int Time2;
volatile int Time3;
volatile int Time4;
volatile int Time5;

hw_timer_t* timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  Time1++;
  Time2++;
  Time3++;
  Time4++;
  Time5++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

//MEMORIA
#include <Preferences.h>

Preferences MX;

////////////////////////////////////////////////////////////////////

//PINOS BOTOES
const int LigaPin = 13;    // número do pino
const int DesligaPin = 4;  //16 número do pino
int EstLigaPin = 0;
int EstDesligaPin = 0;
int GuarEstLigaPin = LOW;

//VARIAVEIS SENSOR PISTAO
int SensorP = 35;
int SensorV;
int EstSensor;

//VARIAVEIS LEITURA ESP32
String SiloESP;
String GavetaAESP;
String EspVibGavESP;
String GavetaBESP;
String RepGavetaESP;
String AgitadorESP;
String VibradorESP;
String PrensaESP;
String ExtrusoraESP;
String EsperaESP;
String CicloTESP;
String CicloPESP;
String CicloTESPE;
String CicloPESPE;
String CicloPESPZ;

//VARIAVEIS LEITURA WIFI
String SiloWIFI;
String GavetaAWIFI;
String EspVibGavWIFI;
String GavetaBWIFI;
String RepGavetaWIFI;
String AgitadorWIFI;
String VibradorWIFI;
String PrensaWIFI;
String ExtrusoraWIFI;
String EsperaWIFI;

//VARIAVEIS CONVERÇÃO LEITURA
int Silo;
int GavetaA;
int EspVibGav;
int GavetaB;
int RepGaveta;
int Agitador;
int Vibrador;
int Prensa;
int Extrusora;
int Espera;
int CicloT;
int CicloP;

//VARIAVEIS BOTOES BLE
int BotZerar;

//VARIAVEIS DO PROGAMA
int P1 = 1;
int P2 = 1;
int P3 = 1;
int P4 = 1;
int P5 = 1;

//VARIAVEIS EXECUCAO
int RepGavetaE;
int PrensaB;
int PrensaE;
int VoltaPrensa;
int ExtrusoraE;
int TempoVoltaSilo = 10;
int TempoVoltaGaveta = 20;
int AV1;
int AV2;

//VARIAVEIS ENVIO
String Envio;
int LerEstagio = 0;
int EnvioZerar;

void setup() {
  Serial.begin(115200);

  pinMode(LigaPin, INPUT);
  pinMode(DesligaPin, INPUT);

  MX.begin("Dados", false);

  //INICIA RELE SERIAL
  mcp.begin_I2C();
  mcp.pinMode(10, OUTPUT);  //SILO
  mcp.digitalWrite(10, HIGH);
  mcp.pinMode(11, OUTPUT);  //GAVETA
  mcp.digitalWrite(11, HIGH);
  mcp.pinMode(12, OUTPUT);  //VIBRADOR
  mcp.digitalWrite(12, HIGH);
  mcp.pinMode(13, OUTPUT);  //AGITADOR
  mcp.digitalWrite(13, HIGH);
  mcp.pinMode(14, OUTPUT);  //PRENSA
  mcp.digitalWrite(14, HIGH);
  mcp.pinMode(15, OUTPUT);  //EXTRUSORA
  mcp.digitalWrite(15, HIGH);

  //INICIA TIMER
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 100000, true);  //1000000 = 1seg
  timerAlarmEnable(timer);

  ///*
  // Connect to Wi-Fi network with SSID and password
  Serial.print("Setting AP (Access Point)…");
  // Remove the password parameter, if you want the AP (Access Point) to be open
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  //*/

  /*
  WiFi.begin(ssid, password);              //
  while (WiFi.status() != WL_CONNECTED) {  //
    delay(250);                            //
    Serial.print(".");                     //
  }
  Serial.print("IP address:\t");   //
  Serial.println(WiFi.localIP());  //
  */

  server.begin();
  initBT();
  ReadOff();
}

void initBT() {
  if (!SerialBT.begin("MX-BLUE007")) {
    Serial.println("An error occurred initializing Bluetooth");
    ESP.restart();
  } else {
    Serial.println("Bluetooth initialized");
  }

  SerialBT.register_callback(btCallback);
  Serial.println("The device started, now you can pair it with bluetooth");
}

void btCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t* param) {
  if (event == ESP_SPP_CLOSE_EVT) {
    Serial.println("Client disconnected");
    deviceConnected = false;
  }
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    Serial.println("Client Connected!");
    deviceConnected = true;
  } else if (event == ESP_SPP_DATA_IND_EVT) {
    //Serial.printf("ESP_SPP_DATA_IND_EVT len=%d, handle=%d\n\n", param->data_ind.len, param->data_ind.handle);
    String received = bluetoothReadLine();
    Serial.println(received);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    //ZERAR
    if (received.indexOf("Z1") != -1) {
      EnvioZerar = 1;
    }
  }
}

String bluetoothReadLine() {
  String text_received = "";
  while (SerialBT.available()) {
    byte r = SerialBT.read();
    if (r != 13 && r != 10 && char(r) != '\0')
      text_received = text_received + char(r);
  }
  return text_received;
}

void writeSerialBT(String respuesta) {
  SerialBT.println(respuesta);
  SerialBT.flush();
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    Serial.println("New Client.");
    String currentLine = "";
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        if (c == '\n') {
          if (currentLine.length() == 0) {
            client.println("HTTP/1.1 200 OK");
            client.println("Content-Type: text/html");
            client.println();
            if (LerEstagio == 1) {
              client.print(Envio);
              LerEstagio = 0;
              Envio = "";
            }
            client.println();
            break;
          } else {
            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////
        //Silo
        if (currentLine.indexOf("SILO") != -1) {
          SiloWIFI = "";
          for (int i = 9; i < (currentLine.length() - 9); i++) {
            SiloWIFI = SiloWIFI + currentLine[i];
          }
          Silo = SiloWIFI.toInt();
          Serial.print("Silo ");
          Serial.println(Silo);
          Save();
        }
        //Gaveta A
        if (currentLine.indexOf("GAVA") != -1) {
          GavetaAWIFI = "";
          for (int i = 9; i < (currentLine.length() - 9); i++) {
            GavetaAWIFI = GavetaAWIFI + currentLine[i];
          }
          GavetaA = GavetaAWIFI.toInt();
          Serial.print("Gaveta A ");
          Serial.println(GavetaA);
          Save();
        }
        //Gaveta B
        if (currentLine.indexOf("GAVB") != -1) {
          GavetaBWIFI = "";
          for (int i = 9; i < (currentLine.length() - 9); i++) {
            GavetaBWIFI = GavetaBWIFI + currentLine[i];
          }
          GavetaB = GavetaBWIFI.toInt();
          Serial.print("Gaveta B ");
          Serial.println(GavetaB);
          Save();
        }
        //Espera Vibrador Da Gaveta
        if (currentLine.indexOf("ESVG") != -1) {
          EspVibGavWIFI = "";
          for (int i = 9; i < (currentLine.length() - 9); i++) {
            EspVibGavWIFI = EspVibGavWIFI + currentLine[i];
          }
          EspVibGav = EspVibGavWIFI.toInt();
          Serial.print("Esp Vib Gav ");
          Serial.println(EspVibGav);
          Save();
        }
        //Repetição Gaveta
        if (currentLine.indexOf("REGA") != -1) {
          RepGavetaWIFI = "";
          for (int i = 9; i < (currentLine.length() - 9); i++) {
            RepGavetaWIFI = RepGavetaWIFI + currentLine[i];
          }
          RepGaveta = RepGavetaWIFI.toInt();
          Serial.print("Rep Gaveta ");
          Serial.println(RepGaveta);
          Save();
        }
        //Agitador
        if (currentLine.indexOf("AGIT") != -1) {
          AgitadorWIFI = "";
          for (int i = 9; i < (currentLine.length() - 9); i++) {
            AgitadorWIFI = AgitadorWIFI + currentLine[i];
          }
          Agitador = AgitadorWIFI.toInt();
          Serial.print("Agitador ");
          Serial.println(Agitador);
          Save();
        }
        //Vibrador
        if (currentLine.indexOf("VIBR") != -1) {
          VibradorWIFI = "";
          for (int i = 9; i < (currentLine.length() - 9); i++) {
            VibradorWIFI = VibradorWIFI + currentLine[i];
          }
          Vibrador = VibradorWIFI.toInt();
          Serial.print("Vibrador ");
          Serial.println(Vibrador);
          Save();
        }
        //Prensa
        if (currentLine.indexOf("PREN") != -1) {
          PrensaWIFI = "";
          for (int i = 9; i < (currentLine.length() - 9); i++) {
            PrensaWIFI = PrensaWIFI + currentLine[i];
          }
          Prensa = PrensaWIFI.toInt();
          Serial.print("Prensa ");
          Serial.println(Prensa);
          Save();
        }
        //Extrusora
        if (currentLine.indexOf("EXTR") != -1) {
          ExtrusoraWIFI = "";
          for (int i = 9; i < (currentLine.length() - 9); i++) {
            ExtrusoraWIFI = ExtrusoraWIFI + currentLine[i];
          }
          Extrusora = ExtrusoraWIFI.toInt();
          Serial.print("Extrusora ");
          Serial.println(Extrusora);
          Save();
        }
        //Espera
        if (currentLine.indexOf("ESPE") != -1) {
          EsperaWIFI = "";
          for (int i = 9; i < (currentLine.length() - 9); i++) {
            EsperaWIFI = EsperaWIFI + currentLine[i];
          }
          Espera = EsperaWIFI.toInt();
          Serial.print("Espera ");
          Serial.println(Espera);
          Save();
        }
        //Envio
        if (currentLine.endsWith("GET /ENVIO")) {
          Read();
        }
      }
    }
    client.stop();
    Serial.println("Client Disconnected.");
  }
  /////////////////////////////////////////////////////////////////////////////////
  EstLigaPin = digitalRead(LigaPin);
  EstDesligaPin = digitalRead(DesligaPin);

  if (EstLigaPin == HIGH) {
    if (Time4 > 5) {
      GuarEstLigaPin = HIGH;
    }
  } else {
    Time4 = 0;
  }
  if (EstDesligaPin == HIGH) {
    if (Time5 > 5) {
      GuarEstLigaPin = LOW;
    }
  } else {
    Time5 = 0;
  }
  //////////////////////////////////////////////
  SensorV = analogRead(SensorP);
  //char SensorLido[14];
  //SensorV.toCharArray(SensorLido, 14);
  //writeSerialBT("A1J0K1");


  if (SensorV >= 3000) {
    EstSensor = 1;
  }
  if (SensorV < 3000) {
    EstSensor = 1;  //0
  }
  //////////////////////////////////////////////////////////////
  if (GuarEstLigaPin == HIGH) {
    if (P1 == 16) {
      writeSerialBT("T" + CicloTESP + "P" + CicloPESP + "F");
      P1 = 17;
    }
    if (P1 == 1) {
      RepGavetaE = RepGaveta;
      PrensaE = Prensa + 20;
      ExtrusoraE = Extrusora + 20;

      Time1 = 0;

      P1 = 2;
      P2 = 0;
      P3 = 0;
      P4 = 0;
      P5 = 0;

      AV1 = 1;
      AV2 = 1;
    }
    //////////////////////////////////////////////////////////////
    if (Silo >= Time1 && P1 == 2) {
      //LIGA O SILO
      mcp.digitalWrite(10, LOW);

      writeSerialBT("A1J0K1");
      //Serial.println("ON SILO");

      P1 = 3;
    }
    if (Silo < Time1 && P1 == 3) {
      //DESLIGA O SILO
      mcp.digitalWrite(10, HIGH);

      writeSerialBT("A0J0");
      //Serial.println("OFF SILO");

      Time1 = 0;
      P1 = 4;
    }
    if (TempoVoltaSilo < Time1 && P1 == 4) {
      Time1 = 0;
      Time2 = 0;
      Time3 = 0;
      P1 = 5;
      P2 = 1;
      P3 = 2;
    }
    //////////////////////////////////////////////////////////////
    if (GavetaA >= Time1 && P1 == 5) {
      //LIGA A GAVETA
      mcp.digitalWrite(11, LOW);

      writeSerialBT("B1F1");
      //Serial.println("ON GAVETA");

      P1 = 6;
    }
    if (GavetaA < Time1 && P1 == 6) {
      Time1 = 0;
      P1 = 7;
    }
    ///////////////////////////////////
    if (EspVibGav < Time2 && P2 == 1) {
      //LIGA O VIBRADOR
      mcp.digitalWrite(12, LOW);

      writeSerialBT("C1");
      //Serial.println("ON VIBRADOR GAVETA");

      P2 = 0;
    }
    ///////////////////////////////////
    if (Agitador < Time3 && P3 == 3) {
      AV2 = AV2 * -1;
      Time3 = 0;
      P3 = 2;
    }

    if (Agitador >= Time3 && P3 == 2) {
      if (AV2 == 1) {
        //LIGA O AGITADOR
        mcp.digitalWrite(13, LOW);
        //Serial.println("ON AGITADOR");
      }
      if (AV2 == -1) {
        //DESLIGA O AGITADOR
        mcp.digitalWrite(13, HIGH);
        //Serial.println("OFF AGITADOR");
      }
      P3 = 3;
    }
    ///////////////////////////////////
    if (GavetaB < Time1 && P1 == 8 && RepGavetaE >= 1) {
      if (AV1 == -1) {
        RepGavetaE = RepGavetaE - 1;
      }
      AV1 = AV1 * -1;
      Time1 = 0;
      P1 = 7;
    }

    if (GavetaB >= Time1 && P1 == 7 && RepGavetaE >= 1) {
      writeSerialBT("B0D1E1");

      if (AV1 == -1) {
        //LIGA A GAVETA
        mcp.digitalWrite(11, LOW);
        //Serial.println("ON GAVETA 2");
      }
      if (AV1 == 1) {
        //DESLIGA A GAVETA
        mcp.digitalWrite(11, HIGH);
        //Serial.println("OFF GAVETA 2");
      }
      P1 = 8;
    }

    if (P1 == 7 && RepGavetaE <= 0) {
      //DESLIGA A GAVETA
      mcp.digitalWrite(11, HIGH);

      //DESLIGA O VIBRADOR
      mcp.digitalWrite(12, HIGH);
      P2 = 0;

      //DESLIGA O AGITADOR
      mcp.digitalWrite(13, HIGH);
      P3 = 0;

      writeSerialBT("B0C0D0E0F0");
      //Serial.println("OFF GAVETA 2");
      //Serial.println("OFF VIBRADOR GAVETA");
      //Serial.println("OFF AGITADOR");

      P1 = 9;
      Time1 = 0;
    }
    ///////////////////////////////////
    if (TempoVoltaGaveta < Time1 && P1 == 9) {
      Time1 = 0;
      P1 = 10;
    }
    ///////////////////////////////////
    if (PrensaE >= Time1 && P1 == 10 && EstSensor == 1) {
      //LIGA A PRENSA
      mcp.digitalWrite(14, LOW);

      writeSerialBT("H1K1");
      //Serial.println("ON PRENSA");

      P1 = 11;
      P4 = 1;
    }
    if (Vibrador >= Time1 && P4 == 1 && EstSensor == 1) {
      //LIGA O VIBRADOR
      mcp.digitalWrite(12, LOW);

      writeSerialBT("G1");
      //Serial.println("ON VIBRADOR");

      P4 = 2;
    }
    if (Vibrador < Time1 && P4 == 2) {
      //DESLIGA O VIBRADOR
      mcp.digitalWrite(12, HIGH);

      writeSerialBT("G0");
      //Serial.println("OFF VIBRADOR");

      P4 = 3;
    }
    if (Prensa < Time1 && P1 == 11) {
      Time2 = 0;
      P1 = 12;
      P5 = 1;
    }
    if (PrensaE < Time1 && P5 == 1) {
      //DESLIGA A PRENSA
      mcp.digitalWrite(14, HIGH);

      writeSerialBT("H0");
      //Serial.println("OFF PRENSA");

      P5 = 2;
    }
    //////////////////////////////////////////////
    if (ExtrusoraE >= Time2 && P1 == 12 && EstSensor == 1) {
      //LIGA A EXTRUSORA
      mcp.digitalWrite(15, LOW);

      writeSerialBT("I1");
      //Serial.println("ON EXTRUSORA");

      P1 = 13;
    }
    if (ExtrusoraE < Time2 && P1 == 13) {
      //DESLIGA A EXTRUSORA
      mcp.digitalWrite(15, HIGH);

      if (P4 == 3) {
        P1 = 14;
        Time1 = 0;
      }
    }
    //////////////////////////////////////////////
    if (Espera >= Time1 && P1 == 14) {
      writeSerialBT("A0B0C0D0E0F0G0H0I0J1K1");
      //Serial.println("ON ESPERA");

      P1 = 15;
    }
    if (Espera < Time1 && P1 == 15) {
      writeSerialBT("J0K1");
      //Serial.println("OFF ESPERA");

      CicloTESP = MX.getString("CICLOT", "0");
      CicloTESP = CicloTESP.toInt() + 1;
      MX.putString("CICLOT", CicloTESP);
      Serial.print("CICLO T: ");
      Serial.println(CicloTESP);

      CicloPESP = MX.getString("CICLOP", "0");
      CicloPESP = CicloPESP.toInt() + 1;
      MX.putString("CICLOP", CicloPESP);
      Serial.print("CICLO P: ");
      Serial.println(CicloPESP);

      P1 = 16;
    }
    if (P1 == 17) {
      P1 = 1;
    }
  }
  ////////////////////////////////////////////////
  if (GuarEstLigaPin == LOW) {
    writeSerialBT("K0");

    P1 = 1;
    P2 = 0;
    P3 = 0;
    P4 = 0;
    AV1 = 1;
    AV2 = 1;
    //DESLIGA TUDO
    mcp.digitalWrite(10, HIGH);
    mcp.digitalWrite(11, HIGH);
    mcp.digitalWrite(12, HIGH);
    mcp.digitalWrite(13, HIGH);
    mcp.digitalWrite(14, HIGH);
    mcp.digitalWrite(15, HIGH);
  }
  if (EnvioZerar == 1) {
    MX.putString("CICLOP", "0");
    CicloPESP = MX.getString("CICLOP", "0");

    writeSerialBT("T" + CicloTESP + "P" + CicloPESP + "F");

    EnvioZerar = 0;
  }
}
///////////////////////////////////////////////////////////////////
void Save() {
  if (SiloESP != SiloWIFI) {
    SiloESP = SiloWIFI;
    MX.putString("Silo", SiloWIFI);
  }
  if (GavetaAESP != GavetaAWIFI) {
    GavetaAESP = GavetaAWIFI;
    MX.putString("GavetaA", GavetaAESP);
  }
  if (EspVibGavESP != EspVibGavWIFI) {
    EspVibGavESP = EspVibGavWIFI;
    MX.putString("EspVibGav", EspVibGavESP);
  }
  if (GavetaBESP != GavetaBWIFI) {
    GavetaBESP = GavetaBWIFI;
    MX.putString("GavetaB", GavetaBESP);
  }
  if (RepGavetaESP != RepGavetaWIFI) {
    RepGavetaESP = RepGavetaWIFI;
    MX.putString("RepGaveta", RepGavetaESP);
  }
  if (AgitadorESP != AgitadorWIFI) {
    AgitadorESP = AgitadorWIFI;
    MX.putString("Agitador", AgitadorESP);
  }
  if (VibradorESP != VibradorWIFI) {
    VibradorESP = VibradorWIFI;
    MX.putString("Vibrador", VibradorESP);
  }
  if (PrensaESP != PrensaWIFI) {
    PrensaESP = PrensaWIFI;
    MX.putString("Prensa", PrensaESP);
  }
  if (ExtrusoraESP != ExtrusoraWIFI) {
    ExtrusoraESP = ExtrusoraWIFI;
    MX.putString("Extrusora", ExtrusoraESP);
  }
  if (EsperaESP != EsperaWIFI) {
    EsperaESP = EsperaWIFI;
    MX.putString("Espera", EsperaESP);
  }
}
///////////////////////////////////////////////////////////////////
void ReadOff() {
  SiloESP = MX.getString("Silo", "0");
  Silo = SiloESP.toInt();
  SiloWIFI = SiloESP;
  GavetaAESP = MX.getString("GavetaA", "0");
  GavetaA = GavetaAESP.toInt();
  GavetaAWIFI = GavetaAESP;
  EspVibGavESP = MX.getString("EspVibGav", "0");
  EspVibGav = EspVibGavESP.toInt();
  EspVibGavWIFI = EspVibGavESP;
  GavetaBESP = MX.getString("GavetaB", "0");
  GavetaB = GavetaBESP.toInt();
  GavetaBWIFI = GavetaBESP;
  RepGavetaESP = MX.getString("RepGaveta", "0");
  RepGaveta = RepGavetaESP.toInt();
  RepGavetaWIFI = RepGavetaESP;
  AgitadorESP = MX.getString("Agitador", "0");
  Agitador = AgitadorESP.toInt();
  AgitadorWIFI = AgitadorESP;
  VibradorESP = MX.getString("Vibrador", "0");
  Vibrador = VibradorESP.toInt();
  VibradorWIFI = VibradorESP;
  PrensaESP = MX.getString("Prensa", "0");
  Prensa = PrensaESP.toInt();
  PrensaWIFI = PrensaESP;
  ExtrusoraESP = MX.getString("Extrusora", "0");
  Extrusora = ExtrusoraESP.toInt();
  ExtrusoraWIFI = ExtrusoraESP;
  EsperaESP = MX.getString("Espera", "0");
  Espera = EsperaESP.toInt();
  EsperaWIFI = EsperaESP;
  CicloTESP = MX.getString("CICLOT", "0");
  CicloPESP = MX.getString("CICLOP", "0");
}
///////////////////////////////////////////////////////////////////
void Read() {
  SiloESP = MX.getString("Silo", "0");
  Silo = SiloESP.toInt();
  SiloWIFI = SiloESP;
  GavetaAESP = MX.getString("GavetaA", "0");
  GavetaA = GavetaAESP.toInt();
  GavetaAWIFI = GavetaAESP;
  EspVibGavESP = MX.getString("EspVibGav", "0");
  EspVibGav = EspVibGavESP.toInt();
  EspVibGavWIFI = EspVibGavESP;
  GavetaBESP = MX.getString("GavetaB", "0");
  GavetaB = GavetaBESP.toInt();
  GavetaBWIFI = GavetaBESP;
  RepGavetaESP = MX.getString("RepGaveta", "0");
  RepGaveta = RepGavetaESP.toInt();
  RepGavetaWIFI = RepGavetaESP;
  AgitadorESP = MX.getString("Agitador", "0");
  Agitador = AgitadorESP.toInt();
  AgitadorWIFI = AgitadorESP;
  VibradorESP = MX.getString("Vibrador", "0");
  Vibrador = VibradorESP.toInt();
  VibradorWIFI = VibradorESP;
  PrensaESP = MX.getString("Prensa", "0");
  Prensa = PrensaESP.toInt();
  PrensaWIFI = PrensaESP;
  ExtrusoraESP = MX.getString("Extrusora", "0");
  Extrusora = ExtrusoraESP.toInt();
  ExtrusoraWIFI = ExtrusoraESP;
  EsperaESP = MX.getString("Espera", "0");
  Espera = EsperaESP.toInt();
  EsperaWIFI = EsperaESP;
  CicloTESP = MX.getString("CICLOT", "0");
  CicloPESP = MX.getString("CICLOP", "0");

  Envio = "SI" + SiloESP;
  Envio = Envio + "GA" + GavetaAESP;
  Envio = Envio + "EV" + EspVibGavESP;
  Envio = Envio + "GB" + GavetaBESP;
  Envio = Envio + "RE" + RepGavetaESP;
  Envio = Envio + "AG" + AgitadorESP;
  Envio = Envio + "VI" + VibradorESP;
  Envio = Envio + "PR" + PrensaESP;
  Envio = Envio + "EX" + ExtrusoraESP;
  Envio = Envio + "ES" + EsperaESP;
  Envio = Envio + "CT" + CicloTESP;
  Envio = Envio + "CP" + CicloPESP + "CF";

  //Serial.println(Envio);

  LerEstagio = 1;
}