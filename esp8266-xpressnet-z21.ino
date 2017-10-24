/*
   Aim of this project is to create a z21 to XpressNet interface on esp8266

   Credits:
   - z21 lib created by Philipp Gahtow http://pgahtow.de/wiki/index.php?title=Z21_mobile but modifed to include support for esp8266 EEPROM
   - xpressnet lib created by Philipp Gahtow http://pgahtow.de/wiki/index.php?title=XpressNet
   - SoftwareSerial lib for esp8266 https://github.com/plerup/espsoftwareserial modified into https://github.com/kind3r/espsoftwareRS485 for 9bit support

*/
#include <SPI.h>

// WiFi ESP library
#include <ESP8266WiFi.h>

// WiFiManager auto-configuration library
#include <DNSServer.h>        //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h> //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>      //https://github.com/tzapu/WiFiManager WiFi Configuration Magic

// WiFi UDP library
#include <WiFiUdp.h>

// XpressNet library settings
// RS485 interface
#define XNetRS485_TX 4    // RS485 TX pin
#define XNetRS485_RX 2    // RS485 RX pin
#define RS485_TXRX_PIN 5  // RS485 TX control pin (can be ommited for auto TX control)
// XpressNet settings
byte XNetAddress = 30; // The XpressNet address of this device
#include <esp8266-XpressNet.h>

// Z21 library and settings
#define Z21_UDP_TX_MAX_SIZE 64 // max received UDP packet size
#define z21Port 21105          // z21 UDP port to listen on
#include <z21.h>

// S88 settings
#define S88DataPin 15  //S88 Data IN pin
#define S88ClkPin 13   //S88 Clock pin
#define S88PSPin 12    //S88 PS/LOAD pin
#define S88ResetPin 14 //S88 Reset pin
byte S88Module = 2;    // Number of S88 modules. Each module has 8 inputs so a 16 inputs board is composed of 2 modules

extern "C" {
#include "user_interface.h"
}
os_timer_t s88Timer;
uint8_t S88RCount = 0;  //Lesezähler 0-39 Zyklen
uint8_t S88RMCount = 0; //Lesezähler Modul-Pin
char S88sendon = '0';   //Bit Änderung
byte data[62];          //Zustandsspeicher für 62x 8fach Modul

// Debug variants:
// 1. Via hardware serial
#define DEBUGSERIAL Serial
// 2. Via software serial
//#include <SoftwareDEBUGSERIAL.h>
//#define DEBUGSERIAL SoftwareSerial(1,2);

// IP settings
#define maxIP 20 //Total number of storred IP address (clients)
typedef struct   // Structure to hold IP's and ports
{
  byte IP0;
  byte IP1;
  byte IP2;
  byte IP3;
  uint16_t port;
} listofIP;
listofIP mem[maxIP]; // IP storage
byte storedIP = 0;   // number of currently stored IPs

// Init local variables
unsigned char packetBuffer[Z21_UDP_TX_MAX_SIZE];
WiFiUDP Udp;
z21Class z21;
XpressNetClass XpressNet;

void setup()
{
// put your setup code here, to run once:
#if defined(DEBUGSERIAL)
  DEBUGSERIAL.begin(115200);
  DEBUGSERIAL.println();
  DEBUGSERIAL.println();
  DEBUGSERIAL.println(F("z21 XpressNet emulation starting"));
#endif

  // Start WiFi
  WiFiManager wifiManager;
  wifiManager.autoConnect("Z21-Config");

#if defined(DEBUGSERIAL)
  DEBUGSERIAL.println(F("WiFi connected"));
  DEBUGSERIAL.print(F("Local IP: "));
  DEBUGSERIAL.println(WiFi.localIP());
#endif

// Start XpressNet
#if defined(RS485_TXRX_PIN)
  XpressNet.start(XNetAddress, RS485_TXRX_PIN);
#else
  XpressNet.start(XNetAddress);
#endif

  // Start S88
  SetupS88();

  // Start z21 emulation
  Udp.begin(z21Port);
  z21.setPower(csNormal);
}

void loop()
{
  // Receive XpressNet packets
  XpressNet.receive();

  // Receive S88 data
  notifyS88Data();

  // Receive UDP packets and send them to z21 library
  if (Udp.parsePacket() > 0)
  {                                              //packetSize
    Udp.read(packetBuffer, Z21_UDP_TX_MAX_SIZE); // read the packet into packetBufffer
    IPAddress remote = Udp.remoteIP();
    z21.receive(addIP(remote[0], remote[1], remote[2], remote[3], Udp.remotePort()), packetBuffer);
  }
}

// Store IP in list and return it's index
byte addIP(byte ip0, byte ip1, byte ip2, byte ip3, uint16_t port)
{
  //suche ob IP schon vorhanden?
  for (byte i = 0; i < storedIP; i++)
  {
    if (mem[i].IP0 == ip0 && mem[i].IP1 == ip1 && mem[i].IP2 == ip2 && mem[i].IP3 == ip3)
      return i + 1;
  }
  if (storedIP >= maxIP)
    return 0;
  mem[storedIP].IP0 = ip0;
  mem[storedIP].IP1 = ip1;
  mem[storedIP].IP2 = ip2;
  mem[storedIP].IP3 = ip3;
  mem[storedIP].port = port;
  storedIP++;
  return storedIP;
}

//--------------------------------------------------------------------------------------------
// z21 library callback functions
//--------------------------------------------------------------------------------------------
void notifyz21RailPower(uint8_t State)
{
#if defined(DEBUGSERIAL)
  DEBUGSERIAL.print("Power: ");
  DEBUGSERIAL.println(State, HEX);
#endif
  XpressNet.setPower(State);
}

//--------------------------------------------------------------------------------------------
void notifyz21EthSend(uint8_t client, uint8_t *data)
{
  if (client == 0)
  { //all stored
    for (byte i = 0; i < storedIP; i++)
    {
      IPAddress ip(mem[i].IP0, mem[i].IP1, mem[i].IP2, mem[i].IP3);
      Udp.beginPacket(ip, mem[i].port); //Broadcast
      Udp.write(data, data[0]);
      Udp.endPacket();
    }
  }
  else
  {
    IPAddress ip(mem[client - 1].IP0, mem[client - 1].IP1, mem[client - 1].IP2, mem[client - 1].IP3);
    Udp.beginPacket(ip, mem[client - 1].port); //no Broadcast
    Udp.write(data, data[0]);
    Udp.endPacket();
  }
}

void notifyz21getSystemInfo(uint8_t client)
{
  byte data[16];
  data[0] = 0x00;  //MainCurrent mA
  data[1] = 0x00;  //MainCurrent mA
  data[2] = 0x00;  //ProgCurrent mA
  data[3] = 0x00;  //ProgCurrent mA        
  data[4] = 0x00;  //FilteredMainCurrent
  data[5] = 0x00;  //FilteredMainCurrent
  data[6] = 0x00;  //Temperature
  data[7] = 0x20;  //Temperature
  data[8] = 0x0F;  //SupplyVoltage
  data[9] = 0x00;  //SupplyVoltage
  data[10] = 0x00;  //VCCVoltage
  data[11] = 0x03;  //VCCVoltage
  data[12] = XpressNet.getPower();  //CentralState
  data[13] = 0x00;  //CentralStateEx
  data[14] = 0x00;  //reserved
  data[15] = 0x00;  //reserved
  notifyz21EthSend(client, data);
}

//--------------------------------------------------------------------------------------------
void notifyz21S88Data(uint8_t gIndex)
{
//z21.setS88Data (datasend);  //Send back state of S88 Feedback
#if defined(DEBUGSERIAL)
  DEBUGSERIAL.printf("notifyz21S88Data %d\r\n", gIndex);
#endif
  // must cache the last sent S88 data and resend it as someone requested it
}

//--------------------------------------------------------------------------------------------
void notifyz21getLocoState(uint16_t Adr, bool bc)
{
#if defined(DEBUGSERIAL)
  DEBUGSERIAL.printf("notifyz21getLocoState %d\r\n", Adr);
#endif
  XpressNet.getLocoInfo(highByte(Adr), lowByte(Adr));
  XpressNet.getLocoFunc(highByte(Adr), lowByte(Adr)); 
  // XpressNet.getLocoStateFull(highByte(Adr), lowByte(Adr), bc);
}

void notifyz21LocoFkt(uint16_t Adr, uint8_t type, uint8_t fkt)
{
#if defined(DEBUGSERIAL)
  DEBUGSERIAL.printf("notifyz21LocoFkt %d %d %d\r\n", Adr, type, fkt);
#endif
  XpressNet.setLocoFunc(highByte(Adr), lowByte(Adr), type, fkt);
}

//--------------------------------------------------------------------------------------------
void notifyz21LocoSpeed(uint16_t Adr, uint8_t speed, uint8_t steps)
{
#if defined(DEBUGSERIAL)
  DEBUGSERIAL.printf("notifyz21LocoSpeed %d %d %d\r\n", Adr, speed, steps);
#endif
  // XpressNet.setLocoDrive((Adr >> 8) & 0x3F, (Adr & 0xFF), steps, speed);
  uint8_t xSteps = 1;
  if(steps == 128) xSteps = 3;
  if(steps == 28) xSteps = 2;
  XpressNet.setLocoDrive(highByte(Adr), lowByte(Adr), xSteps, speed);
}

//--------------------------------------------------------------------------------------------
void notifyz21Accessory(uint16_t Adr, bool state, bool active)
{
#if defined(DEBUGSERIAL)
  DEBUGSERIAL.printf("notifyz21Accessory %d %d %d\r\n", Adr, state, active);
#endif
  XpressNet.setTrntPos(highByte(Adr), lowByte(Adr), ((active?1:0) << 3) + (state?1:0));
}

//--------------------------------------------------------------------------------------------
uint8_t notifyz21AccessoryInfo(uint16_t Adr)
//return state of the Address (left/right = true/false)
{
#if defined(DEBUGSERIAL)
  DEBUGSERIAL.printf("notifyz21AccessoryInfo %d\r\n", Adr);
#endif
  XpressNet.getTrntInfo(highByte(Adr), lowByte(Adr));
  // we must wait for XpressNet to return this information as z21 lib requires it to send a response
  // Or should we cache this data ?
  return false;
}

//--------------------------------------------------------------------------------------------
uint8_t notifyz21LNdispatch(uint8_t Adr2, uint8_t Adr)
//return the Slot that was dispatched, 0xFF at error!
{
#if defined(DEBUGSERIAL)
  DEBUGSERIAL.printf("notifyz21LNdispatch %d %d\r\n", Adr2, Adr);
#endif
  return 0xFF;
}

//--------------------------------------------------------------------------------------------
// void notifyz21LNSendPacket(uint8_t *data, uint8_t length)
// {
// #if defined(DEBUGSERIAL)
//   DEBUGSERIAL.println("notifyz21LNSendPacket");
// #endif
// }

//--------------------------------------------------------------------------------------------
void notifyz21CVREAD(uint8_t cvAdrMSB, uint8_t cvAdrLSB)
{
#if defined(DEBUGSERIAL)
  DEBUGSERIAL.printf("notifyz21CVREAD %d %d\r\n", cvAdrMSB, cvAdrLSB);
#endif
  XpressNet.readCVMode(cvAdrLSB+1);
}

//--------------------------------------------------------------------------------------------
void notifyz21CVWRITE(uint8_t cvAdrMSB, uint8_t cvAdrLSB, uint8_t value)
{
#if defined(DEBUGSERIAL)
  DEBUGSERIAL.printf("notifyz21CVWRITE %d %d %d\r\n", cvAdrMSB, cvAdrLSB, value);
#endif
  XpressNet.writeCVMode(cvAdrLSB+1, value);
}

//--------------------------------------------------------------------------------------------
void notifyz21CVPOMWRITEBYTE(uint16_t Adr, uint16_t cvAdr, uint8_t value)
{
#if defined(DEBUGSERIAL)
  DEBUGSERIAL.printf("notifyz21CVPOMWRITEBYTE %d %d %d\r\n", Adr, cvAdr, value);
#endif

}

//--------------------------------------------------------------------------------------------
// xpressnet library callback functions
//--------------------------------------------------------------------------------------------
void notifyXNetStatus(uint8_t LedState)
{
  //#if defined(DEBUGSERIAL)
  //  DEBUGSERIAL.printf("notifyXNetStatus %d\r\n", LedState);
  //#endif
}

//--------------------------------------------------------------------------------------------
void notifyXNetVer(uint8_t V, uint8_t ID)
{
#if defined(DEBUGSERIAL)
  DEBUGSERIAL.printf("notifyXNetVer %d %d\r\n", V, ID);
#endif
}

//--------------------------------------------------------------------------------------------
void notifyXNetPower(uint8_t State)
{
#if defined(DEBUGSERIAL)
  DEBUGSERIAL.printf("notifyXNetPower %d\r\n", State);
#endif
  z21.setPower(State);
}

//--------------------------------------------------------------------------------------------
void notifyLokFunc(uint8_t Adr_High, uint8_t Adr_Low, uint8_t F2, uint8_t F3)
{
#if defined(DEBUGSERIAL)
  DEBUGSERIAL.printf("notifyLokFunc %d %d %d %d\r\n", Adr_High, Adr_Low, F2, F3);
#endif
}

//--------------------------------------------------------------------------------------------
void notifyLokAll(uint8_t Adr_High, uint8_t Adr_Low, boolean Busy, uint8_t Steps, uint8_t Speed, uint8_t Direction, uint8_t F0, uint8_t F1, uint8_t F2, uint8_t F3, boolean bc)
{
#if defined(DEBUGSERIAL)
  DEBUGSERIAL.printf("notifyLokAll %d %d %d %d %d %d %d %d %d\r\n", Adr_High, Adr_Low, Steps, Speed, Direction, F0, F1, F2, F3);
#endif
  z21.setLocoStateFull(word(Adr_High, Adr_Low), Steps, Speed, F0, F1, F2, F3, bc);
}

//--------------------------------------------------------------------------------------------
void notifyCVInfo(uint8_t State)
{
#if defined(DEBUGSERIAL)
  DEBUGSERIAL.printf("notifyCVInfo %d\r\n", State);
#endif
  if (State == 0x01 || State == 0x02) {
    z21.setCVNack();
  } else {
    // z21.setCVNackSC();
  }
}

//--------------------------------------------------------------------------------------------
void notifyCVResult(uint8_t cvAdr, uint8_t cvData)
{
#if defined(DEBUGSERIAL)
  DEBUGSERIAL.printf("notifyCVResult %d %d\r\n", cvAdr, cvData);
#endif
  z21.setCVReturn(cvAdr, cvData);
}

//--------------------------------------------------------------------------------------------
void notifyTrnt(uint8_t Adr_High, uint8_t Adr_Low, uint8_t Pos)
{
#if defined(DEBUGSERIAL)
  DEBUGSERIAL.printf("notifyTrnt %d %d %d\r\n", Adr_High, Adr_Low, Pos);
#endif
  z21.setTrntInfo(word(Adr_High, Adr_Low), Pos - 1);
}

//--------------------------------------------------------------------------------------------
// S88 functions
//--------------------------------------------------------------------------------------------
void SetupS88()
{
  if (S88Module > 62 || S88Module == 0)
  { //S88 off!
    S88Module = 0;
    return;
  }

  os_timer_setfn(&s88Timer, S88Timer, NULL);
  os_timer_arm(&s88Timer, 1, true);
}

//--------------------------------------------------------------
//S88 Timer ISR Routine
void S88Timer(void *pArg)
{
  if (S88RCount == 3) //Load/PS Leitung auf 1, darauf folgt ein Schiebetakt nach 10 ticks!
    digitalWrite(S88PSPin, HIGH);
  if (S88RCount == 4)              //Schiebetakt nach 5 ticks und S88Module > 0
    digitalWrite(S88ClkPin, HIGH); //1. Impuls
  if (S88RCount == 5)              //Read Data IN 1. Bit und S88Module > 0
    S88readData();                 //LOW-Flanke während Load/PS Schiebetakt, dann liegen die Daten an
  if (S88RCount == 9)              //Reset-Plus, löscht die den Paralleleingängen vorgeschaltetetn Latches
    digitalWrite(S88ResetPin, HIGH);
  if (S88RCount == 10) //Ende Resetimpuls
    digitalWrite(S88ResetPin, LOW);
  if (S88RCount == 11) //Ende PS Phase
    digitalWrite(S88PSPin, LOW);
  if (S88RCount >= 12 && S88RCount < 10 + (S88Module * 8) * 2)
  {                         //Auslesen mit weiteren Schiebetakt der Latches links
    if (S88RCount % 2 == 0) //wechselnder Taktimpuls/Schiebetakt
      digitalWrite(S88ClkPin, HIGH);
    else
      S88readData(); //Read Data IN 2. bis (Module*8) Bit
  }
  S88RCount++; //Zähler für Durchläufe/Takt
  if (S88RCount >= 10 + (S88Module * 8) * 2)
  {                 //Alle Module ausgelesen?
    S88RCount = 0;  //setzte Zähler zurück
    S88RMCount = 0; //beginne beim ersten Modul von neuem
    //init der Grundpegel
    digitalWrite(S88PSPin, LOW);
    digitalWrite(S88ClkPin, LOW);
    digitalWrite(S88ResetPin, LOW);
    if (S88sendon == 's') //Änderung erkannt
      S88sendon = 'i';    //senden
  }
  //Capture the current timer value. This is how much error we have due to interrupt latency and the work in this function
  //  TCNT2 = TCNT2 + TIMER_Time;    //Reload the timer and correct for latency.
}

// //--------------------------------------------------------------
// //Einlesen des Daten-Bit und Vergleich mit vorherigem Durchlauf
void S88readData()
{
  digitalWrite(S88ClkPin, LOW); //LOW-Flanke, dann liegen die Daten an
  byte Modul = S88RMCount / 8;
  byte Port = S88RMCount % 8;
  byte getData = digitalRead(S88DataPin); //Bit einlesen
  if (bitRead(data[Modul], Port) != getData)
  {                                       //Zustandsänderung Prüfen?
    bitWrite(data[Modul], Port, getData); //Bitzustand Speichern
    S88sendon = 's';                      //Änderung vorgenommen. (SET)
  }
  S88RMCount++;
}

// //--------------------------------------------------------------------------------------------
void notifyS88Data()
{
  if (S88sendon == 'i' || S88sendon == 'm')
  {
    byte MAdr = 1;     //Rückmeldemodul
    byte datasend[11]; //Array Gruppenindex (1 Byte) & Rückmelder-Status (10 Byte)
    datasend[0] = 0;   //Gruppenindex für Adressen 1 bis 10
    for (byte m = 0; m < S88Module; m++)
    { //Durchlaufe alle aktiven Module im Speicher
      datasend[MAdr] = data[m];
      MAdr++; //Nächste Modul in der Gruppe
      if (MAdr >= 11)
      {           //10 Module à 8 Ports eingelesen
        MAdr = 1; //beginne von vorn
                  //  EthSend (0x0F, 0x80, datasend, false, 0x02); //RMBUS_DATACHANED
        z21.setS88Data(datasend);
        datasend[0]++; //Gruppenindex erhöhen
      }
    }
    if (MAdr < 11)
    { //noch unbenutzte Module in der Gruppe vorhanden? Diese 0x00 setzten und dann Melden!
      while (MAdr < 11)
      {
        datasend[MAdr] = 0x00; //letzten leeren Befüllen
        MAdr++;                //Nächste Modul in der Gruppe
      }
      //  EthSend (0x0F, 0x80, datasend, false, 0x02); //RMBUS_DATACHANED
      z21.setS88Data(datasend);
    }
    S88sendon = '0'; //Speicher Rücksetzten
  }
}