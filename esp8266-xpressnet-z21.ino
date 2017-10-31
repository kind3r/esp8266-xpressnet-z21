/*
   Aim of this project is to create a z21 command station on esp8266

   Credits:
   - z21 lib created by Philipp Gahtow http://pgahtow.de/wiki/index.php?title=Z21_mobile but modifed to include support for esp8266 EEPROM
   - xpressnet lib created by Philipp Gahtow http://pgahtow.de/wiki/index.php?title=XpressNet
   - SoftwareSerial lib for esp8266 https://github.com/plerup/espsoftwareserial modified into https://github.com/kind3r/espsoftwareRS485 for 9bit support

*/

// RS485 interface
// #define XNetRS485_TX 4   // RS485 TX pin
// #define XNetRS485_RX 2   // RS485 RX pin
// #define RS485_TXRX_PIN 5 // RS485 TX control pin (can be ommited for auto TX control)

// S88n interface
// #define S88DataPin 15 //S88 Data IN
// #define S88ClkPin 13 //S88 Clock
// #define S88PSPin 12 //S88 PS/LOAD
// #define S88ResetPin 14 //S88 Reset

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
byte XNetAddress = 30; // The XpressNet address of this device
#include <esp8266-XpressNet.h>

// Z21 library and settings
#define Z21_UDP_TX_MAX_SIZE 128 // max received UDP packet size
#define z21Port 21105          // z21 UDP port to listen on
#include <z21.h>

// S88 settings
#include <esp8266-S88n.h>
byte S88Modules = 2; // number of S88 8 port modules

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
S88nClass S88;

//--------------------------------------------------------------------------------------------
// Setup
//--------------------------------------------------------------------------------------------
void setup()
{
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

  // Start S88 with 2 modules
  if(S88.start(S88Modules)) {
#if defined(DEBUGSERIAL)
    DEBUGSERIAL.println(F("Started S88n"));
#endif    
  }

  // Start z21 emulation
  Udp.begin(z21Port);
  z21.setPower(csNormal);
}

//--------------------------------------------------------------------------------------------
// Main loop
//--------------------------------------------------------------------------------------------
void loop()
{
  // Receive XpressNet packets
  XpressNet.receive();
  yield();
  // Receive S88 data
  S88.checkS88Data();
  yield();
  // Receive UDP packets and send them to z21 library
  if (Udp.parsePacket() > 0)
  {                                              //packetSize
    Udp.read(packetBuffer, Z21_UDP_TX_MAX_SIZE); // read the packet into packetBufffer
    IPAddress remote = Udp.remoteIP();
    z21.receive(addIP(remote[0], remote[1], remote[2], remote[3], Udp.remotePort()), packetBuffer);
  }
  yield();
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
  data[0] = 0x00;                  //MainCurrent mA
  data[1] = 0x00;                  //MainCurrent mA
  data[2] = 0x00;                  //ProgCurrent mA
  data[3] = 0x00;                  //ProgCurrent mA
  data[4] = 0x00;                  //FilteredMainCurrent
  data[5] = 0x00;                  //FilteredMainCurrent
  data[6] = 0x00;                  //Temperature
  data[7] = 0x20;                  //Temperature
  data[8] = 0x0F;                  //SupplyVoltage
  data[9] = 0x00;                  //SupplyVoltage
  data[10] = 0x00;                 //VCCVoltage
  data[11] = 0x03;                 //VCCVoltage
  data[12] = XpressNet.getPower(); //CentralState
  data[13] = 0x00;                 //CentralStateEx
  data[14] = 0x00;                 //reserved
  data[15] = 0x00;                 //reserved
  notifyz21EthSend(client, data);
}

//--------------------------------------------------------------------------------------------
void notifyz21S88Data(uint8_t gIndex)
{
#if defined(DEBUGSERIAL)
  DEBUGSERIAL.printf("notifyz21S88Data %d\r\n", gIndex);
#endif
  byte *data;
  data = S88.getData();
  z21.setS88Data(data, S88Modules);
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
  if (steps == 128)
    xSteps = 3;
  if (steps == 28)
    xSteps = 2;
  XpressNet.setLocoDrive(highByte(Adr), lowByte(Adr), xSteps, speed);
}

//--------------------------------------------------------------------------------------------
void notifyz21Accessory(uint16_t Adr, bool state, bool active)
{
#if defined(DEBUGSERIAL)
  DEBUGSERIAL.printf("notifyz21Accessory %d %d %d\r\n", Adr, state, active);
#endif
  XpressNet.setTrntPos(highByte(Adr), lowByte(Adr), ((active ? 1 : 0) << 3) + (state ? 1 : 0));
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
  XpressNet.readCVMode(cvAdrLSB + 1);
}

//--------------------------------------------------------------------------------------------
void notifyz21CVWRITE(uint8_t cvAdrMSB, uint8_t cvAdrLSB, uint8_t value)
{
#if defined(DEBUGSERIAL)
  DEBUGSERIAL.printf("notifyz21CVWRITE %d %d %d\r\n", cvAdrMSB, cvAdrLSB, value);
#endif
  XpressNet.writeCVMode(cvAdrLSB + 1, value);
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
  if (State == 0x01 || State == 0x02)
  {
    z21.setCVNack();
  }
  else
  {
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
// S88n library callback functions
//--------------------------------------------------------------------------------------------
void notifyS88Data(byte *S88data)
{
#if defined(DEBUGSERIAL)
  DEBUGSERIAL.printf("notifyS88Data\r\n");
#endif  
  z21.setS88Data(S88data, S88Modules);
}
