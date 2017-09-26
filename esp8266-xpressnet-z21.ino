/*
   Aim of this project is to create a z21 to xpressnet interface on esp8266

   Credits:
   - z21 lib created by Philipp Gahtow http://pgahtow.de/wiki/index.php?title=Z21_mobile but modifed to include support for esp8266 EEPROM
   - xpressnet lib created by Philipp Gahtow http://pgahtow.de/wiki/index.php?title=XpressNet
   - SoftwareSerial lib for esp8266 https://github.com/plerup/espsoftwareserial modified into https://github.com/kind3r/espsoftwareRS485 for 9bit support

*/
#include <SPI.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <z21.h>
#include <XpressNet.h>

// Debug variants:
// 1. Via hardware serial
#define DEBUGSERIAL Serial
// 2. Via software serial
//#include <SoftwareDEBUGSERIAL.h>
//#define DEBUGSERIAL SoftwareSerial(1,2);

// ---------------------------------------------------------------------
// WiFi settings
// ---------------------------------------------------------------------
// WiFi SSID to connect to
const char* ssid = "n3t";
// WiFi network password
const char* password = "aturiociv";
// Time to wait for WiFi to connect (number of seconds * 10)
const byte connectionTimer = 300; // default 300 = 30 seconds

// ---------------------------------------------------------------------
// esp8266 settings
// ---------------------------------------------------------------------
// esp8266 led pin (if one is available)
#define ESP8266_LED 5 // default pin 5 for sparkfun WiFi Shield

// ---------------------------------------------------------------------
// RS485 shield settings
// ---------------------------------------------------------------------
#define RS485_TX_PIN 4
#define RS485_RX_PIN 2
#define RS485_TXRX_PIN 12

// ---------------------------------------------------------------------
// XpressNet settings
// ---------------------------------------------------------------------
byte XNetAddress = 30;

// ---------------------------------------------------------------------
// z21 settings
// ---------------------------------------------------------------------
#define Z21_UDP_TX_MAX_SIZE 64
// z21 UDP port to listen on
#define z21Port 21105

// ---------------------------------------------------------------------
// general settings
// ---------------------------------------------------------------------
//Total number of storred IP address (clients)
#define maxIP 20



// IP structure
typedef struct
{
  byte IP0;
  byte IP1;
  byte IP2;
  byte IP3;
} listofIP;
// IP storage
listofIP mem[maxIP];
// number of stored IPs
byte storedIP = 0;

unsigned char packetBuffer[Z21_UDP_TX_MAX_SIZE];
WiFiUDP Udp;
z21Class z21;
XpressNetClass XpressNet;

void setup() {
  // put your setup code here, to run once:
#if defined(DEBUGSERIAL)
  DEBUGSERIAL.begin(115200);
  DEBUGSERIAL.println();
  DEBUGSERIAL.println();
  DEBUGSERIAL.println(F("z21 XpressNet passthrough starting"));
#endif

#if defined(ESP8266_LED)
  // Turn off LED
  pinMode(ESP8266_LED, OUTPUT);
  digitalWrite(ESP8266_LED, HIGH);
  boolean led = false;
#endif

  byte counter = 0;
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED && counter < connectionTimer) {
    counter++;
#if defined(ESP8266_LED)
    if (led) {
      digitalWrite(ESP8266_LED, HIGH);
      led = false;
    } else {
      digitalWrite(ESP8266_LED, LOW);
      led = true;
    }
#endif
    delay(100);
  }

  if (WiFi.status() == WL_CONNECTED) {
#if defined(ESP8266_LED)
    digitalWrite(ESP8266_LED, LOW);
#endif
#if defined(DEBUGSERIAL)
    DEBUGSERIAL.println(F("WiFi connected"));
    DEBUGSERIAL.print(F("Local IP: ")); DEBUGSERIAL.println(WiFi.localIP());
#endif
  } else {
#if defined(ESP8266_LED)
    digitalWrite(ESP8266_LED, HIGH);
#endif
#if defined(DEBUGSERIAL)
    DEBUGSERIAL.println(F("Error: cannot connect to WiFi"));
#endif
    return;
  }

  XpressNet.start(XNetAddress, RS485_TXRX_PIN);

  Udp.begin(z21Port);

  z21.setPower(csNormal);
}

void loop() {
  // Receive XpressNet packets
  XpressNet.receive();
  
  // Receive UDP packets and send them to z21 library
  if (Udp.parsePacket() > 0) { //packetSize
    Udp.read(packetBuffer, Z21_UDP_TX_MAX_SIZE); // read the packet into packetBufffer
    IPAddress remote = Udp.remoteIP();
    z21.receive(addIP(remote[0], remote[1], remote[2], remote[3]), packetBuffer);
  }

}

// Store IP in list and return it's index
byte addIP (byte ip0, byte ip1, byte ip2, byte ip3) {
  //suche ob IP schon vorhanden?
  for (byte i = 0; i < storedIP; i++) {
    if (mem[i].IP0 == ip0 && mem[i].IP1 == ip1 && mem[i].IP2 == ip2 && mem[i].IP3 == ip3)
      return i + 1;
  }
  if (storedIP >= maxIP)
    return 0;
  mem[storedIP].IP0 = ip0;
  mem[storedIP].IP1 = ip1;
  mem[storedIP].IP2 = ip2;
  mem[storedIP].IP3 = ip3;
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
}

//--------------------------------------------------------------------------------------------
void notifyz21EthSend(uint8_t client, uint8_t *data)
{
  if (client == 0) { //all stored
    for (byte i = 0; i < storedIP; i++) {
      IPAddress ip(mem[i].IP0, mem[i].IP1, mem[i].IP2, mem[i].IP3);
      Udp.beginPacket(ip, Udp.remotePort());    //Broadcast
      Udp.write(data, data[0]);
      Udp.endPacket();
    }
  }
  else {
    IPAddress ip(mem[client - 1].IP0, mem[client - 1].IP1, mem[client - 1].IP2, mem[client - 1].IP3);
    Udp.beginPacket(ip, Udp.remotePort());    //no Broadcast
    Udp.write(data, data[0]);
    Udp.endPacket();
  }
}

//--------------------------------------------------------------------------------------------
void notifyz21S88Data()
{
  //z21.setS88Data (datasend);  //Send back state of S88 Feedback
#if defined(DEBUGSERIAL)
  DEBUGSERIAL.println(F("notifyz21S88Data"));
#endif
}

//--------------------------------------------------------------------------------------------
void notifyz21getLocoState(uint16_t Adr, bool bc)
{
#if defined(DEBUGSERIAL)
  DEBUGSERIAL.printf("notifyz21getLocoState %d\r\n", Adr);
#endif
  //void setLocoStateFull (int Adr, byte steps, byte speed, byte F0, byte F1, byte F2, byte F3, bool bc);
}

void notifyz21LocoFkt(uint16_t Adr, uint8_t type, uint8_t fkt)
{
#if defined(DEBUGSERIAL)
  DEBUGSERIAL.printf("notifyz21LocoFkt %d %d %d\r\n", Adr, type, fkt);
#endif
}

//--------------------------------------------------------------------------------------------
void notifyz21LocoSpeed(uint16_t Adr, uint8_t speed, uint8_t steps)
{
#if defined(DEBUGSERIAL)
  DEBUGSERIAL.printf("notifyz21LocoSpeed %d %d %d\r\n", Adr, speed, steps);
#endif
XpressNet.setLocoDrive((Adr >> 8) & 0x3F, (Adr & 0xFF), steps, speed);
}

//--------------------------------------------------------------------------------------------
void notifyz21Accessory(uint16_t Adr, bool state, bool active)
{
#if defined(DEBUGSERIAL)
  DEBUGSERIAL.printf("notifyz21Accessory %d\r\n", Adr);
#endif
}

//--------------------------------------------------------------------------------------------
uint8_t notifyz21AccessoryInfo(uint16_t Adr)
//return state of the Address (left/right = true/false)
{
#if defined(DEBUGSERIAL)
  DEBUGSERIAL.printf("notifyz21AccessoryInfo %d\r\n", Adr);
#endif
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
void notifyz21LNSendPacket(uint8_t *data, uint8_t length)
{
#if defined(DEBUGSERIAL)
  DEBUGSERIAL.println("notifyz21LNSendPacket");
#endif
}

//--------------------------------------------------------------------------------------------
void notifyz21CVREAD(uint8_t cvAdrMSB, uint8_t cvAdrLSB)
{
#if defined(DEBUGSERIAL)
  DEBUGSERIAL.printf("notifyz21CVREAD %d %d\r\n", cvAdrMSB, cvAdrLSB);
#endif
}

//--------------------------------------------------------------------------------------------
void notifyz21CVWRITE(uint8_t cvAdrMSB, uint8_t cvAdrLSB, uint8_t value)
{
#if defined(DEBUGSERIAL)
  DEBUGSERIAL.printf("notifyz21CVWRITE %d %d %d\r\n", cvAdrMSB, cvAdrLSB, value);
#endif
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
void notifyXNetStatus(uint8_t LedState )
{
//#if defined(DEBUGSERIAL)
//  DEBUGSERIAL.printf("notifyXNetStatus %d\r\n", LedState);
//#endif
}

//--------------------------------------------------------------------------------------------
void notifyXNetVer(uint8_t V, uint8_t ID )
{
#if defined(DEBUGSERIAL)
  DEBUGSERIAL.printf("notifyXNetVer %d %d\r\n", V, ID);
#endif
}

//--------------------------------------------------------------------------------------------
void notifyXNetPower(uint8_t State )
{
#if defined(DEBUGSERIAL)
  DEBUGSERIAL.printf("notifyXNetPower %d\r\n", State);
#endif
}

//--------------------------------------------------------------------------------------------
void notifyLokFunc(uint8_t Adr_High, uint8_t Adr_Low,  uint8_t F2, uint8_t F3 )
{
#if defined(DEBUGSERIAL)
  DEBUGSERIAL.printf("notifyLokFunc %d %d %d %d\r\n", Adr_High, Adr_Low, F2, F3);
#endif
}

//--------------------------------------------------------------------------------------------
void notifyLokAll(uint8_t Adr_High, uint8_t Adr_Low, boolean Busy, uint8_t Steps, uint8_t Speed, uint8_t Direction, uint8_t F0, uint8_t F1, uint8_t F2, uint8_t F3, boolean Req )
{
#if defined(DEBUGSERIAL)
  DEBUGSERIAL.printf("notifyLokAll %d %d %d %d %d %d %d %d %d\r\n", Adr_High, Adr_Low, Steps, Speed, Direction, F0, F1, F2, F3);
#endif
}

//--------------------------------------------------------------------------------------------
void notifyCVInfo(uint8_t State )
{
#if defined(DEBUGSERIAL)
  DEBUGSERIAL.printf("notifyCVInfo %d\r\n", State);
#endif
}

//--------------------------------------------------------------------------------------------
void notifyCVResult(uint8_t cvAdr, uint8_t cvData )
{
#if defined(DEBUGSERIAL)
  DEBUGSERIAL.printf("notifyCVResult %d %d\r\n", cvAdr, cvData);
#endif
}

//--------------------------------------------------------------------------------------------
void notifyTrnt(uint8_t Adr_High, uint8_t Adr_Low, uint8_t Pos)
{
#if defined(DEBUGSERIAL)
  DEBUGSERIAL.printf("notifyTrnt %d %d %d\r\n", Adr_High, Adr_Low, Pos);
#endif
}
