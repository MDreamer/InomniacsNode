#include <NeoPixelBus.h>
#define colorSaturation 128
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

const uint16_t PixelCount = 6; 
const uint8_t PixelPin = 2;  // make sure to set this to the correct pin, ignored for Esp8266

const uint8_t pinsAdyNum = 5; //The number of pins used for setting address
uint8_t adyPin[pinsAdyNum] = {16,5,4,12,14};

//inits with number 99 so if the there is a faulty reading the node won't be 0
uint8_t nodeAddress = 99;

//on normal operation mode the Serial output should be disables. It slows the MCU and 
//creates juttering in the LEDs
const bool serialDebug = false;

//in udp wifi lib max buff size is 75. 25(noes)*3(RGB) is needed 
const int udp_buff_size = 72;


//const char* ssid = "Aussie Broadband 9970";
//const char* password = "mufegireso";
//const char* ssid = "Pretty Fly for a Wi-Fi";
//const char* password = "highsecurity";
const char* ssid = "NETGEAR05";
const char* password = "yellowhouse";
//const char* ssid = "Greyhaven 2";
//const char* password = "serenity is here";

WiFiUDP Udp;
unsigned int localUdpPort = 4210;  // local port to listen on

char incomingPacket[udp_buff_size];  // buffer for incoming packets
char  replyPacket[] = "Hi there! Got the message :-)";  // a reply string to send back



// Uart method is good for the Esp-01 or other pin restricted modules
// NOTE: These will ignore the PIN and use GPI02 pin with is D4 in NodeMCU LiLon V3
NeoPixelBus<NeoRgbFeature, NeoEsp8266Uart800KbpsMethod> strip(PixelCount, PixelPin);

//reads form 5 digital GPIOs and set the number of the node according to it
int getNodeAddress()
{  
  uint8_t tempNodeAdy = 0;
  
  //sets all the digital address pin allocation to INPUT
  //Reads all the pins and shift left except the last one which just read and adds it
  int i=0;
  //i= pinsAdyNum-1
  for (; i< pinsAdyNum-1; i++)
  //for (; i >= 0; i--)
  {
    if (serialDebug)
    {
      Serial.print("Reading pin..");
      Serial.print(adyPin[i]);
      Serial.print("..value:");
      Serial.println(digitalRead(adyPin[i]));
    }
    tempNodeAdy = tempNodeAdy | !digitalRead(adyPin[i]);
    tempNodeAdy = tempNodeAdy << 1;
  }
  if (serialDebug)
  {
    Serial.print("Reading pin..");
    Serial.print(adyPin[i]);
    Serial.print("..value:");
    Serial.println(digitalRead(adyPin[i]));
  }

  tempNodeAdy = tempNodeAdy | !digitalRead(adyPin[i]);
  
  return tempNodeAdy;
  //return 1;
}

void setup()
{
  if (serialDebug)
  {
    Serial.begin(115200);
    Serial.println();
  }
  
  //sets all the digital address pin allocation to INPUT
  for (int i=0; i< pinsAdyNum; i++)
  {
    pinMode(adyPin[i],INPUT);
  }
  
  nodeAddress = getNodeAddress();

  if (serialDebug)
  {
    Serial.printf("Node's address is: %d ",nodeAddress);
    Serial.printf("Connecting to %s ", ssid);
    
  }

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    if (serialDebug)
    {
      Serial.print(".");
    }
  }

  if (serialDebug)
  {
    Serial.println(" connected");
  }

  Udp.begin(localUdpPort);

  if (serialDebug)
  {
    Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);
    
    Serial.println();
    Serial.println("Initializing...");
    Serial.flush();
  }

  // this resets all the neopixels to an off state
  strip.Begin();
  strip.Show();


  if (serialDebug)
  {
    Serial.println();
    Serial.println("Running...");
  }
  delay(1000);
}


void loop()
{
  bool getData = false;
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    // receive incoming UDP packets
    if (serialDebug)
    {
      Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
    }
    int len = Udp.read(incomingPacket, udp_buff_size);
    getData = true;
    Serial.printf("Received %d data bytes in the packet. ", len);
    if (len > 0)
    {
      incomingPacket[len] = 0;
    }
    if (serialDebug)
    {
      Serial.printf("relevant data segment in the UDP packet contents: %d %d %d\n", incomingPacket[nodeAddress*3], 
                                  incomingPacket[(nodeAddress*3)+1],
                                  incomingPacket[(nodeAddress*3)+2]);
    }
  }

  if (getData)
  {
    Serial.println("Sending data to LEDs...");
    //gets from the input udp stream the relevant node segment of data out of the whole packet
    //according to the nodes address
    RgbColor targetColor = RgbColor(incomingPacket[nodeAddress*3], 
                                    incomingPacket[(nodeAddress*3)+1],
                                    incomingPacket[(nodeAddress*3)+2]);
    // RgbColor targetColor = RgbColor(255, 
    //                                 0,
    //                                 0);

    for (int i = 0; i < PixelCount; i++)
    {
      strip.SetPixelColor(i, targetColor);
    }

    strip.Show();
  }
    
  delay(10);  //up to 100Hz

}
