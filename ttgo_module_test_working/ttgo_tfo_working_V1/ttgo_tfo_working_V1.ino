/* Set ToF to 115200 in ToF assistant
 *  After code is uploaded, Connect ToF RX to TX in TTGO and ToF TX to RX in TTGO 
 *  Update thingspeak channel details in code 
 */


// #define SIM800L_IP5306_VERSION_20190610
#define SIM800C_AXP192_VERSION_20200609 //SIM800L_AXP192_VERSION_20200327 

#define DUMP_AT_COMMANDS
#define TINY_GSM_DEBUG SerialMon

#include "utilities.h"
#define SerialMon Serial // Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialAT  Serial1 // Set serial for AT commands (to the module)

// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800          // Modem is SIM800
#define TINY_GSM_RX_BUFFER      1024   // Set RX buffer to 1Kb

#include <TinyGsmClient.h>
#include "ThingSpeak.h"

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

// I2C for SIM800 (to keep it running when powered from battery)
TwoWire I2CPower = TwoWire(0);

#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  600        /* Time ESP32 will go to sleep (in seconds) */


// Server details
const char server[] = "thingspeak.com";
const char resource[] = "http://api.thingspeak.com/update?api_key=5GRWAP501IZ7P6QI";
const int  port = 80;
unsigned long ChannelNumber = 1662945; 

// Your GPRS credentials (leave empty, if missing)
const char apn[]      = "bsnlnet"; // Your APN
const char gprsUser[] = ""; // User
const char gprsPass[] = ""; // Password
const char simPIN[]   = ""; // SIM card PIN code, if any
String apiKeyValue = "5GRWAP501IZ7P6QI";
String myReadAPIKey = "YKPV3Y4JWL30A5J7";

//ToF declaration
unsigned char TOF_data[32] = {0};   //store 2 TOF frames
unsigned char TOF_length = 16;
unsigned char TOF_header[3] {0x57,0x00,0xFF};
unsigned long TOF_system_time = 0;
unsigned long TOF_distance = 0;
unsigned char TOF_status = 0;
unsigned int TOF_signal = 0;
unsigned char TOF_check = 0;


TinyGsmClient client(modem);




void setupModem()
{
#ifdef MODEM_RST
    // Keep reset high
    pinMode(MODEM_RST, OUTPUT);
    digitalWrite(MODEM_RST, HIGH);
#endif

    pinMode(MODEM_PWRKEY, OUTPUT);
    pinMode(MODEM_POWER_ON, OUTPUT);

    // Turn on the Modem power first
    digitalWrite(MODEM_POWER_ON, HIGH);

    // Pull down PWRKEY for more than 1 second according to manual requirements
    digitalWrite(MODEM_PWRKEY, HIGH);
    delay(100);
    digitalWrite(MODEM_PWRKEY, LOW);
    delay(1000);
    digitalWrite(MODEM_PWRKEY, HIGH);

    // Initialize the indicator as an output
    pinMode(LED_GPIO, OUTPUT);
    digitalWrite(LED_GPIO, LED_OFF);
}

void turnOffNetlight()
{
    SerialMon.println("Turning off SIM800 Red LED...");
    modem.sendAT("+CNETLIGHT=0");
}

void turnOnNetlight()
{
    SerialMon.println("Turning on SIM800 Red LED...");
    modem.sendAT("+CNETLIGHT=1");
}



void setup()
{
    
    SerialMon.begin(115200); // Set console baud rate
    Serial.begin(115200);
    // Start power management
    if (setupPMU() == false) {
        Serial.println("Setting power error");
    }

    // setupSDCard();

    // Some start operations
    setupModem();
    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);// Set GSM module baud rate and UART pins
    delay(3000);
}

bool verifyCheckSum(unsigned char data[], unsigned char len){
  TOF_check = 0;

  for(int k=0;k<len-1;k++)
  {
      TOF_check += data[k];
  }

  if(TOF_check == data[len-1])
  {
      Serial.println("TOF data is ok!");
      return true;    
  }else{
      Serial.println("TOF data is error!");
      return false;  
  }
}

void loop()
{
        
    // read from serial port:
  delay(100);
  if (Serial.available()>=32) {
     for(int i=0;i<32;i++)
     {
       TOF_data[i] = Serial.read();
     }
  
    for(int j=0;j<16;j++)
    {
      if( (TOF_data[j]==TOF_header[0] && TOF_data[j+1]==TOF_header[1] && TOF_data[j+2]==TOF_header[2]) && (verifyCheckSum(&TOF_data[j],TOF_length)))
      {
        if(((TOF_data[j+12]) | (TOF_data[j+13]<<8) )==0)
        {
           Serial.println("Out of range!");
         }else{
           Serial.print("TOF id is: ");
           Serial.println(TOF_data[j+3],DEC);
    
           TOF_system_time = TOF_data[j+4] | TOF_data[j+5]<<8 | TOF_data[j+6]<<16 | TOF_data[j+7]<<24;
           Serial.print("TOF system time is: ");
           Serial.print(TOF_system_time,DEC);
           Serial.println("ms");
    
           TOF_distance = (TOF_data[j+8]) | (TOF_data[j+9]<<8) | (TOF_data[j+10]<<16);
           Serial.print("TOF distance is: ");
           Serial.print(TOF_distance,DEC);
           Serial.println("mm");
    
           TOF_status = TOF_data[j+11];
           Serial.print("TOF status is: ");
           Serial.println(TOF_status ,DEC);
    
           TOF_signal = TOF_data[j+12] | TOF_data[j+13]<<8;
           Serial.print("TOF signal is: ");
           Serial.println(TOF_signal ,DEC);
    
           Serial.println("");
         
        }
        break;
      }
    }
  }
    // Restart takes quite some time To skip it, call init() instead of restart()
    SerialMon.println("Initializing modem...");
    modem.restart();

    turnOffNetlight(); // Turn off network status lights to reduce current consumption

    // Or, use modem.init() if you don't need the complete restart
    String modemInfo = modem.getModemInfo();
    SerialMon.print("Modem: ");
    SerialMon.println(modemInfo);

    // Unlock your SIM card with a PIN if needed
    if (strlen(simPIN) && modem.getSimStatus() != 3 ) {
        modem.simUnlock(simPIN);
    }

    SerialMon.print("Waiting for network...");
    if (!modem.waitForNetwork(240000L)) {
        SerialMon.println(" fail");
        delay(10000);
        return;
    }
    SerialMon.println(" OK");
    digitalWrite(LED_GPIO, LED_ON); // When the network connection is successful, turn on the indicator

    if (modem.isNetworkConnected()) {
        SerialMon.println("Network connected");
    }

    SerialMon.print(F("Connecting to APN: "));
    SerialMon.print(apn);
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
        SerialMon.println(" fail");
        delay(10000);
        return;
    }
    SerialMon.println(" OK");

    SerialMon.print("Connecting to ");
    SerialMon.print(server);
    if (!client.connect(server, port)) {
        SerialMon.println(" fail");
        delay(10000);
        return;
    }
    SerialMon.println(" OK");
    axp.adc1Enable(AXP202_VBUS_VOL_ADC1 | AXP202_VBUS_CUR_ADC1 | AXP202_BATT_CUR_ADC1 | AXP202_BATT_VOL_ADC1, true);

    float vbus_v = axp.getVbusVoltage();
    float vbus_c = axp.getVbusCurrent();
    float batt_v = axp.getBattVoltage();
    // Make a HTTP POST request:
    SerialMon.println("Performing HTTP POST request...");
    String httpRequestData = "api_key=" + apiKeyValue + "&field1=" + String(TOF_distance)+ "&field2=" + String(vbus_c)+ "&field3=" + String(batt_v)+  "";
    //SerialMon.println(depth);
    
    client.print(String("POST ") + resource + " HTTP/1.1\r\n");
    client.print(String("Host: ") + server + "\r\n");
    client.println("Connection: close");
    client.println("Content-Type: application/x-www-form-urlencoded");
    client.print("Content-Length: ");
    client.println(httpRequestData.length());
    client.println();
    client.println(httpRequestData);

   /* SerialMon.println("Performing HTTP GET request...");
    String httpRequestData1 = "api_key=" + myReadAPIKey + String(depth)+  "";
    Serial.println(depth);
    
    client.print(String("GET ") + resource + " HTTP/1.1\r\n");
    client.print(String("Host: ") + server + "\r\n");
    client.println("Connection: close");
    client.println("Content-Type: application/x-www-form-urlencoded");
    client.print("Content-Length: ");
    client.println(httpRequestData1.length());
    client.println();
    client.println(httpRequestData1); */



    unsigned long timeout = millis();
    while (client.connected() && millis() - timeout < 10000L) {
        // Print available data
        while (client.available()) {
            char c = client.read();
            SerialMon.print(c);
            timeout = millis();
        }
    }
    SerialMon.println();

    // Shutdown
    client.stop();
    SerialMon.println(F("Server disconnected"));

    modem.gprsDisconnect();
    SerialMon.println(F("GPRS disconnected"));
    // DTR is used to wake up the sleeping Modem
    // DTR is used to wake up the sleeping Modem
    // DTR is used to wake up the sleeping Modem
#ifdef MODEM_DTR
    bool res;

    modem.sleepEnable();

    delay(100);

    res = modem.testAT(); // test modem response , res == 0 , modem is sleep
    Serial.print("SIM800 Test AT result -> ");
    Serial.println(res);

    delay(1000);

    Serial.println("Use DTR Pin Wakeup");
    pinMode(MODEM_DTR, OUTPUT);
    
    digitalWrite(MODEM_DTR, LOW); //Set DTR Pin low , wakeup modem .


    res = modem.testAT();// test modem response , res == 1 , modem is wakeup
    Serial.print("SIM800 Test AT result -> ");
    Serial.println(res);

#endif


#ifdef TEST_RING_RI_PIN
#ifdef MODEM_RI
    // Swap the audio channels
    SerialAT.print("AT+CHFA=1\r\n");
    delay(2);

    //Set ringer sound level
    SerialAT.print("AT+CRSL=100\r\n");
    delay(2);

    //Set loud speaker volume level
    SerialAT.print("AT+CLVL=100\r\n");
    delay(2);

    // Calling line identification presentation
    SerialAT.print("AT+CLIP=1\r\n");
    delay(2);

    //Set RI Pin input
    pinMode(MODEM_RI, INPUT);

    Serial.println("Wait for call in");
    //When is no calling ,RI pin is high level
    while (digitalRead(MODEM_RI)) {
        Serial.print('.');
        delay(500);
    }
    Serial.println("call in ");

    //Wait 10 seconds for the bell to ring
    delay(10000);

    //Accept call
    SerialAT.println("ATA");


    delay(10000);

    // Wait ten seconds, then hang up the call
    SerialAT.println("ATH");
#endif  //MODEM_RI
#endif  //TEST_RING_RI_PIN

    // Make the LED blink three times before going to sleep
    int i = 3;
    while (i--) {
        digitalWrite(LED_GPIO, LED_ON);
        modem.sendAT("+SPWM=0,1000,80");
        delay(500);
        digitalWrite(LED_GPIO, LED_OFF);
        modem.sendAT("+SPWM=0,1000,0");
        delay(500);
    }

    //After all off
    modem.poweroff();

    SerialMon.println(F("Poweroff"));

    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

    esp_deep_sleep_start();

    /*
    The sleep current using AXP192 power management is about 500uA,
    and the IP5306 consumes about 1mA
    */
}
