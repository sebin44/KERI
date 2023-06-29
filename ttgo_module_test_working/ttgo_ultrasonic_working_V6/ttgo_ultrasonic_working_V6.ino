#define SIM800C_AXP192_VERSION_20200609
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
#define TIME_TO_SLEEP  120        /* Time ESP32 will go to sleep (in seconds) */


// Server details
const char server[] = "thingspeak.com";
const char resource[] = "http://api.thingspeak.com/update?api_key=2TA8W2GNYCRCYH74";
const int  port = 80;
unsigned long ChannelNumber = 1662945; 

// Your GPRS credentials (leave empty, if missing)
const char apn[]      = "bsnlnet"; // Your APN
const char gprsUser[] = ""; // User
const char gprsPass[] = ""; // Password
const char simPIN[]   = ""; // SIM card PIN code, if any
String apiKeyValue = "2TA8W2GNYCRCYH74";
String myReadAPIKey = "YKPV3Y4JWL30A5J7";

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

// Ultrasonic
int trig= 0; //trig pin
int echo= 2; //echo pin
float distance = 0;
float duration = 0;
float depth = 0;

void ultraSonic()
{
    SerialMon.println("Initializing distance measurement");
    digitalWrite(trig, LOW);
    delayMicroseconds(100);
    digitalWrite(trig,HIGH);
    delayMicroseconds(100);
    digitalWrite(trig,LOW);
    duration=pulseIn(echo,HIGH);
     if (duration<28000){
        distance=duration*(0.0340/2.0);
        Serial.println("distance:");
        Serial.print(distance);
        Serial.println("cm");
        delay(3000);
        Serial.println("duration");
        Serial.println(duration);
        delay(3000);
        }
   else{Serial.print("no result");}
}

void setup()
{
    
    SerialMon.begin(115200); // Set console baud rate

    delay(10);
      //Ultrasonic sensor
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);

    // Start power management
    if (setupPMU() == false) {
        Serial.println("Setting power error");
    }

    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    
    // Some start operations
    setupModem();
    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);// Set GSM module baud rate and UART pins
    delay(3000);
}

void loop()
{
        
    ultraSonic();
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
    String httpRequestData = "api_key=" + apiKeyValue + "&field1=" + String(distance)+ "&field2=" + String(vbus_c)+ "&field3=" + String(batt_v)+  "";
    SerialMon.println(depth);
    
    client.print(String("POST ") + resource + " HTTP/1.1\r\n");
    client.print(String("Host: ") + server + "\r\n");
    client.println("Connection: close");
    client.println("Content-Type: application/x-www-form-urlencoded");
    client.print("Content-Length: ");
    client.println(httpRequestData.length());
    client.println();
    client.println(httpRequestData);

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

    SerialMon.println(F("Poweroff"));

    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

    esp_deep_sleep_start();

}
