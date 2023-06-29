#define MODEM_POWER_ON 25

#define TIME_TO_SLEEP 10
#define uS_TO_S_FACTOR 1000000ULL

#define LED_BUILTIN 12

void setup() {
// put your setup code here, to run once:

esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
pinMode(MODEM_POWER_ON, OUTPUT);
digitalWrite(MODEM_POWER_ON, LOW);
esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);

pinMode(LED_BUILTIN, OUTPUT);

}

void loop() {
// put your main code here, to run repeatedly:

digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
delay(1000); // wait for a second
digitalWrite(LED_BUILTIN, LOW); // turn the LED off by making the voltage LOW
delay(1000); // wait for a second

esp_deep_sleep_start();

}
