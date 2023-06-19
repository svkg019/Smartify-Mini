//This code is written by Souvik Ghosh
//Smartify Mini_v1_ESPRainMaker
//Electronify

#include "RMaker.h"
#include "WiFi.h"
#include "WiFiProv.h"
#define DEFAULT_POWER_MODE false

const char *service_name = "Electronify"; //name of node in BLE
const char *pop = "SouvikDebmitra@19"; //password

// GPIO for push button
static uint8_t gpio_reset = 0;
// GPIO for switch
static uint8_t gpio_switch1 = 32; //IO3
// GPIOs for relay
static uint8_t gpio_relay1 = 27; //IO18
/* Variables for reading pin status*/
bool switch_state_ch1 = false;

struct LightSwitch {
    const uint8_t pin;
    bool pressed;
};

// Define the light switches for channel 1 and 2
LightSwitch switch_ch1 = {gpio_switch1, false};

//The framework provides some standard device types like switch, lightbulb, fan, temperature sensor.
static LightBulb my_switch1("Switch_ch1", &gpio_relay1);

void sysProvEvent(arduino_event_t *sys_event)
{
    switch (sys_event->event_id) {      
        case ARDUINO_EVENT_PROV_START:
#if CONFIG_IDF_TARGET_ESP32
        Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on BLE\n", service_name, pop);
        printQR(service_name, pop, "ble");
#else
        Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on SoftAP\n", service_name, pop);
        printQR(service_name, pop, "softap");
#endif        
        break;
        case ARDUINO_EVENT_WIFI_STA_CONNECTED:
        Serial.printf("\nConnected to Wi-Fi!\n");
        break;
    }
}

void write_callback(Device *device, Param *param, const param_val_t val, void *priv_data, write_ctx_t *ctx)
{
    const char *device_name = device->getDeviceName();
    const char *param_name = param->getParamName();

    if(strcmp(device_name, "Switch_ch1") == 0) {
      
      Serial.printf("Lightbulb = %s\n", val.val.b? "true" : "false");
      
      if(strcmp(param_name, "Power") == 0) {
          Serial.printf("Received value = %s for %s - %s\n", val.val.b? "true" : "false", device_name, param_name);
        switch_state_ch1 = val.val.b;
        (switch_state_ch1 == false) ? digitalWrite(gpio_relay1, LOW) : digitalWrite(gpio_relay1, HIGH);
        param->updateAndReport(val);
      }
      
    } 
 
}

void ARDUINO_ISR_ATTR isr(void* arg) {
    LightSwitch* s = static_cast<LightSwitch*>(arg);
    s->pressed = true;
}

void setup()
{
    uint32_t chipId = 0;
    
    Serial.begin(115200);

    // Configure the input GPIOs
    pinMode(gpio_reset, INPUT);
    pinMode(switch_ch1.pin, INPUT_PULLUP);
    attachInterruptArg(switch_ch1.pin, isr, &switch_ch1, CHANGE);
    
    // Set the Relays GPIOs as output mode
    pinMode(gpio_relay1, OUTPUT);
   

    // Write to the GPIOs the default state on booting
    digitalWrite(gpio_relay1, DEFAULT_POWER_MODE);

    Node my_node;    
    my_node = RMaker.initNode("Smartify Mini v1");

    //Standard switch device
    my_switch1.addCb(write_callback);
  
    //Add switch device to the node   
    my_node.addDevice(my_switch1);

    //This is optional 
    RMaker.enableOTA(OTA_USING_PARAMS);
    //If you want to enable scheduling, set time zone for your region using setTimeZone(). 
    //The list of available values are provided here https://rainmaker.espressif.com/docs/time-service.html
    // RMaker.setTimeZone("Asia/Shanghai");
    // Alternatively, enable the Timezone service and let the phone apps set the appropriate timezone
    RMaker.enableTZService();
    RMaker.enableSchedule();

    //Service Name
    for(int i=0; i<17; i=i+8) {
      chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
    }

    Serial.printf("\nChip ID:  %d Service Name: %s\n", chipId, service_name);

    Serial.printf("\nStarting ESP-RainMaker\n");
    RMaker.start();

    WiFi.onEvent(sysProvEvent);
#if CONFIG_IDF_TARGET_ESP32
    WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_FREE_BTDM, WIFI_PROV_SECURITY_1, pop, service_name);
#else
    WiFiProv.beginProvision(WIFI_PROV_SCHEME_SOFTAP, WIFI_PROV_SCHEME_HANDLER_NONE, WIFI_PROV_SECURITY_1, pop, service_name);
#endif
}

void loop()
{

    if (switch_ch1.pressed) {
        Serial.printf("Switch 1 has been changed\n");
        switch_ch1.pressed = false;
        // Toggle switch 1 device state
        switch_state_ch1 = !switch_state_ch1;
        Serial.printf("Toggle State to %s.\n", switch_state_ch1 ? "true" : "false");
        my_switch1.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, switch_state_ch1);
        (switch_state_ch1 == false) ? digitalWrite(gpio_relay1, LOW) : digitalWrite(gpio_relay1, HIGH);
    } 
    // Read GPIO0 (external button to reset device
    if(digitalRead(gpio_reset) == LOW) { //Push button pressed
        Serial.printf("Reset Button Pressed!\n");
        // Key debounce handling
        delay(100);
        int startTime = millis();
        while(digitalRead(gpio_reset) == LOW) delay(50);
        int endTime = millis();

        if ((endTime - startTime) > 10000) {
          // If key pressed for more than 10secs, reset all
          Serial.printf("Reset to factory.\n");
          RMakerFactoryReset(2);
        } else if ((endTime - startTime) > 3000) {
          Serial.printf("Reset Wi-Fi.\n");
          // If key pressed for more than 3secs, but less than 10, reset Wi-Fi
          RMakerWiFiReset(2);
        }
    }
    delay(100);
}
