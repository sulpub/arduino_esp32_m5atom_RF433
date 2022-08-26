/*
  Utilisation d'une radio 433MHz pour allumer et eteindre une
  prise radio standard phenix.
  Utilisation d'un ESP32 pour le controle de l'entrée RF 433MHz
  d'une telecommande phenix YC-4000S

  https://github.com/sui77/rc-switch/

  https://gladysassistant.com/fr/blog/gerer-les-appareils-electrique/

  https://docs.m5stack.com/en/core/atom_lite

  M5stack ATOM LITE
  grove port
    - 5V / GND alimentation pile de la telecommande PHENIX YC-4000S
    - pin26 (soudure fil sur la pin 17 du composant HX2262 (acces pin
      entrée module radio)

  Format trame JSON:
    code : sur 5 caractéres 0 ou F (exemple "00FF0") en fonction des
           dip switch sur les recepteurs
    channel 0 : A
    channel 1 : B
    channel 2 : C
    channel 3 : D
    state 0 : OFF
    state 1 : ON

  Exemple de trame JSON a envoyé en MQTT avec le topic "action_telecommande_433" :
  bouton A ON  : {"code":"F0F00","channel":0,"state":1}
  bouton A OFF : {"code":"F0F00","channel":0,"state":0}
  bouton B ON  : {"code":"F0F00","channel":1,"state":1}
  bouton B OFF : {"code":"F0F00","channel":1,"state":0}
*/



/*
  .__              .__            .___
  |__| ____   ____ |  |  __ __  __| _/____
  |  |/    \_/ ___\|  | |  |  \/ __ |/ __ \
  |  |   |  \  \___|  |_|  |  / /_/ \  ___/
  |__|___|  /\___  >____/____/\____ |\___  >
          \/     \/                \/    \/
*/
#include <stdio.h>
#include "M5Atom.h"
#include <Wire.h>
#include "esp_deep_sleep.h"

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>

#include <WiFi.h>
#include <WiFiMulti.h>

#include <PubSubClient.h>
#include <RCSwitch.h>

RCSwitch mySwitch = RCSwitch();

#include <Arduino_JSON.h>


/*
    ________.__        ___.          .__                                              __
   /  _____/|  |   ____\_ |__ _____  |  |   ___________ ____________    _____   _____/  |_  ___________  ______
  /   \  ___|  |  /  _ \| __ \\__  \ |  |   \____ \__  \\_  __ \__  \  /     \_/ __ \   __\/ __ \_  __ \/  ___/
  \    \_\  \  |_(  <_> ) \_\ \/ __ \|  |__ |  |_> > __ \|  | \// __ \|  Y Y  \  ___/|  | \  ___/|  | \/\___ \
   \______  /____/\____/|___  (____  /____/ |   __(____  /__|  (____  /__|_|  /\___  >__|  \___  >__|  /____  >
          \/                \/     \/       |__|       \/           \/      \/     \/          \/           \/
*********************************************************************************************************************
*/
//Global parameters

const char* ssid1             = "ssid wifi - a remplir";
const char* password1         = "password wifi - a remplir";

//const char* ssid2           = "";
//const char* password2       = "";
//
//const char* ssid3           = "";
//const char* password3       = "";
//
//const char* ssid4           = "";
//const char* password4       = "";
//
//const char* ssid5           = "";
//const char* password5       = "";
//
//const char* ssid6           = "";
//const char* password6       = "";

const char* mqtt_server       = "ip serveur MQTT - a remplir";

// value for MQTT broker.
String clientIdMqtt           = "telecommande_433";
String subscribe_str          = "action_telecommande_433";
String clientLoginMqtt        = "login MQTT - a remplir";
String clientPassMqtt         = "password MQTT - a remplir";

//End global parameter
//******************************************************************************************************************



/*
      .___      _____.__
    __| _/_____/ ____\__| ____   ____
   / __ |/ __ \   __\|  |/    \_/ __ \
  / /_/ \  ___/|  |  |  |   |  \  ___/
  \____ |\___  >__|  |__|___|  /\___  >
       \/    \/              \/     \/
*/

//define button
#define BUTTON_PIN 39

//define capteur PIR
#define PIR_PIN 32

// Number de test max
#define TEST_WIFI      1  //20
#define TEST_MQTT      1  //5



/*
                      .__      ___.   .__
  ___  _______ _______|__|____ \_ |__ |  |   ____   ______
  \  \/ /\__  \\_  __ \  \__  \ | __ \|  | _/ __ \ /  ___/
   \   /  / __ \|  | \/  |/ __ \| \_\ \  |_\  ___/ \___ \
    \_/  (____  /__|  |__(____  /___  /____/\___  >____  >
              \/              \/    \/          \/     \/
*/
//------------------------VARIABLES------------------------------
int intcounter = 0;
int i = 0;
int acq_restart = 1;

uint8_t DisBuff[2 + 5 * 5 * 3];

unsigned long ulong_time_now = 0;
unsigned long ulong_time_meas_cycle = 0;
unsigned long ulong_diff_time_meas_cycle = 0;
unsigned long ulong_time_send_meas_cycle = 0;

RTC_DATA_ATTR int bootCount = 0;

int count_lost_mqtt = 0;
int count_lost_wifi = 0;

const int wdtTimeout = 10000;  //20000ms=20s time in ms to trigger the watchdog
hw_timer_t *timer = NULL;


//new variable wifi + MQTT
IPAddress Ip_Node_adress;
WiFiMulti wifiMulti;

String string_value               = "   ";
int value                         = 0;
int intTempCpu                    = -20;

// buffer envoi et reception message
char msg[150];
char buf[10];
char buffer_tmp[250];
char buffer_cmd[250];

int nb_test_ctl_wifi              = 0;
int nb_test_ctl_mqtt              = 0;

//variables pour la reception JSON MQTT
String stringCode = "";
String stringChannel = "";
String stringState = "";
int intChannel = 0;
int intState = 0;
int intDemandeCmdRf433 = 0;
int intErrorCmdReceive = 0;

//pir sensor variable
int intPirSensor = 0;

/*
    _________       __                  __      __.______________.___
   /   _____/ _____/  |_ __ ________   /  \    /  \   \_   _____/|   |
   \_____  \_/ __ \   __\  |  \____ \  \   \/\/   /   ||    __)  |   |
   /        \  ___/|  | |  |  /  |_> >  \        /|   ||     \   |   |
  /_______  /\___  >__| |____/|   __/    \__/\  / |___|\___  /   |___|
          \/     \/           |__|            \/           \/
*/
void setup_wifi() {

  //affichage rouge no wifi
  setBuff(0x44, 0x00, 0x00);
  M5.dis.displaybuff(DisBuff);

  //réarmement du watchdog (echoue si > 10secondes)
  timerWrite(timer, 0); //reset timer (feed watchdog)

  //delai restart
  delay(random(500)); // Delay for a period of time (in milliseconds).

  //réarmement du watchdog (echoue si > 10secondes)
  timerWrite(timer, 0); //reset timer (feed watchdog)

  wifiMulti.addAP(ssid1, password1);
  //  wifiMulti.addAP(ssid2, password2);
  //  wifiMulti.addAP(ssid3, password3);
  //  wifiMulti.addAP(ssid4, password4);
  //  wifiMulti.addAP(ssid5, password5);

  Serial.println("Connecting Wifi...");
  if (wifiMulti.run() == WL_CONNECTED) {
    Serial.println("");
    Serial.print("WiFi connected: ");
    Serial.println(WiFi.SSID());

    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    //affichage bleu wifi ok MQTT wait
    setBuff(0x00, 0x00, 0x44);
    M5.dis.displaybuff(DisBuff);
    delay(1000);
    count_lost_wifi++;
  }

  Ip_Node_adress = WiFi.localIP();

}

WiFiClient espClient;
PubSubClient client(mqtt_server, 1883, callback_mqtt, espClient);
void send_status_mqtt(void);



/*
                .__  .__ ___.                  __        _____   ______________________________
    ____ _____  |  | |  |\_ |__ _____    ____ |  | __   /     \  \_____  \__    ___/\__    ___/
  _/ ___\\__  \ |  | |  | | __ \\__  \ _/ ___\|  |/ /  /  \ /  \  /  / \  \|    |     |    |
  \  \___ / __ \|  |_|  |_| \_\ \/ __ \\  \___|    <  /    Y    \/   \_/.  \    |     |    |
   \___  >____  /____/____/___  (____  /\___  >__|_ \ \____|__  /\_____\ \_/____|     |____|
       \/     \/              \/     \/     \/     \/         \/        \__>
*/

void callback_mqtt(char* topic, byte* payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print(" - long ");
  Serial.print(length);
  Serial.print("] ");
  string_value = "";
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    string_value += (char)payload[i];
  }
  Serial.println();

  //Decodage JSON
  JSONVar myObject = JSON.parse(string_value);
  if (JSON.typeof(myObject) == "undefined")
  {
    Serial.println("Echec du Parsing JSON!");
  }
  else
  {
    //json : {"code":,"channel":,"state":}
    if (myObject.hasOwnProperty("code"))
    {
      Serial.print("myObject[\"code\"] = ");
      Serial.println(myObject["code"]);
      stringCode = myObject["code"];
    }

    if (myObject.hasOwnProperty("channel"))
    {
      Serial.print("myObject[\"channel\"] = ");
      Serial.println((int) myObject["channel"]);
      intChannel = (int) myObject["channel"];

      //A:0FFF   B:F0FF  C:FF0F   D:FFF0
      intErrorCmdReceive++;
      if (intChannel == 0)
      {
        stringChannel = "0FFF";
        intErrorCmdReceive--;
      }
      if (intChannel == 1)
      {
        stringChannel = "F0FF";
        intErrorCmdReceive--;
      }
      if (intChannel == 2)
      {
        stringChannel = "FF0F";
        intErrorCmdReceive--;
      }
      if (intChannel == 3)
      {
        stringChannel = "FFF0";
        intErrorCmdReceive--;
      }
    }

    if (myObject.hasOwnProperty("state"))
    {
      Serial.print("myObject[\"state\"] = ");
      Serial.println((int) myObject["state"]);
      intState = (int) myObject["state"];

      //ON:F0F /OFF:FF0
      intErrorCmdReceive++;
      if (intState == 0)
      {
        stringState = "FF0";
        intErrorCmdReceive--;

      }
      if (intState == 1)
      {
        stringState = "F0F";
        intErrorCmdReceive--;

      }
    }

    //formatage trame radio 433MHz
    sprintf (buffer_cmd, "%s%s%s", stringCode, stringChannel, stringState);
    Serial.println("Commande RADIO 433MHz:");
    Serial.println(buffer_cmd);

    if (intErrorCmdReceive == 0)
    {
      intDemandeCmdRf433 = 1;
    }
    else
    {
      intDemandeCmdRf433 = 0;
      Serial.println("!!!!! Erreur commande recu non pris en compte");
    }

  }




  value = string_value.toInt();

  if (string_value.compareTo("resetreset") == 0)
    //if (string_value=="resetreset")
  {
    Serial.println("Reset board");
    while (1);
  }

  /*Serial.print("value [");
    Serial.print(string_value);
    Serial.print(" - ");
    Serial.print(value);
    Serial.print("] ");
    Serial.println();*/

}



/*
  __________                                                 __       _____   ______________________________
  \______   \ ____   ____  ____   ____   ____   ____   _____/  |_    /     \  \_____  \__    ___/\__    ___/
   |       _// __ \_/ ___\/  _ \ /    \ /    \_/ __ \_/ ___\   __\  /  \ /  \  /  / \  \|    |     |    |
   |    |   \  ___/\  \__(  <_> )   |  \   |  \  ___/\  \___|  |   /    Y    \/   \_/.  \    |     |    |
   |____|_  /\___  >\___  >____/|___|  /___|  /\___  >\___  >__|   \____|__  /\_____\ \_/____|     |____|
          \/     \/     \/           \/     \/     \/     \/               \/        \__>
*/
void reconnect() {

  int compteur_mqtt = 0;

  //test wifi connexion
  if (wifiMulti.run() == WL_CONNECTED) {

    // Loop until we're reconnected
    while (!(client.connect(clientIdMqtt.c_str(), clientLoginMqtt.c_str(), clientPassMqtt.c_str())))
    {
      //affichage bleu not mqtt
      setBuff(0x00, 0x00, 0x44);
      M5.dis.displaybuff(DisBuff);
      delay(2000);

      //réarmement du watchdog (echoue si > 10secondes)
      timerWrite(timer, 0); //reset timer (feed watchdog)

      compteur_mqtt++;
      if (compteur_mqtt >= TEST_MQTT) break;

      Serial.print("Attente connexion MQTT...");

      //delai reboot
      delay(random(1000)); // Delay for a period of time (in milliseconds).

      // Create a random client ID
      //!!!clientId = "id_random_";
      //!!!clientId += String(random(0xffff), HEX);
      client.setCallback(callback_mqtt);

      // Attempt to connect
      if (client.connect(clientIdMqtt.c_str(), clientLoginMqtt.c_str(), clientPassMqtt.c_str()))
      {
        Serial.println("Connexion MQTT OK");

        //affichage verte
        setBuff(0x00, 0x44, 0x00);
        M5.dis.displaybuff(DisBuff);
        delay(2000);

        //effacement ecran
        setBuff(0x00, 0x00, 0x00);
        M5.dis.displaybuff(DisBuff);

        intcounter++;
        sprintf (msg, "{\"ssid\":\"%s\",\"rssi\":%d,\"counter\":%d,\"tempcpu\":%d}", WiFi.SSID().c_str(), WiFi.RSSI(), intcounter, intTempCpu);

        Serial.print("Publish message: ");
        Serial.print(clientIdMqtt.c_str());
        Serial.print(" ");
        Serial.println(msg);
        client.publish(clientIdMqtt.c_str(), msg);

        // init subscribe
        sprintf(buffer_tmp, "%s", subscribe_str.c_str());
        client.subscribe(buffer_tmp);

        //rechargement ecoute MQTT
        client.setCallback(callback_mqtt);
      }
      else
      {
        Serial.print("Echec, rc=");
        Serial.print(client.state());
        Serial.println(" Retest dans 5 secondes");
      }
      // Wait 5 seconds before retrying
      delay(5000);
    }

    if (client.connect(clientIdMqtt.c_str(), clientLoginMqtt.c_str(), clientPassMqtt.c_str()))
    {
      Serial.println("Connexion MQTT OK");
      count_lost_mqtt++;

      //effacement ecran
      setBuff(0x00, 0x00, 0x00);
      M5.dis.displaybuff(DisBuff);

      delay(2000);
      //delai reboot
      delay(random(200)); // Delay for a period of time (in milliseconds).

      intcounter++;
      sprintf (msg, "{\"ssid\":\"%s\",\"rssi\":%d,\"counter\":%d,\"tempcpu\":%d}", WiFi.SSID().c_str(), WiFi.RSSI(), intcounter, intTempCpu);

      Serial.print("Publish message: ");
      Serial.print(clientIdMqtt.c_str());
      Serial.print(" ");
      Serial.println(msg);
      client.publish(clientIdMqtt.c_str(), msg);

      // init subscribe
      sprintf(buffer_tmp, "%s", subscribe_str.c_str());
      client.subscribe(buffer_tmp);

      //rechargement ecoute MQTT
      client.setCallback(callback_mqtt);

    }

  } //end test wifi connexion

}

void status_wifi(void);
void status_mqtt(void);



/*
    _____                    __  .__
  _/ ____\_ __  ____   _____/  |_|__| ____   ____   ______
  \   __\  |  \/    \_/ ___\   __\  |/  _ \ /    \ /  ___/
   |  | |  |  /   |  \  \___|  | |  (  <_> )   |  \\___ \
   |__| |____/|___|  /\___  >__| |__|\____/|___|  /____  >
                   \/     \/                    \/     \/
*/
//---------------------FUNCTION----------------------------------

void sensor_in_live(void);
void measure_distance(int int_spec);
void compteur (void);
void print_wakeup_reason();
void void_fct_info_uart(unsigned long ulong_interval);

void send_status_mqtt(unsigned long ulong_interval);

//watchdog reset
void IRAM_ATTR resetModule()
{
  ets_printf("reboot watchdog\n");
  esp_restart();
}

//buffer used for neopixel matrix on ATOM_MATRIX
void setBuff(uint8_t Rdata, uint8_t Gdata, uint8_t Bdata)
{
  DisBuff[0] = 0x05;
  DisBuff[1] = 0x05;
  for (int i = 0; i < 25; i++)
  {
    DisBuff[2 + i * 3 + 0] = Rdata;
    DisBuff[2 + i * 3 + 1] = Gdata;
    DisBuff[2 + i * 3 + 2] = Bdata;
  }
}

// ------------------------INTERNAL TEMPERATURE ----------------
#ifdef __cplusplus
extern "C" {
#endif
uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif
uint8_t temprature_sens_read();
// ---------------------FIN INTERNAL TEMPERATURE ----------------



/*
                 __
    ______ _____/  |_ __ ________
   /  ___// __ \   __\  |  \____ \
   \___ \\  ___/|  | |  |  /  |_> >
  /____  >\___  >__| |____/|   __/
       \/     \/           |__|
*/
void setup() {

  M5.begin(true, false, true);
  // M5.Power.begin();

  //setup watchdog sur 10 secondes
  timer = timerBegin(0, 80, true);                  //timer 0, div 80
  timerAttachInterrupt(timer, &resetModule, true);  //attach callback
  timerAlarmWrite(timer, wdtTimeout * 1000, false); //set time in us
  timerAlarmEnable(timer);                          //enable interrupt

  setBuff(0xff, 0x00, 0x00);
  M5.dis.displaybuff(DisBuff);
  delay(10);

  //button
  pinMode(BUTTON_PIN, INPUT);

  //pir sensor
  pinMode(PIR_PIN, INPUT);

  Serial.begin(115200);
  delay(1000);
  Serial.println("Start programm.");

  //CPU information on boot
  //function takes the following frequencies as valid values:
  //  240, 160, 80    <<< For all XTAL types
  //  40, 20, 10      <<< For 40MHz XTAL
  //  26, 13          <<< For 26MHz XTAL
  //  24, 12          <<< For 24MHz XTAL
  //bool setCpuFrequencyMhz(uint32_t cpu_freq_mhz);

  uint32_t ui32XtalFreq = getXtalFrequencyMhz(); // In MHz
  uint32_t ui32CpuFreq  = getCpuFrequencyMhz();  // In MHz
  uint32_t ui32ApbFreq  = getApbFrequency();     // In Hz

  Serial.print("Frequence horloge:");
  Serial.println(ui32XtalFreq);
  Serial.print("frequence du CPU:");
  Serial.println(ui32CpuFreq);
  Serial.print("Frequence du bus APB:");
  Serial.println(ui32ApbFreq);

  //change frequency 80MHz
  Serial.print("Changement frequence CPU a 160MHz:");
  bool boolCpuFreq = false;
  boolCpuFreq = setCpuFrequencyMhz(160);
  Serial.println(boolCpuFreq);

  ui32XtalFreq = getXtalFrequencyMhz(); // In MHz
  ui32CpuFreq  = getCpuFrequencyMhz();  // In MHz
  ui32ApbFreq  = getApbFrequency();     // In Hz

  Serial.print("Frequence horloge:");
  Serial.println(ui32XtalFreq);
  Serial.print("frequence du CPU:");
  Serial.println(ui32CpuFreq);
  Serial.print("Frequence du bus APB:");
  Serial.println(ui32ApbFreq);

  //Increment boot number and print it every reboot
  bootCount = bootCount + 1;
  delay(10);
  Serial.println("Boot number: " + String(bootCount));

  //Print the wakeup reason for ESP32
  print_wakeup_reason();

  //réarmement du watchdog (echoue si > 10secondes)
  timerWrite(timer, 0); //reset timer (feed watchdog)

  // Transmitter is connected to Arduino Pin #10 arduino #26 atom lite grove port
  mySwitch.enableTransmit(26);

  // Optional set protocol (default is 1, will work for most outlets)
  mySwitch.setProtocol(1);

  // Optional set pulse length.
  mySwitch.setPulseLength(325);

  // Optional set number of transmission repetitions.
  mySwitch.setRepeatTransmit(15);

  //Init wifi
  setup_wifi();

  //verif wifi
  status_wifi();

  //réarmement du watchdog (echoue si > 10secondes)
  timerWrite(timer, 0); //reset timer (feed watchdog)

  // abonnement MQTT
  client.setCallback(callback_mqtt);

  //connect MQTT
  reconnect();

}



/*
  .__
  |  |   ____   ____ ______
  |  |  /  _ \ /  _ \\____ \
  |  |_(  <_> |  <_> )  |_> >
  |____/\____/ \____/|   __/
                     |__|
*/
void loop() {

  //test wifi et reconnexion si probleme
  status_wifi();

  //status MQTT
  status_mqtt();

  client.loop();

  //envoi status MQTT toute les 20 secondes (en vie)
  send_status_mqtt(20000);

  //réarmement du watchdog (echoue si > 10secondes)
  timerWrite(timer, 0); //reset timer (feed watchdog)

  //period with millis()
  ulong_time_now = millis();

  //no measure if no wifi
  if ((wifiMulti.run() == WL_CONNECTED) &&
      ((client.connect(clientIdMqtt.c_str(), clientLoginMqtt.c_str(), clientPassMqtt.c_str()))))
  {
    if (acq_restart == 1)
    {
      acq_restart = 0;
      //led off
      Serial.println("Connexion MQTT OK");

      //affichage verte
      setBuff(0x00, 0x44, 0x00);
      M5.dis.displaybuff(DisBuff);
      delay(2000);

      //effacement ecran
      setBuff(0x00, 0x00, 0x00);
      M5.dis.displaybuff(DisBuff);

      intcounter++;
      sprintf (msg, "{\"ssid\":\"%s\",\"rssi\":%d,\"counter\":%d,\"tempcpu\":%d}", WiFi.SSID().c_str(), WiFi.RSSI(), intcounter, intTempCpu);
      Serial.print("Publish message: ");
      Serial.print(clientIdMqtt.c_str());
      Serial.print(" ");
      Serial.println(msg);
      client.publish(clientIdMqtt.c_str(), msg);

      // init subscribe
      sprintf(buffer_tmp, "%s", subscribe_str.c_str());
      client.subscribe(buffer_tmp);

      //rechargement ecoute MQTT
      client.setCallback(callback_mqtt);
    }
  }
  else
  {
    Serial.println("Wifi ou MQTT PB");
    acq_restart = 1;
  }

  //debug uart
  void_fct_info_uart(1000);

  /* use RF module 433MHz or connect out of the RF command PHENIX */
  //{"code":,"channel":,"status":}
  //5 bit channel
  //4 bit A,B,C,D perso:00FF0
  //A:0FFF   B:F0FF  C:FF0F   D:FFF0
  //3 bit ON:F0F /OFF:FF0

  //Envoi trame RF si bonne reception MQTT
  if (intDemandeCmdRf433 == 1)
  {
    intDemandeCmdRf433 = 0;
    //trame a envoyer "buffer_cmd"
    mySwitch.sendTriState(buffer_cmd);
    Serial.print("TRAME RF envoyé ----->");
    Serial.println(buffer_cmd);
    delay(100);
  }
}




/*
  .___        _____               ____ ___  _____ _____________________
  |   | _____/ ____\____  ______ |    |   \/  _  \\______   \__    ___/
  |   |/    \   __\/  _ \/  ___/ |    |   /  /_\  \|       _/ |    |
  |   |   |  \  | (  <_> )___ \  |    |  /    |    \    |   \ |    |
  |___|___|  /__|  \____/____  > |______/\____|__  /____|_  / |____|
           \/                \/                  \/       \/
*/
void void_fct_info_uart(unsigned long ulong_interval)
{
  //ulong_diff_time_meas_cycle = ulong_time_now - ulong_time_meas_cycle;
  //ulong_time_meas_cycle = ulong_time_now;

  if ( (ulong_time_now - ulong_time_meas_cycle) >= ulong_interval )
  {
    ulong_time_meas_cycle = ulong_time_now;

    //Measure internal temperature esp32
    intTempCpu = ((temprature_sens_read() - 32) / 1.8);
    Serial.print(intTempCpu);
    Serial.print(",");

    //read PIR sensor
    intPirSensor = digitalRead(PIR_PIN);
    Serial.print(intPirSensor);
    Serial.println();

  }
}




/*
   __      __         __
  /  \    /  \_____  |  | __ ____    __ ________     ____ _____   __ __  ______ ____
  \   \/\/   /\__  \ |  |/ // __ \  |  |  \____ \  _/ ___\\__  \ |  |  \/  ___// __ \
   \        /  / __ \|    <\  ___/  |  |  /  |_> > \  \___ / __ \|  |  /\___ \\  ___/
    \__/\  /  (____  /__|_ \\___  > |____/|   __/   \___  >____  /____//____  >\___  >
         \/        \/     \/    \/        |__|          \/     \/           \/     \/
*/
//Function that prints the reason by which ESP32 has been awaken from sleep
void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason)
  {
    case 1  : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case 2  : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case 3  : Serial.println("Wakeup caused by timer"); break;
    case 4  : Serial.println("Wakeup caused by touchpad"); break;
    case 5  : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.println("Wakeup was not caused by deep sleep"); break;
  }
}




/*
    _________ __          __                    _____   ______________________________
   /   _____//  |______ _/  |_ __ __  ______   /     \  \_____  \__    ___/\__    ___/
   \_____  \\   __\__  \\   __\  |  \/  ___/  /  \ /  \  /  / \  \|    |     |    |
   /        \|  |  / __ \|  | |  |  /\___ \  /    Y    \/   \_/.  \    |     |    |
  /_______  /|__| (____  /__| |____//____  > \____|__  /\_____\ \_/____|     |____|
          \/           \/                \/          \/        \__>
*/
void status_mqtt(void)
{
  //test wifi connexion before test MQTT
  if (wifiMulti.run() == WL_CONNECTED)
  {
    //test MQTT et reconnexion si probleme
    if (!(client.connect(clientIdMqtt.c_str(), clientLoginMqtt.c_str(), clientPassMqtt.c_str())))
    {
      nb_test_ctl_mqtt++;
      if (nb_test_ctl_mqtt > TEST_MQTT)
      {
        nb_test_ctl_mqtt = 0;
        Serial.println("Deconnexion MQTT (pb MQTT)");
        reconnect();
      }
    }
    else
    {

    }
  }
}



/*
    _________ __          __                  __      __.______________.___
   /   _____//  |______ _/  |_ __ __  ______ /  \    /  \   \_   _____/|   |
   \_____  \\   __\__  \\   __\  |  \/  ___/ \   \/\/   /   ||    __)  |   |
   /        \|  |  / __ \|  | |  |  /\___ \   \        /|   ||     \   |   |
  /_______  /|__| (____  /__| |____//____  >   \__/\  / |___|\___  /   |___|
          \/           \/                \/         \/           \/
*/
void status_wifi(void)
{

  if (wifiMulti.run() != WL_CONNECTED)
  {
    delay(500);
    nb_test_ctl_wifi++;
    if (nb_test_ctl_wifi > TEST_WIFI)
    {
      nb_test_ctl_wifi = 0;
      Serial.println("Deconnexion wifi (pb wifi)\r\n");
      setup_wifi();
      Serial.println("Deconnexion MQTT (pb wifi)\r\n");
      reconnect();
    }
  }

}



/*
    _________                  .___    _____   ______________________________
   /   _____/ ____   ____    __| _/   /     \  \_____  \__    ___/\__    ___/
   \_____  \_/ __ \ /    \  / __ |   /  \ /  \  /  / \  \|    |     |    |
   /        \  ___/|   |  \/ /_/ |  /    Y    \/   \_/.  \    |     |    |
  /_______  /\___  >___|  /\____ |  \____|__  /\_____\ \_/____|     |____|
          \/     \/     \/      \/          \/        \__>
*/
//Information status of led to the MQTT server
void send_status_mqtt(unsigned long ulong_interval)
{
  if ( (ulong_time_now - ulong_time_send_meas_cycle) >= ulong_interval )
  {
    ulong_time_send_meas_cycle = ulong_time_now;

    //Abonnement MQTT
    client.setCallback(callback_mqtt);

    if ((client.connect(clientIdMqtt.c_str(), clientLoginMqtt.c_str(), clientPassMqtt.c_str())))
    {
      //envoi status MQTT
      intcounter++;
      sprintf (buffer_tmp, "{\"ssid\":\"%s\",\"rssi\":%d,\"counter\":%d,\"tempcpu\":%d}", WiFi.SSID().c_str(), WiFi.RSSI(), intcounter, intTempCpu);

      Serial.print("Publish message: ");
      Serial.print(clientIdMqtt.c_str());
      Serial.print(" ");
      Serial.println(buffer_tmp);
      client.publish(clientIdMqtt.c_str(), buffer_tmp);

      // init subscribe
      sprintf(buffer_tmp, "%s", subscribe_str.c_str());
      //Serial.println(buffer_tmp);
      client.subscribe(buffer_tmp);

      //rechargement ecoute MQTT
      client.setCallback(callback_mqtt);
    }

  }
}


/*
   TO DO
   - documentation du code
*/
