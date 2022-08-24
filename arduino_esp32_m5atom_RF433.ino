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

*/

#include <RCSwitch.h>

RCSwitch mySwitch = RCSwitch();



void setup() {

  Serial.begin(115200);

  // Transmitter is connected to Arduino Pin #10 arduino #26 atom lite grove port
  mySwitch.enableTransmit(26);

  // Optional set protocol (default is 1, will work for most outlets)
  mySwitch.setProtocol(1);

  // Optional set pulse length.
  mySwitch.setPulseLength(325);

  // Optional set number of transmission repetitions.
  //mySwitch.setRepeatTransmit(15);

}




void loop() {

  /* use RF module 433MHz or connect out of the RF command PHENIX */
  //{"code":,"channel":,"status":}
  //5 bit channel
  //4 bit A,B,C,D perso:00FF0
  //A:0FFF   B:F0FF  C:FF0F   D:FFF0
  //3 bit ON:F0F /OFF:FF0

  //active bouton A
  Serial.println("POWER ON A");
  mySwitch.sendTriState("00FF00FFFF0F");
  delay(1000);
  //Attente 10 secondes
  for (int i = 0; i < 10; i++)
  {
    Serial.print(".");
    delay(1000);
  }
  Serial.println();


  //desactive bouton A
  Serial.println("POWER OFF A");
  mySwitch.sendTriState("00FF00FFFFF0");
  delay(1000);
  //Attente 10 secondes
  for (int i = 0; i < 10; i++)
  {
    Serial.print(".");
    delay(1000);
  }
  Serial.println();



  //active bouton A
  Serial.println("POWER ON B");
  mySwitch.sendTriState("00FF0F0FFF0F");
  delay(1000);
  //Attente 10 secondes
  for (int i = 0; i < 10; i++)
  {
    Serial.print(".");
    delay(1000);
  }
  Serial.println();


  //desactive bouton B
  Serial.println("POWER OFF B");
  mySwitch.sendTriState("00FF0F0FFFF0");
  delay(1000);
  //Attente 10 secondes
  for (int i = 0; i < 10; i++)
  {
    Serial.print(".");
    delay(1000);
  }
  Serial.println();

  while(1);
}
