# arduino_esp32_m5atom_RF433


**__________________________ DEBUT SOMMAIRE _____________________________**

* [Librairie necessaire](#arduino_librairie)
* [Cablage](#Cablage)
* [Fonctionnement du logiciel](#fonctionnement)

**___________________________ FIN SOMMAIRE ______________________________**


**_______________________ LIBRAIRE ARDUINO _________________________**

# <a id="arduino_librairie">Libraire arduino necessaire</a>

Pour cette application les librairies suivantes sont necessaires :
 1. RC SWITCH (controle emetteur/recepteur RF 433MHz) : https://github.com/sui77/rc-switch/
 2. ARDUINO JSON (gestion formattage message JSON)
 3. WIFI multi (multi ssid connexion)
 4. PubSubClient (MQTT receive and send)

## Notes : Ajout manuel sur un depot github

Voici les taches à effectuer pour ajouter une librairie non disponible dans les dépots Arduino :
 1. Reperer le lien github d'une libraire Arduino
 2. Copier le lien git
 ![libraire rc-switch](./images/example_clone_git.png)
 3. ouvrir le repertoire "librairie" d'arduino localisation standard : C:\Users\<utilisateur>\Documents\Arduino\libraries
 4. clique droit sur la souris et choisir "Git clone" (git et tortoise git doivent etre installer)
 ![Clone de la libraire rc-switch](./images/git_clone_arduino_lib.png)
 5. Cliquer sur OK de la fentre de clonage de tortoise git
  ![Validation de l'emplacement de la libraire rc-switch](./images/git_clone_arduino_lib2.png)
 6. Voila la librairie est ajoutée à arduino.
   ![Installation de la libraire rc-switch finalisée](./images/git_clone_arduino_lib3.png)
 7. Note importante : pour la prise en compte d'ajout manuel de depot dans la librairie, il faut redemarrer arduino.

**_______________________ CABLAGE & INTERFACE RF _________________________**

# <a id="Cablage">CABLAGE & INTERFACE RF</a>

Lien informations M5stack ATOM lite : https://docs.m5stack.com/en/core/atom_lite

## Photo montage

![Photo montage](./images/atom_lite_connect_telecommande.png)

## interconnexion

![ATOM LITE pinout](./images/atom_lite_connect.png)

