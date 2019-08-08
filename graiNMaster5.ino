/*******************************************************
  GraiNMaster
  système de contrôle de température de la cuve
  Débuté le 24/11/2015 par Nicolas Morival
  Testé et compilé sur Arduino IDE v1.6.8
*******************************************************/

#define version  " --V1.004-- "

/*
  v1.000 le 14/05/2016
  V1.002 le 08/10/2016
  nettoyage du code, suppression des morceaux de code inutiles (relatifs à la SD)
  suppression du paramètre sur SD detect : cette broche est une interruption fixée par le hardware
  ajout de la possibilité d'utiliser une sonde DS18B20
  v1.003 le 09/11/2017
  correction de bugs sur l'initialisation de l'eeprom (bug relevé par artiche)
  mise en fonction du buzzer (suggestion artiche)
  v1.004 le 15/04/2019 - DaOurZ
  Ajout de la possibilité de deux sondes de température
  "Simplification" du code
  Désactivation possible de l'EEPROM, mode Debug, du buzzer, du défilement
  Changement de la température et du temps restant à la volée pour chaque palier en cours
  Sortie propre du programme en fin de brassage
  Création d'un fichier de log différent à chaque brassage
  Redéfinition des informations affichées à l'écran (avec témoin d'atteinte du palier)
*/

 
//bibliothèques
#include <SPI.h>
#include <SD.h>
#include <PID_v1.h> //gestion de l'algo PID
#include <avr/pgmspace.h>
#include "Timer.h" // gestion des timers
#include <LiquidCrystal.h> //gestion du LCD
#include <avr/io.h>
#include <avr/interrupt.h> //gestion des interruptions


//#define DEBUG           // Décommenter pour afficher les informations de deboguage (PID sur le LCD)
//#define DEUXSONDES      // Décommenter pour l'utilisation de deux sondes de température (DS18B202) - il est possible de n'utiliser qu'un seul canal OneWire pour toutes les sondes, ce n'est pas l'option d'ici, ce sont bien deux canaux distincts !
//#define DAC             // Décommenter pour utilisation du DAC (Version Originale)
//#define BIPPEUR         // Commenter pour ne pas utiliser de buzzer
//#define EEPROMRW        // Décommenter pour enregistrement sur l'EEPROM si pas de carte SD
#define DEFIL           // Commenter pour ne pas faire alterner le texte sur la deuxième ligne pour afficher le temps restant total avant la fin de l'ébullition
#define PIDDEBUG        // Commenter pour enlever l'information du PID sur l'écran lors de la chauffe

#ifdef PIDDEBUG
  #define PID_DEBUG(x) + String(x)
#else
  #define PID_DEBUG(x)
#endif
    
#ifdef DEBUG
  #define DEBUGPRINTLN(x) Serial.println(x)
#else
  #define DEBUGPRINTLN(x)
#endif

#ifdef DEUXSONDES
  #define DIF_DEUXSONDES(x) + String(x)
#else
  #define DIF_DEUXSONDES(x)
#endif

#ifdef EEPROMRW
  #include <EEPROMex.h>
  #include <EEPROMVar.h>  // gestion des enregistrements en EEPROM
#endif

#define BLINK() ledState = !ledState; digitalWrite(ledPin, ledState);

#ifdef DEFIL
  bool defilement = 0;    // Position du défilement / 1 = température et température à atteindre / 0 = température et temps restant / s'échange à chaque écriture sur la carte SD, donc toutes les 10 s environ
#endif

byte DETECT = 2 , GATE = 3; //Broches utilisées pour le pilotage du triac -> detect : impulsion du passage au point zéro, gate : gachette du triac
byte PULSE = 4  ; //triac gate pulse : largeur de l'impulsion pour activer le triac, en nombre de cycles d'horloge
int HALF_WAVE = 560;// //nombre de "tics" d'horloge à chaque demi onde 50Hz . 625 théoriques, mais 560 réels .

#if !(defined(DAC)) // Déclaration de la sonde de température DS18B20 si l'on n'utilise pas de DAC
  #include <Wire.h>
  #include <OneWire.h>
  #include <DallasTemperature.h>

  byte ONE_WIRE_BUS = 30;  // DS18B20 pin
  OneWire oneWire(ONE_WIRE_BUS);
  DallasTemperature DS18B20(&oneWire);
#endif

// Déclaration de la deuxième sonde DS18B20 si jamais elle doit être définie (DEUXSONDES doit exister et ne pas être commenté)
#if defined(DEUXSONDES)
  byte ONE_WIRE_BUS2 = 31; // DS18B20 pin (2ème sonde)
  OneWire oneWire2(ONE_WIRE_BUS2);
  DallasTemperature DS18B202(&oneWire2);
  float diff_deuxsondes; // Différence entre les deux sondes, si deux sont utilisées
#endif

// constantes pour les touches reliées au module LCD
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

#if defined(DAC)
  #include <Adafruit_ADS1015.h> // gestion du CAN 16 bits
  // pour la lecture de la sonde via le DAC (Version Originale)
  byte THERM_PIN2  = 1 ;// entrée n°1 du dac 16bits
  byte THERM_PIN  = 3 ; // entrée n°3 du dac 16bits
  //on lit sur deux entrées dont les filtres passe bas sont légèrement différents, afin de faire une moyenne et augmenter la précision
  
  int OFFSET_PROBE  = 138; // correction constante de la température
  int COEF_PROBE = 260 ; // correction proportionelle
  // Température calculée = (mesure du DAC  - OFFSET_PROBE) / COEF_PROBE

  // déclaration de l'ADC 16 bits - DAC pour température (Version Originale)
  Adafruit_ADS1115 ads1115(0x48);  // ADC16 bits addresse 0x48
/*
  String variables_SD_DAC[] = {
    "THERMPIN2",
    "THERMPIN1",
    "OFSETPRBE",
    "COEFPROBE"
  };
*/
#endif

//valeurs de l'hysteresis pour passage en PID fin ou grossier
byte hysteresis_Pos =  1;
byte hysteresis_Neg  = 1;

//  pins utilisées par le LCD
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// valeurs utilisées pour configurer le LCD
byte backLight   = 10;    // LCD Panel Backlight LED connected to digital pin 10
int lcd_key     = 0;
int adc_key_in  = 0;
byte ledPin = 13;
byte ledState = LOW;

// variables du programme
volatile unsigned int menu = 0;
byte submenu = 0;
byte startprog = 0; //marqueur pour indiquer que le programme s'éxécute
byte jump = 0; //pour indiquer qu'on va sauter un menu

byte annuler = 0; //pour annuler les opérations 0 = fonctionnement normal  / -1 on recule d'une étape / 1 on avance d'une étape

byte secondes = 0;
unsigned long secondes_reel = 0; //temps écoulé depuis le début du programme, utilisé pour le data log
unsigned int minutes = 0; //minutes écoulées
unsigned long total_time = 0; //cumul des temps programme
unsigned long temps_pause = 0; //si le programme est en pause, compte le temps écoulé
bool palier_atteint = false;
int logRecord_ID;   // Timer ID pour la fonction logRecord, pour un arrêt propre du programme (on veut le stopper avant d'arrêter le programme pour éviter toute écriture quand on ddébranche l'Arduino)

String myPaliers[2][7] = {
  {"Prechauffe", "Palier 1  ", "Palier 2  ", "Palier 3  ", "Palier 4  ", "Mash-out  ", "Ebullition"},   // Nom des paliers en version longue (10 caractères)
  {"PREC",         "PAL1",     "PAL2",     "PAL3",     "PAL4",     "MOUT",     "EBUL"      }    // Nom des paliers en version courte (4 caractères)
};

float myTemperatures[] = {
  40,     // = thetastart, température de préchauffe de la cuve
  40.25,  // = theta1, température empâtage céréales non maltées
  52.5,   // = theta2, température protéines
  62.0,   // = theta3, température de sacharification B amylase
  68.75,  // = theta4, température de sacharification A amylase
  75.75,  // = thetaMO, température de mash out
  103.0   // = theta5, température d'ébullition
}; // Températures pour chaque myPaliers

byte myTempo[] {
  0,      // tempo_A, pour l'affichage temporaire
  0,      // tempo1, palier empâtage céréales non maltées
  15,     // tempo2, palier protéinique
  35,     // tempo3, palier de sacharification B amylase
  30,     // tempo4, palier de sacharification A amylase
  5,      // palier de Mash Out
  70      // tempo5, ébullition
}; // Temporisation pour chaque myPaliers

unsigned int cooling = 0; // compteur du temps de refroidissement

float last_temp = 0; // dernière température positive

//int therm; //variable pour le relevé de température

// double chauffe_reel = 0, tx_chauffe, theta_mesure, theta_PID, theta_objectif = 20;
float tx_chauffe;
float theta_PID, theta_mesure, theta_objectif = 20;

// Règlages par défaut du PID
// Ces constantes sont correctes pour une cuve de 27L 2500W non isolée type kitchen chef

byte P_strong = 100;
float I_strong = 0.0;
byte D_strong = 16;

byte P_weak = 80;
float I_weak = 0.02;
byte D_weak = 8;

double Kp = P_strong; // multiplicateur de l'erreur différentielle de température.
double Ki = I_strong; //coef de correction inverse
double Kd = D_strong; //coef de dérivée

byte PID_OFFSET = 1; // on ruse le PID pour lui faire décaller la température cible d'une valeur constante par exemple +1° donc si l'utilisateur vise 62° et que la température mesurée est de 61° le PID croit qu'il a atteint les 62° et coupe la chauffe. Ainsi l'overshoot est limité.
// en théorie, avec les paramètres réglés au top, ce paramètre peut être remis à zéro.

PID myPID((double*)&theta_PID, (double*)&tx_chauffe, (double*)&theta_objectif, Kp, Ki, Kd, DIRECT); //déclaration du PID

#ifdef BIPPEUR
    //pin du bipeur
    byte Beep_PIN = 40;
    // variables utilisées pour faire marcher le bippeur
    #define BeepONState  LOW
    #define BeepOFFState  HIGH
    #define BeepON(s) BeepON(s)
    byte beep_duration = 0;   // temps du bip : s / 128, 128 ms étant le temps entre chaque appel de sel_menu, la plus rapide des périodes pour vérifier que le bip est en route ou non 
    void BeepON(unsigned int s) {
      digitalWrite(Beep_PIN, BeepONState);
      beep_duration = (byte) ceil(s / 128);   // la durée du bip est comptée par période de 128 ms (arrondies à l'entier supérieur) - sel_menu, qui va vérifier si le bip est en route ou non, est appelée toutes les 128 ms
    }
    void BeepOFF() {
      digitalWrite(Beep_PIN, BeepOFFState);       
    }
#else
    #define BeepON(s)
#endif


//réglages des timers
Timer T;

//déclaratioon du fichier pour la SD
File myFile;
String datalogFile; // Fichier de log, à chaque brassage différent
int check_data = 0;

char degre = (char)223;  // char 223 = ° (signe degré)


void setup()
{

  #if defined(DEBUG)
      Serial.begin(9600); 
  #endif
  
  #if defined(DAC)
      // démarrage du DAC 16 bits
      ads1115.begin(); // (Version Originale)
  #endif
     
  //config pour SD
  String inString = "";
  
  pinMode(53, OUTPUT); //la pin 53 est normalement attribuée à la fonction SPI Slave Select (SS)Permet d'activer ou désactiver le périphérique
  // Suivant les modules SD, il peut être nécessaire de commenter cette ligne (c'est le cas de mon module même si c'est pas logique)


  // ECRAN DE PRESENTATION ----------------------------
  lcd.begin(16, 2);     // déclaration de la bibliothèque LCD avec 16 caractères, 2 lignes

  // création des caractères spéciaux
  byte beer1[8] = {
    B10001,
    B11111,
    B10001,
    B10001,
    B10001,
    B10001,
    B11111,
  };
  byte beer2[8] = {
    B00000,
    B11100,
    B00100,
    B00100,
    B00100,
    B00100,
    B11000,
  };

  lcd.createChar(0, beer1);
  lcd.createChar(1, beer2);
  CLS();           // fonction qui efface le LCD et met le curseur au début
  lcd.print("- GraiN.Master -");
  lcd.setCursor(0, 1);
  lcd.write(byte(0));
  lcd.write(byte(1));
  lcd.print(version);
  lcd.write(byte(0));
  lcd.write(byte(1));
  delay(2000);


  //Effet de scrolling sympa
  for (int positionCounter = 0; positionCounter < 16; positionCounter++) {
    // scroll one position left:
    lcd.scrollDisplayLeft();
    // wait a bit:
    delay(300);
  }

  // set up des pins
  pinMode(DETECT, INPUT);     //détection du passage au pont zéro / zero cross detect
  digitalWrite(DETECT, HIGH); // enable pull-up resistor
  pinMode(GATE, OUTPUT);      //triac gate control
  #if (defined(BIPPEUR))
    pinMode(Beep_PIN, OUTPUT);  // Déclaration du bipper
    BeepOFF();
  #endif

  pinMode(backLight, INPUT); //set backlight pin to input to avoid MCU overcurent on pin 10
  // pinMode(11, OUTPUT);     // pin11 = MOSI sur uno
  pinMode(ledPin, OUTPUT);

  // set up Timer1
  //(see ATMEGA 328 data sheet pg 134 for more details)
  OCR1A = 25;      //initialize the comparator
  TIMSK1 = 0x03;    //enable comparator A and overflow interrupts
  TCCR1A = 0x00;    //timer control registers set for
  TCCR1B = 0x00;    //normal operation, timer disabled

  myPID.SetOutputLimits(0, 255);

  #if (defined(EEPROMRW))
      EEPROM.setMaxAllowedWrites(48);
  #endif

  // pinMode(10, OUTPUT);
  CLS();          // move cursor to beginning of line "0"
  lcd.print("initialise  SD  "); // print a simple message
  delay(600);
  if (!SD.begin(4)) {
    lcd.home();           // move cursor to beginning of line "0"
    lcd.print("ERREUR SD CARD "); // print a simple message
    delay(6000);
  }

  CLS();           // move cursor to beginning of line "0"
  lcd.print(" Lecture  param "); // print a simple message

  if (SD.exists("setup.grm")) {

    lcd.setCursor(0, 1);
    lcd.print(" sur carte SD ");
    delay(1500);

    myFile = SD.open("setup.grm");

    while (myFile.available()) {
      int inChar = myFile.read();
      if ((inChar != ',')) {
        inString += (char)inChar;
      } //fin if alphanum

      else { //on a récupéré la première ligne sous forme "variable=000.00"
        //        Serial.print("Input string: \n");
        //       Serial.print(inString);

        if (inString.startsWith("THERMPIN2"))
        {
          inString.remove(0, 10); // Remove 10 characters starting at index=0

          #if defined(DAC)
              THERM_PIN2 = inString.toInt();
          #endif
          check_data++;

        }

        if (inString.startsWith("THERMPIN1"))
        {
          inString.remove(0, 10); // Remove 10 characters starting at index=0
          #if defined(DAC)
              THERM_PIN = inString.toInt();
          #endif
          check_data++;

        }
        if (inString.startsWith("OFSETPRBE"))
        {
          inString.remove(0, 10); // Remove 10 characters starting at index=0
          #if defined(DAC)
              OFFSET_PROBE = inString.toInt();
          #endif
          check_data++;
        }

        if (inString.startsWith("COEFPROBE"))
        {
          inString.remove(0, 10); // Remove 10 characters starting at index=0
          #if defined(DAC)
              COEF_PROBE = inString.toInt();
          #endif
          check_data++;

        }

        if (inString.startsWith("hysterPos"))
        {
          inString.remove(0, 10); // Remove 10 characters starting at index=0
          hysteresis_Pos = inString.toInt();
          check_data++;

        }
        if (inString.startsWith("hysterNeg"))
        {
          inString.remove(0, 10); // Remove 10 characters starting at index=0
          hysteresis_Neg = inString.toInt();
          check_data++;

        }
        
        for (int i = 0; i < 7; i++) {
          if (inString.startsWith(myPaliers[0][i])) {   // Température (de préchauffage ou de palier)
                  DEBUGPRINTLN("Reading " + myPaliers[0][i] + " temperature:");
                  inString.remove(0, 10); // Remove 10 characters starting at index=0
                  DEBUGPRINTLN(inString.toFloat());
                  myTemperatures[i] = inString.toFloat();
                  check_data++;
          }

          if (inString.startsWith(myPaliers[1][i] + "TEMPO")) {  // Temporisation pour chaque palier
                  DEBUGPRINTLN("Reading " + myPaliers[0][i] + " timing:");
                  inString.remove(0, 10); // Remove 10 characters starting at index=0
                  DEBUGPRINTLN(inString.toInt());
                  myTempo[i] = inString.toInt();
                  check_data++;
          }

        }

        if (inString.startsWith("Pstrong00"))
        {
          inString.remove(0, 10); // Remove 10 characters starting at index=0
          P_strong = inString.toFloat();
          check_data++;

        }

        if (inString.startsWith("Istrong00"))
        {
          inString.remove(0, 10); // Remove 10 characters starting at index=0
          I_strong = inString.toFloat();
          check_data++;

        }
        if (inString.startsWith("Dstrong00"))
        {
          inString.remove(0, 10); // Remove 10 characters starting at index=0
          D_strong = inString.toFloat();
          check_data++;

        }

        if (inString.startsWith("Pweak0000"))
        {
          inString.remove(0, 10); // Remove 10 characters starting at index=0
          P_weak = inString.toFloat();
          check_data++;

        }

        if (inString.startsWith("Iweak0000"))
        {
          inString.remove(0, 10); // Remove 10 characters starting at index=0
          I_weak = inString.toFloat();
          check_data++;

        }
        if (inString.startsWith("Dweak0000"))
        {
          inString.remove(0, 10); // Remove 10 characters starting at index=0
          D_weak = inString.toFloat();
          check_data++;

        }

        if (inString.startsWith("LEDPIN000"))
        {
          inString.remove(0, 10); // Remove 10 characters starting at index=0
          ledPin = inString.toInt();
          check_data++;

        }
        if (inString.startsWith("GATE00000"))
        {
          inString.remove(0, 10); // Remove 10 characters starting at index=0
          GATE = inString.toInt();
          check_data++;

        }
        if (inString.startsWith("PULSE0000"))
        {
          inString.remove(0, 10); // Remove 10 characters starting at index=0
          PULSE = inString.toInt();
          check_data++;

        }
        if (inString.startsWith("HALFWAVE0"))
        {
          inString.remove(0, 10); // Remove 10 characters starting at index=0
          HALF_WAVE = inString.toInt();
          check_data++;

        }
        if (inString.startsWith("BEEPPIN00"))
        {
          inString.remove(0, 10); // Remove 10 characters starting at index=0
          #if (defined(BIPPEUR))
            Beep_PIN = inString.toInt();
          #endif
          check_data++;

        }
        if (inString.startsWith("PIDOFFSET"))
        {
          inString.remove(0, 10); // Remove 10 characters starting at index=0
          PID_OFFSET = inString.toInt();
          check_data++;

        }

        if (inString.startsWith("ONEWIRE00"))
        {
          inString.remove(0, 10); // Remove 10 characters starting at index=0
          #if !(defined(DAC))
              ONE_WIRE_BUS = inString.toInt();
          #endif
          check_data++;
        }

        if (inString.startsWith("ONEWIRE02"))
        {
          inString.remove(0, 10); // Remove 10 characters starting at index=0
          #if (defined(DEUXSONDES))
              ONE_WIRE_BUS2 = inString.toInt();
          #endif
          check_data++;
        }

        inString = "";

      } //fin else


    } //fin while


    CLS();
    lcd.print(check_data);
    lcd.setCursor(0, 1);
    lcd.print("parametres lus ");
    myFile.close();
    delay(1500);
  }
  else {
    // Pas possible de lire sur la carte SD --> on lit sur l'EEPROM si c'est défini...
    #if (defined(EEPROMRW))
        lcd.setCursor(0, 1);
        lcd.print(" sur EEPROM ");
        delay(1500);
        myTempo[1] = EEPROM.readLong(0);
        myTempo[2] = EEPROM.readLong(7);
        myTempo[3] = EEPROM.readLong(15);
        myTempo[4] = EEPROM.readLong(23);
        myTempo[6] = EEPROM.readLong(31);
        myTemperatures[1] = EEPROM.readFloat(39);
        myTemperatures[2] = EEPROM.readFloat(47);
        myTemperatures[3] = EEPROM.readFloat(55);
        myTemperatures[4] = EEPROM.readFloat(63);
        myTemperatures[6] = EEPROM.readFloat(71);
        myTemperatures[5] = EEPROM.readFloat(79);
        myTemperatures[0] = EEPROM.readFloat(87);
    #else     // sinon on reste sur les valeurs par défaut définies en dur dans le programme
        lcd.setCursor(0, 1);
        lcd.print("  par defaut  ");
        delay(1500);
    #endif
  }

//////// Pour créer un nom de fichier différent à chaque brassage (ou presque), on prend la durée que l'utilisateur a pris pour appuyer sur le bouton comme variable de seed pour la fonction random (aléatoire)
  unsigned long millisec = millis();
  CLS();
  lcd.print(" APPUYEZ SUR UN ");
  lcd.setCursor(0, 1);
  lcd.print("     BOUTON     ");
  while(read_LCD_buttons() == btnNONE); // On attend que l'utilisateur appuie sur un bouton
  randomSeed(millisec - millis());     // On initialise de manière pseudo-aléatoire le générateur de nombres pseudo-aléatoires, avec la durée de pression sur un bouton en millisecondes
  datalogFile = "log" + String(random(0, 99999)) + ".txt"; // Le nom du fichier, avec un numéro pseudo-aléatoire pour éviter de réécrire sur le même à chaque brassin
  DEBUGPRINTLN("Datalogfile = " + datalogFile);

  digitalWrite(ledPin, 1);
  File dataFile = SD.open(datalogFile, FILE_WRITE);
  if (dataFile) {
    dataFile.print("Temps (secondes)");
    dataFile.print(";");
    dataFile.print("Palier en cours");
    dataFile.print(";");
    dataFile.print("Objectif de temperature (°C)");
    dataFile.print(";");
    dataFile.print("Temperature mesuree (°C)");
    dataFile.print(";");
    dataFile.print("PID - taux de chauffe (0-255)");
    dataFile.print('\n');
    dataFile.close();
  }
  digitalWrite(ledPin, 0);

  T.every(2205, lecture); // constante de temps pour le timer 1 - lecture de la sonde
  T.every(128, sel_menu); // constante de temps pour le timer 2 - lecture des touches
  T.every(1024, LCD_upd); // timer 3 - mise à jour écran
  T.every(1000, horloge); // timer 4 - compte les secondes et minutes
  T.every(350, regle_chauffe); //timer 5 - fréquence de mise à jour de la chauffe
  logRecord_ID = T.every(10002, logRecord); //enregiste sur la carte SD
  
  OCR1A = 500;
  CLS();
  LCD_upd();
  // set up zero crossing interrupt
  attachInterrupt(digitalPinToInterrupt(2), zeroCrossingInterrupt, CHANGE);
  //IRQ0 is pin 2. Call zeroCrossingInterrupt
  //on rising signal

  menu = 0;

}

//Boucle principale ========================================

void loop()
{

  T.update();

  //gestion des sauts d'étapes manuels
  if (annuler == -1)
  {
    menu--; //on était au menu X on retourne au X-1
    annuler = 0;
    logRecord();
  }
  else {
    if (annuler == 1)
    {
      menu++;
      jump = 0; //par sécurité on ne veut pas de double saut au cas où le passage au palier suivant était prévu au même moment
      annuler = 0;
      logRecord();
      palier_atteint = false; // le palier n'est pas encore atteint, il vient de commencer
    }
  }

  //gestion des sauts d'étapes automatiques
  if (jump == 1) {
    menu++;
    jump = 0;
    logRecord();
    BeepON(1500);
    palier_atteint = false; // le palier n'est pas encore atteint, il vient de commencer
  }

} //fin Boucle principale ===================================

// Sélection de la température et du temps du palier
void paliers_temp(int numpalier, String *line1, String *line2) {
        *line1 = myPaliers[0][numpalier] + " ?";
        if (submenu == 0) {
          *line2 = ">T:" + String(myTemperatures[numpalier]) + degre + "  " + String(myTempo[numpalier]) + " min";
        }
        else {
          *line2 = " T:" + String(myTemperatures[numpalier]) + degre + " >" + String(myTempo[numpalier]) + " min";
        }
}

// Sauter le palier en cours
void pass_palier(int numpalier, String *line1, String *line2) {
        *line1 = "SAUTER " + myPaliers[0][numpalier] + " ?";
        *line2 = "SEL=OUI autre=NO";
}

// Annuler le palier en cours
void restart_palier(int numpalier, String *line1, String *line2) {
        *line1 = "RESTART " + myPaliers[0][numpalier] + " ?";
        *line2 = "SEL=OUI autre=NO";  
}

void display_palier(int numpalier, String *line1, String *line2) {
  unsigned int t = 0;
  // L'objectif de température est celui du palier en cours
  theta_objectif = myTemperatures[numpalier];
  // On ajoute tous les temps des paliers pour connaître le temps restant
  for (int i = 1; i <= numpalier; i++) {
    t += myTempo[i];
  }
  DEBUGPRINTLN("t : " + String(t) + " / minutes : " + String(minutes));
  // Si le temps n'est pas dépassé (t = temps total qu'il faut atteindre depuis le premier palier jusqu'à celui en cours)
  if (t > minutes) {
    myTempo[0] = t - minutes;
  }
  else {  // le palier est fini : on saute au palier suivant automatiquement
    jump = 1;  
    BeepON(200);
    palier_atteint = false;
    return;
  }
  *line1 = myPaliers[1][numpalier] + "(" + String(myTempo[numpalier]) + ")";
  
  // on affiche un caractère différent s'il y a chauffe (">") ou si le palier a déjà été atteint (":")
  if (palier_atteint) {
    *line1 = *line1 + ":" + String(myTempo[0]) + " min";
  }
  else {
    *line1 = *line1 + ">" + String(myTempo[0]) + " min";
  }

  *line2 = String((int)theta_mesure) + "\/" + String((int)myTemperatures[numpalier]);
#ifdef DEFIL
  if (defilement) { // on affiche alternativement la température en cours et la température à atteindre, ou...
#endif
    *line2 = *line2 + degre + "C ";
    *line2 = *line2 DIF_DEUXSONDES("DIF:" + String((int)((abs(diff_deuxsondes))))); // Ne va s'ajouter que si on active les deux sondes (désactivation : enlever #define DEUXSONDES)
    *line2 = *line2 PID_DEBUG("PID:" + String((int)ceil(tx_chauffe)) + "  "); // Ne va s'ajouter que si on active la fonction de debug (désactivation : enlever #define PIDDEBUG)
#ifdef DEFIL
  }
  else {  // ... la température en cours et le temps total restant estimé avant la fin de l'ébullition, sans le temps inter-palier néanmoins
      t = 0;
      for (int i = numpalier + 1; i <= 6; i++) {
        t += myTempo[i];
      }
      t += myTempo[0];
      *line2 = *line2 + " REST:" + String(t) + "min";
  }
#endif
}   // Fin Display_palier

void LCD_upd() // affiche les infos à l'écran *********************************************************
{
  String Ligne1;
  String Ligne2;
//  CLS(); // on efface l'écran et on place le curseur au début de la première ligne

  switch (menu)
  {
    case 0:   // Sélection de température de préchauffage
      {
        Ligne1 = myPaliers[0][menu] + " ?";
        Ligne2 = ">T" + String(myTemperatures[0]) + degre + "C";
        break;
      }


    case 1:   // Sélection de températures des paliers 1 à 6 - inclus Mash-out et ébullition
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
      {
        paliers_temp(menu, &Ligne1, &Ligne2);
        //fin sous menu
        break;
      }


    case 7:
      { // Possibilité de ne pas sauvegarder sur carte SD nos règlages (pour quelque raison que ce soit)
        Ligne1 = "Sauvegarder ?   ";
        Ligne2 = "SEL=oui DROIT=no";
        break;
      }

    case 8:   // préchauffage
    {
      theta_objectif = myTemperatures[0];
      Ligne1 = myPaliers[1][0];
      Ligne2 = String((int)theta_mesure) + "\/" + String((int)myTemperatures[0]) + degre + "C ";
      Ligne2 = Ligne2 PID_DEBUG("PID:" + String(tx_chauffe)); // Ne va s'ajouter que si on active la fonction de debug (désactivation : commenter #define DEBUG)
      Ligne2 = Ligne2 DIF_DEUXSONDES("DIF:" + String(diff_deuxsondes)); // Ne va s'ajouter que si on active les deux sondes (désactivation : commenter #define DEUXSONDES)
      break;
    }
    case 9:   // palier 1
    case 10:  // palier 2
    case 11:  // palier 3
    case 12:  // palier 4
    case 13:  // Mash-out
    case 14:  // Ebullition
      {

        display_palier(menu - 8, &Ligne1, &Ligne2);
        break;
      }

    case 15: // refroidissement
      {
        Ligne1 = "Refroidissement";
        theta_objectif = 0;
        myTempo[0] = (minutes - cooling);
        Ligne2 = "T:" + String(((int)theta_mesure)) + degre + " / " + String(myTempo[0]) + " min";
        break;
      }

    case 109:   // Demander si on veut recommencer le palier en cours
    case 110:
    case 111:
    case 112:
    case 113:
    case 114:
    {
        restart_palier(menu - 108, &Ligne1, &Ligne2);
        break;
    }
    
    case 115:   // Recommencer le refroidissement
    {
        Ligne1 = "RESTART REFROID?";
        Ligne2 = "SEL=OUI autre=NO";
        break; 
    }
    
    case 209:   // Demander si on veut passer le palier en cours
    case 210:
    case 211:
    case 212:
    case 213:
    case 214:
      {
        pass_palier(menu - 208, &Ligne1, &Ligne2);
        break;
      }
    case 215:   // On a demander à passer le refroidissement : fin du brassage
      {
        Ligne1 = "FIN DU BRASSAGE ?";
        Ligne2 = "SEL=OUI autre=NO";
        break;
      }
    default :
      {
        Ligne1 = "Erreur de menu";
        Ligne2 = "menu " + String(menu);
        break;
      }

  }
  lcd.setCursor(0, 0);
  lcd.print(Ligne1);
  lcd.setCursor(0, 1);
  lcd.print(Ligne2);

} //************************************************************************************************************************************************************************FIN LCD

// read the buttons
int read_LCD_buttons() //cette fonction renvoie la touche appuyée
{
  adc_key_in = analogRead(0);      // read the value from the sensor
  // my [Mark Bramwell's] buttons when read are centered at these valies: 0, 144, 329, 504, 741
  // we add approx 50 to those values and check to see if we are close
  // DEBUGPRINTLN(String(adc_key_in));
  if (adc_key_in > 1000) return btnNONE; // We make this the 1st option for speed reasons since it will be the most likely result
  if (adc_key_in < 50)   return btnRIGHT;
  if (adc_key_in < 195)  return btnUP;
  if (adc_key_in < 380)  return btnDOWN;
  if (adc_key_in < 555)  return btnLEFT;
  if (adc_key_in < 790)  return btnSELECT;
  return btnNONE;  // when all others fail, return this...
} // fin read the buttons

float gettemp()
{
  myPID.Compute(); //on Lance un calcul du PID. le calcul se fait en même temps que le rafraichissement des variables de température

  float ttt;

// Version Originale avec DAC
  #if defined(DAC)
        int16_t therm, therm2;
        float ttt2;       // Le DAC à 2 entrées reliées à la même sonde. Pour déparasiter, on fait une moyenne des deux lectures
        therm = ads1115.readADC_SingleEnded(THERM_PIN);
        therm2 = ads1115.readADC_SingleEnded(THERM_PIN2);
        therm -= OFFSET_PROBE;
        therm2 -= OFFSET_PROBE;
        ttt = float(therm);
        ttt2 = float(therm2);
        ttt = ttt / COEF_PROBE;
        ttt2 = ttt2 / COEF_PROBE;
        ttt = (ttt + ttt2) / 2;

// Version avec deux sondes
  #elif defined(DEUXSONDES)
        float ttt2; 
        DS18B20.requestTemperatures();
        ttt = DS18B20.getTempCByIndex(0);
        DS18B202.requestTemperatures(); // récupérer la température de la deuxième sonde
        ttt2 = DS18B202.getTempCByIndex(0);

        if (ttt < 1) {
          if (ttt2 < 1) {    // les 2 sondes de températures sont dans les choux (on lit 0° ou -127°)(sauf si vous vivez au Pôle Nord) => on prend la dernière valeur dispo
            ttt = last_temp;
          }
          // sinon il n'y a que la première sonde qui est à l'ouest => on prend la valeur de la deuxième sonde uniquement
          else {
            ttt = ttt2;              
          }
        }
/*        else if (ttt2 < 1) {
          
        }*/ // Pas besoin : ttt est déjà la valeur que l'on cherche
        // enfin, si les deux sondes sont ok, on fait la moyenne des deux températures
        else if (ttt2 > 0 ) {
          diff_deuxsondes = ttt2 - ttt;
          ttt = (ttt + ttt2) / 2;         // faire la moyenne des deux températures
        }

        // 1 BIP court si jamais la déviation de température est d'au moins 3°C entre les deux sondes
        if ( abs(diff_deuxsondes) >= 3 ) {
              BeepON(100);
        }
        
// Version avec une sonde
  #else
        DS18B20.requestTemperatures(); 
        ttt = DS18B20.getTempCByIndex(0);
        // La sonde est dans les choux (on lit 0° ou -127°) 
        if (ttt < 1) {
           ttt = last_temp;   // on prend la dernière valeur qui était cohérente
        }
  #endif
  
  last_temp = ttt;
  return ttt;
}


void CLS() // efface l'écran et place le curseur au début de la première ligne
{
  lcd.clear();
  lcd.setCursor(0, 0);
}


void horloge() //fonction appelée une fois par seconde . On profite de cette fonction pour certaines fonctionalités actualisées une fois par seconde
{

  secondes_reel++;

  if (palier_atteint) { // on incrémente le compteur minutes toutes les 60s, mais seulement si on a atteint le palier - permet d'éviter les erreurs de mesure de la sonde qui parfois affiche des valeurs farfelues :
                        // si la lecture de la sonde de température donne quelque chose de négatif au moment où on rentre dans ce sous-programme, on pourrait ne pas incrémenter les minutes, et donc faire durer trop longtemps le palier

      secondes++; //on compte le temps écoulé pendant que le palier est en cours
  
      BLINK(); //on fait clignoter une fois par seconde pour montrer que le palier est lancé
  }

  if (theta_objectif == 0) //cas uniquement présent lors du refroidissement
  {
    secondes++;
  }
    
  if (secondes >= 60) {
      secondes = 0;
    
      // Si le menu correpsond à un palier en cours (sauf préchauffage), on incrémente les minutes
      if (menu >= 9) { 
          minutes++;
          DEBUGPRINTLN("Minutes écoulées : : " + String(minutes));
        }
  }
  
  if (theta_mesure > (theta_objectif - hysteresis_Neg) && theta_mesure < (theta_objectif + hysteresis_Pos)) // si on est compris entre les deux valeurs d'hysteresis, on est dans la bonne plage de température. on réduit la force du PID
  {
    Kp = P_weak; // multiplicateur de l'erreur différentielle de température.
    Ki = I_weak; //coef de correction intégrale
    Kd = D_weak; //coef de dérivée
   
  }
  else {
    Kp = P_strong; // multiplicateur de l'erreur différentielle de température. 1° d'écart = Kp% de chauffe
    Ki = I_strong; //coef de correction intégrale
    Kd = D_strong; //coef de dérivée
  }
  myPID.SetTunings(Kp, Ki, Kd);
}

void lecture()
{
  theta_mesure = gettemp();
  if (theta_mesure > (theta_objectif - hysteresis_Neg) && theta_mesure < (theta_objectif + hysteresis_Pos)) {
    theta_PID = theta_mesure; // si on est proche de la température de consigne, on supprime l'offset de sécurité
    palier_atteint = true;    // on annonce que le palier est atteint : c'est lui qui va déterminer si on va incrémenter les minutes pour le palier en cours, pour éviter les erreurs de mesure de la sonde
  }
  else {
    // Si on n'est pas à l'ébullition...
    if (menu != 14) {
      theta_PID = theta_mesure + PID_OFFSET;
    } else {
      theta_PID = theta_mesure;
    }

  }
}

void start_brew() {
    digitalWrite(ledPin, LOW);
    myPID.SetMode(AUTOMATIC);

    //////////////////////////////////
    // on compte le temps que va prendre le programme
    startprog = 1;
    total_time = 0;
    for (int i = 1; i < 7; i++) {
      total_time += myTempo[i];
      DEBUGPRINTLN("Temps palier[" + String(i) + "] : " + String(myTempo[i]));
    }
    DEBUGPRINTLN("Temps total prévu : " + String(total_time));
    CLS();           // move to position 0 on the first line
    lcd.print("    BRASSAGE    ");
    lcd.setCursor(0, 1);           // move to position 0 on the second line
    lcd.print("    IMMINENT !  ");
    delay(800);
    CLS();
    lcd.print("Duree prevue :  ");
    lcd.setCursor(0, 1);           // move to position 0 on the second line
    lcd.print(total_time);
    lcd.print(" min");
    delay(1000);

    digitalWrite(ledPin, LOW);

    menu = 8; //saute au menu suivant (départ programme)
    CLS();
    LCD_upd();
    
    return;
}

// +++++++++++++++++++++++++++++++++++++++++       GESTION DES MENUS                   ++++++++++++++++++++++++++++++++++++++++++++++
void sel_menu()
{
  #ifdef BIPPEUR
    if (beep_duration > 0) {
      beep_duration--;
    }
    else {
      BeepOFF();
    }
  #endif
  
  byte m;
  
  lcd_key = read_LCD_buttons();  // read the buttons

  if (menu >= 100) { // gestion des sauts de programme en cas d'annulation la valeur est >100  en cas de saut menu > 200
    switch (lcd_key)
    {
      case btnNONE:
        {
          break;
        }

      case btnSELECT:
        {
          switch (menu)
          {
            case 108:
              {
                CLS();
                lcd.print("saut erreur 108");
                menu = 8;
                delay(150);
                break;
              }
            case 109: //on a demandé à recommencer l'étape palier 1
            case 110: //on a demandé à recommencer l'étape palier 2
            case 111: //on a demandé à recommencer l'étape palier 3
            case 112: //on a demandé à recommencer l'étape palier 4
            case 113: //on a demandé à recommencer l'étape rincage mash out
            case 114: //on a demandé à recommencer l'étape ébu
            case 115: //on a demandé à recommencer l'étape refroidissement/whirlpool
            {
                m = menu - 108;
                CLS();
                annuler = -1; // on était au menu X (menu - 109) on retourne au X-1 (menu - 110)
                minutes = 0;  //avant le palier 1 (donc l'étape de préchauffage) le temps écoulé était à 0, et
                              //avant le palier 2 (donc le palier 1) le temps écoulé était aussi égal à 0

                myTempo[0] = 0;
                

                for (int i = m; i >= 2; i--) { // on ne réinitialise les minutes que pour les paliers > 1
                    minutes += myTempo[(i - 1)];    // on rajoute les durées des paliers précédents uniquement, car on recommence au début du palier m
                }
                cooling = minutes;                
                delay(150);
                palier_atteint = false;   // on ne sait jamais, on va prétendre que la température du palier n'a pas été atteinte
                break;
              }

            case 208: //on a demandé à sauter l'étape préchauffage (ne devrait pas arriver)
              {
                CLS();
                lcd.print("saut erreur 208");
                menu = 8;
                delay(1000);
                BeepON(50);
                LCD_upd();
                break;
              }

            case 209: //on a demandé à sauter l'étape palier 1
            case 210: //on a demandé à sauter l'étape palier 2
            case 211: //on a demandé à sauter l'étape palier 3
            case 212: //on a demandé à sauter l'étape palier 4
            case 213: //on a demandé à sauter l'étape mash out
            case 214: //on a demandé à sauter l'étape ebullition
            {
                m = menu - 208;
                minutes = 0;
                for (int i = m; i >= 1; i--) { // on ne réinitialise les minutes que pour les paliers > 1
                    minutes += myTempo[i];    // on rajoute les durées des paliers précédents et du palier en cours, car on va recommencer au début du palier m + 1
                }                
                myTempo[0] = 0;
                jump = 0;
                annuler = 1;
                cooling = minutes; // sauter l'étape ébullition (cooling sera mis à jour de toute façon quand on sera à la fin de l'ébullition, pas de problème à l'écraser maintenant)
                delay(150);
                palier_atteint = false;   // on ne sait jamais, on va prétendre que la température du palier n'a pas été atteinte
                break;
              }
            case 215:   // on a fini le brassage : on demande à "sauter" le refroidissement/whirlpool pour finir dans un état permettant d'arrêter proprement l'automate
            {
                T.stop(logRecord_ID); // On arrête le timer lié à l'écriture sur la carte SD, pour éviter qu'une écriture impromptue ne soit en cours quand on débranche l'Arduino
                logRecord();  // On appelle pour la dernière fois l'écriture sur la carte SD, pour logger la fin du brassage (dernier événement du log)
                CLS();
                lcd.print("Fin de BRASSAGE");
                lcd.setCursor(0, 1);
                lcd.print("!!! HAVE FUN !!!");
                delay(800);
                while(1);   // Arrêt propre : plus aucune action ne sera réalisée par la suite (donc pas d'erreur d'écriture sur carte SD par exemple)
                break;              
            }
              break;
          } //fin switch menu
        }//fin bouton select

      case btnUP: // on annule le saut
      case btnDOWN: // on annule le saut
      case btnLEFT: // on annule le saut
      case btnRIGHT: // on annule le saut
      {
          CLS();
          jump = 0;
          delay(500);
          if (menu > 199)
          {
            menu -= 200;
          }
          else {
            menu -= 100;
          }
          break;
        }
        break;
    }// fin switch lcd key
  } else { //Si on n'a pas demandé de saut

    // gestion des menus
    switch (lcd_key)               // depending on which button was pushed, we perform an action
    {

      case btnNONE:
        {
          break;
        }

      case btnUP:
        {

          if (startprog == 0) //si on est encore en phase de programation, on peut naviguer du menu 0 à 7
          {
            if (menu < 7)
            {
              menu++;

              delay(150);
              LCD_upd();
            }
          }
          else { //le programme est lancé
            if (menu > 7) // On demande à sauter l'étape en cours
            {


              if (menu >= 9 && menu < 16) //si on est au menu 9 ou + on considère que c'est un saut de programme (y inclus pour le refroidissement, pour la fin du programme)
              {
                // small_Beep = 1;
                BeepON(200);

                menu += 200;
              }
              if (menu == 8) menu ++; //si on est dans le menu 8 (préchauffe) , le saut est autorisé sans condition
              delay(150);
              LCD_upd();
            }

          }//fin else
          break;
        }
      case btnDOWN:
        {

          if (startprog == 0) //si on est encore en phase de programation, on peut naviguer du 7 à 0
          {
            if (menu > 0) {
              menu--;
              delay(150);
              LCD_upd();
            }
          }
          else {
            if (menu > 8) { //si on est en phase brassage (a partir de menu 9)

              // DEMANDER SI ON SOUHAITE RECOMMENCER L'ETAPE EN COURS
              // small_Beep = 1;
              BeepON(200);
              
              menu += 100;
              delay(150);
              LCD_upd();
            }

          }

          break;
        }
      case btnLEFT:
        {
          //faire un switch menu
          switch (menu)
          {
            case 0: //menu température de préchauffe
            case 1: //menu réglage palier 1
            case 2: //menu réglage palier 2
            case 3: //menu réglage palier 3
            case 4: //menu réglage palier 4
            case 5:  //menu réglage Mash out - rinçage
            case 6: // ébullition
            {
                if (submenu == 0) {
                  if (myTemperatures[menu] > 20) myTemperatures[menu] -= 0.5;
                }
                else {
                  if (myTempo[menu] > 0) myTempo[menu]--;
                }
                LCD_upd();
                break;
              }

            case 7: //sauvegarde
              {
                break;
              }
            case 8: //préchauffe
            case 9: //palier 1
            case 10://palier 2
            case 11://palier 3
            case 12://palier 4
            case 13://rinçage - mash-out
            case 14://ébullition
              {
                m = menu - 8;
                DEBUGPRINTLN("Valeur submenu pour le menu " + String(m) + " : " + String(submenu));
                if (submenu == 0) {
                  if (myTemperatures[m] > 20) myTemperatures[m] -= 0.5;
                }
                else {
                  if (myTempo[m] > 2) myTempo[m]--;
                }
                LCD_upd();
                break;
              }

            case 15://mode manuel
            case 16://mode erreur
              {
                break;
              }
          }
          break;
        }
      case btnRIGHT:
        {
          //faire un switch menu
          switch (menu)
          {
            case 0: //menu température de préchauffe
            case 1: //menu réglage palier 1
            case 2: //menu réglage palier 2
            case 3: //menu réglage palier 3
            case 4: //menu réglage palier 4
            case 5:  //menu réglage Mash out - rinçage
            case 6: // ébullition
            {
                if (submenu == 0) {
                  if (myTemperatures[menu] <= 110) myTemperatures[menu] += 0.5;
                }
                else {
                  myTempo[menu]++;
                }
                LCD_upd();
                break;
                
              }

            case 7: // On a demandé à ne pas faire la sauvegarde
              {
                start_brew();
                break;
              }
            case 8: //préchauffe
            case 9: //palier 1
            case 10://palier 2
            case 11://palier 3
            case 12://palier 4
            case 13://rinçage mash out
            case 14://ébullition
              {
                m = menu - 8;
                DEBUGPRINTLN("Valeur submenu pour le menu " + String(m) + " : " + String(submenu));
                if (submenu == 0) {
                  if (myTemperatures[m] < 110) myTemperatures[m] += 0.5;
                }
                else {
                  myTempo[m]++;
                }
                LCD_upd();
                break;
              }

            case 15://mode manuel
            case 16://mode erreur
              {
                break;
              }
            default:
            {
              CLS();
              lcd.print("Erreur de menu");
              lcd.setCursor(0, 1);           // move to position 0 on the second line
              lcd.print("menu ");
              lcd.print(menu);
              break;
            }
          }
          break;
        }
      case btnSELECT:
        {
          switch (menu)
          {

            case 1: // menu paliers 1 à 4
            case 2:
            case 3:
            case 4:
            case 6: // ébullition
              {
                submenu = !submenu;
                LCD_upd();
                break;
              }

            case 0: //menu température de préchauffe
            case 5: //menu rinçage
              {
                LCD_upd();
                break;
              }

            case 7: // sauvegarde
              {

                digitalWrite(ledPin, HIGH);
                // sauvegarde des variables dans l'EEPROM si défini :
                #if (defined(EEPROMRW))
                    EEPROM.updateLong(0, myTempo[1]);
                    EEPROM.updateLong(7, myTempo[2]);
                    EEPROM.updateLong(15, myTempo[3]);
                    EEPROM.updateLong(23, myTempo[4]);
                    EEPROM.updateLong(31, myTempo[6]);
                    EEPROM.updateFloat(39, myTemperatures[1]);
                    EEPROM.updateFloat(47, myTemperatures[2]);
                    EEPROM.updateFloat(55, myTemperatures[3]);
                    EEPROM.updateFloat(63, myTemperatures[4]);
                    EEPROM.updateFloat(71, myTemperatures[6]);
                    EEPROM.updateFloat(79, myTemperatures[5]);
                    EEPROM.updateFloat(87, myTemperatures[0]);
                #endif
                
                sauve_param();

                myFile = SD.open(datalogFile, FILE_WRITE);
                DEBUGPRINTLN("Write on " + datalogFile);
                if (myFile) {
                  myFile.print("----GraiN.Master----\n");
                  myFile.print(version);
                  myFile.print("\nTempérature de départ : ");
                  myFile.print(myTemperatures[0]);
                  myFile.print("°C\n");

                  for (int i = 1; i < 7; i++) {
                    myFile.print(myPaliers[0][i]);
                    myFile.print(myTempo[i]);
                    myFile.print(" min - ");
                    myFile.print(myTemperatures[i]);
                    myFile.print("°C\n");
                  }

                  myFile.print("\n\n");
                  myFile.print("Temps (s),Température cible °C,Température °C,PID (0-255)\n");
                  myFile.close();
                }
                
                DEBUGPRINTLN("----GraiN.Master----\n");
                DEBUGPRINTLN(version);
                DEBUGPRINTLN("\nTempérature de départ : ");
                DEBUGPRINTLN(myTemperatures[0]);
                DEBUGPRINTLN("°C\n");
                #ifdef DEBUG
                for (int i = 1; i < 7; i++) {
                  DEBUGPRINTLN(myPaliers[0][i]);
                  DEBUGPRINTLN(String(myTempo[i]) + " min - " + String(myTemperatures[i]) + "°C");
                }
                #endif                
                lcd.home();           // move to position 0 on the first line
                lcd.print("    PROGRAMME   ");
                lcd.setCursor(0, 1);           // move to position 0 on the second line
                lcd.print("   SAUVEGARDE ! ");


                delay(600);
                start_brew();
                break;

              }
            case 8: //palier 1
            case 9: //palier 2
            case 10: //palier 3
            case 11://palier 3
            case 12://palier 4
            case 13://rinçage - mash-out
            case 14://ébullition
              {
                submenu = !submenu;   // Permet de modifier la durée ou la température du palier en cours
                break;
              }

            default:
              {
                CLS();
                lcd.print("Erreur de menu");
                lcd.setCursor(0, 1);           // move to position 0 on the second line
                lcd.print("menu ");
                lcd.print(menu);
                break;
              }

            break;
          } //fin switch menu
          break;
        } //fin buton select


        default:
          {
            CLS();
            lcd.print("Erreur de menu");
            lcd.setCursor(0, 1);           // move to position 0 on the second line
            lcd.print("menu ");
            lcd.print(menu);
            break;
          }

        break;
    } //fin switch lcd key
  } //fin du else (pas de saut)
}

void regle_chauffe()
{

  if (tx_chauffe >= 250)
  {
    // Si on est dans un des paliers de chauffe, on veut une courbe de température douce, qui gagne environ 1° toutes les minutes (pour éviter de violenter notre moût), le TRIAC à fond va beaucoup plus fort
    // On ne modifie pas tx_chauffe pour éviter tout effet de bord
    // Ce n'est valide que si le palier n'a pas été atteint. Si ce n'est pas le cas, c'est qu'on a eu une brusque chute de température lors du palier et qu'il faut au plus vite revenir à notre température !
    if (menu >= 9 && menu <= 13 && !palier_atteint) {  // menu 8 = préchauffage, menu 9 = palier 1, menu 13 = mash-out, menu 14 = ébullition
      PROPORTIONNAL();
      Triac_pilot(Power_out(220));
    }
    // Si on est autre part (préchauffage ou ébullition), on y va à fond
    else {
      FULL_ON();
    }
  }
  else {

    if (tx_chauffe <= 5)
    {
      FULL_OFF();
    }
    else
    {
      PROPORTIONNAL();
      Triac_pilot(Power_out(tx_chauffe));
    }
  }

}
//Interrupt Service Routines

void zeroCrossingInterrupt() { //zero cross detect
  TCCR1B = 0x04; //start timer with divide by 256 input
  TCNT1 = 0;   //reset timer - count from zero
}

ISR(TIMER1_COMPA_vect) { //comparator match
  digitalWrite(GATE, HIGH); //set triac gate to high
  TCNT1 = 65536 - PULSE;    //trigger pulse width
}

ISR(TIMER1_OVF_vect) { //timer1 overflow
  digitalWrite(GATE, LOW); //turn off triac gate
  TCCR1B = 0x00;          //disable timer stopd unintended triggers
}

void Triac_pilot(int conduction_time) { // pilote le triac
  if (conduction_time > HALF_WAVE) conduction_time = HALF_WAVE;
  OCR1A = conduction_time;     //set the compare register brightness desired.
}


int Power_out(int Power) //calcule le temps de conduction nécessaire pour obtenir la puissance demandée. Power va de 0 à 255
{

  int CT;
  CT = 540 - (Power * 2);
  return CT;
}
void FULL_ON()
{
  // set up zero crossing interrupt
  detachInterrupt(digitalPinToInterrupt(2));  // équivalent à 0
  digitalWrite(GATE, HIGH);
  //IRQ0 is pin 2
}
void FULL_OFF()
{
  // set up zero crossing interrupt
  detachInterrupt(digitalPinToInterrupt(2));  // équivalent à 0
  digitalWrite(GATE, LOW);
  //IRQ0 is pin 2
}

void PROPORTIONNAL()
{
  attachInterrupt(digitalPinToInterrupt(2), zeroCrossingInterrupt, CHANGE);
}

void logRecord()
{
  byte numpalier = 0;
  #ifdef DEFIL
    defilement = !defilement;    // Position du défilement / 1 = température et température à atteindre / 0 = température et temps restant / s'échange à chaque écriture sur la carte SD, donc toutes les 10 s environ
  #endif
  if (menu > 200)
    numpalier = menu - 208;
  else if (menu > 100)
    numpalier = menu - 108;
  else if (menu <= 7)
    numpalier = 0;
  else numpalier = menu - 8;
  digitalWrite(ledPin, 1);
  File dataFile = SD.open(datalogFile, FILE_WRITE);
  if (dataFile) {
    dataFile.print(secondes_reel);
    dataFile.print(";");
    if (numpalier != 7)
      dataFile.print(myPaliers[0][numpalier]);
    else dataFile.print("Refroidissement");
    dataFile.print(";");
    dataFile.print(theta_objectif);
    dataFile.print(";");
    dataFile.print(theta_mesure);
    dataFile.print(";");
    dataFile.print(tx_chauffe);
    dataFile.print('\n');
    dataFile.close();
  }
  digitalWrite(ledPin, 0);
  DEBUGPRINTLN("Secondes réelles depuis le début : " + String(secondes_reel));
  DEBUGPRINTLN("Theta_Objectif : " + String(theta_objectif));
  DEBUGPRINTLN("Theta_mesure : " + String(theta_mesure));
  DEBUGPRINTLN("Taux chauffe (PID) : " + String(tx_chauffe));
  CLS();                      // On ne refresh l'écran que toutes les 10 secondes, pour éviter l'effet stroboscopique
  
}


void sauve_param()
{
  digitalWrite(ledPin, 1);

  if (SD.exists("setup.grm")) {
    SD.remove("setup.grm");
  }

  File dataFile = SD.open("setup.grm", FILE_WRITE);
  if (dataFile) {

    // Certaines informations ne sont présentes que pour le DAC
    #if defined(DAC)
        dataFile.print("THERMPIN2");
        dataFile.print("=");
        dataFile.print(THERM_PIN2);
        dataFile.print(",");
        dataFile.print("THERMPIN1");
        dataFile.print("=");
        dataFile.print(THERM_PIN);
        dataFile.print(",");
        dataFile.print("OFSETPRBE");
        dataFile.print("=");
        dataFile.print(OFFSET_PROBE);
        dataFile.print(",");
        dataFile.print("COEFPROBE");
        dataFile.print("=");
        dataFile.print(COEF_PROBE);
        dataFile.print(",");
    #endif
    dataFile.print("hysterPos");
    dataFile.print("=");
    dataFile.print(hysteresis_Pos);
    dataFile.print(",");
    dataFile.print("hysterNeg");
    dataFile.print("=");
    dataFile.print(hysteresis_Neg);
    dataFile.print(",");
    dataFile.print(myPaliers[0][0]);
    dataFile.print(myTemperatures[0]);
    dataFile.print(",");
    for (int i = 1; i < 7; i++) {
      dataFile.print(myPaliers[0][i]);
      dataFile.print(myTemperatures[i]);
      dataFile.print(",");
      dataFile.print(myPaliers[1][i] + "TEMPO");
      dataFile.print("=");
      dataFile.print(myTempo[i]);
      dataFile.print(",");
    }

    dataFile.print("Pstrong00");
    dataFile.print("=");
    dataFile.print(P_strong);
    dataFile.print(",");
    dataFile.print("Istrong00");
    dataFile.print("=");
    dataFile.print(I_strong);
    dataFile.print(",");
    dataFile.print("Dstrong00");
    dataFile.print("=");
    dataFile.print(D_strong);
    dataFile.print(",");
    dataFile.print("Pweak0000");
    dataFile.print("=");
    dataFile.print(P_weak);
    dataFile.print(",");
    dataFile.print("Iweak0000");
    dataFile.print("=");
    dataFile.print(I_weak);
    dataFile.print(",");
    dataFile.print("Dweak0000");
    dataFile.print("=");
    dataFile.print(D_weak);
    dataFile.print(",");
    dataFile.print("LEDPIN000");
    dataFile.print("=");
    dataFile.print(ledPin);
    dataFile.print(",");
    dataFile.print("GATE00000");
    dataFile.print("=");
    dataFile.print(GATE);
    dataFile.print(",");
    dataFile.print("PULSE0000");
    dataFile.print("=");
    dataFile.print(PULSE);
    dataFile.print(",");
    dataFile.print("HALFWAVE0");
    dataFile.print("=");
    dataFile.print(HALF_WAVE);
    dataFile.print(",");
    #if (defined(BIPPEUR))
        dataFile.print("BEEPPIN00");
        dataFile.print("=");
        dataFile.print(Beep_PIN);
        dataFile.print(",");
    #endif
    dataFile.print("PIDOFFSET");
    dataFile.print("=");
    dataFile.print(PID_OFFSET);
    dataFile.print(",");
    #if !(defined(DAC))
      dataFile.print("ONEWIRE00");
      dataFile.print("=");
      dataFile.print(ONE_WIRE_BUS);
      dataFile.print(",");
    #endif
    #if (defined(DEUXSONDES))
      dataFile.print("ONEWIRE02");
      dataFile.print("=");
      dataFile.print(ONE_WIRE_BUS2);
      dataFile.print(",");
    #endif
    dataFile.close();
  }
  digitalWrite(ledPin, 0);
}
