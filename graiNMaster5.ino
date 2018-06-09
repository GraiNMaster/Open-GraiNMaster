/*******************************************************
  GraiNMaster
  système de contrôle de température de la cuve
  Débuté le 24/11/2015 par Nicolas Morival
  Testé et compilé sur Arduino IDE v1.6.8
*******************************************************/

#define version  " --V1.003-- " 

/*
v1.000 le 14/05/2016
V1.002 le 08/10/2016
nettoyage du code, suppression des morceaux de code inutiles (relatifs à la SD)
suppression du paramètre sur SD detect : cette broche est une interruption fixée par le hardware
ajout de la possibilité d'utiliser une sonde DS18B20
v1.003 le 09/11/2017
correction de bugs sur l'initialisation de l'eeprom (bug relevé par artiche)
mise en fonction du buzzer (suggestion artiche)
*/


//bibliothèques
#include <SPI.h> 
#include <SD.h>
#include <PID_v1.h> //gestion de l'algo PID
#include <avr/pgmspace.h> 
#include <EEPROMex.h>
#include <EEPROMVar.h>  // gestion des enregistrrements en EEPROM
#include "Timer.h" // gestion des timers
#include <LiquidCrystal.h> //gestion du LCD
#include <avr/io.h> 
#include <avr/interrupt.h> //gestion des interruptions
#include <Wire.h>
#include <Adafruit_ADS1015.h> // gestion du CAN 16 bits
#include <OneWire.h>
#include <DallasTemperature.h>


int DETECT = 2 , GATE = 3; //Broches utilisées pour le pilotage du triac -> detect : impulsion du passage au point zéro, gate : gachette du triac
int PULSE = 4  ; //triac gate pulse : largeur de l'impulsion pour activer le triac, en nombre de cycles d'horloge
int HALF_WAVE = 560;// //nombre de "tics" d'horloge à chaque demi onde 50Hz . 625 théoriques, mais 560 réels .
int probe = 2; // probe = 0 si on utilise le dac externe, probe = 2 pour une sonde DS18B20
int ONE_WIRE_BUS = 30;  // DS18B20 pin
#define BeepONState  1
#define BeepOFFState  0

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);

// constantes pour les touches reliées au module LCD
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

// pour la lecture de la sonde
int THERM_PIN2  = 1 ;// entrée n°1 du dac 16bits
int THERM_PIN  = 3 ; // entrée n°3 du dac 16bits
//on lit sur deux entrées dont les filtres passe bas sont légèrement différents, afin de faire une moyenne et augmenter la précision

int OFFSET_PROBE  = 138; // correction constante de la température 
int COEF_PROBE = 260 ; // correction proportionelle 
// Température calculée = (mesure du DAC  - OFFSET_PROBE) / COEF_PROBE

//valeurs de l'hysteresis pour passage en PID fin ou grossier
int hysteresis_Pos =  1;
int hysteresis_Neg  = 1;

// déclaration de l'ADC 16 bits
Adafruit_ADS1115 ads1115(0x48);	// ADC16 bits addresse 0x48

//  pins utilisées par le LCD
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// valeurs utilisées pour configurer le LCD
int backLight   = 10;    // LCD Panel Backlight LED connected to digital pin 10
int lcd_key     = 0;
int adc_key_in  = 0;
int ledPin = 13;
int ledState = LOW;

// variables du programme
volatile unsigned int menu = 0;
unsigned char submenu = 0;
unsigned char startprog = 0; //marqueur pour indiquer que le programme s'éxécute
unsigned char jump = 0; //pour indiquer qu'on va sauter un menu

char annuler = 0; //pour annuler les opérations 0 = fonctionnement normal  / -1 on recule d'une étape / 1 on avance d'une étape

unsigned char secondes = 0;
unsigned long secondes_reel = 0; //temps écoulé depuis le début du programme, utilisé pour le data log
unsigned int minutes = 0; //minutes écoulées
unsigned long total_time = 0; //cumul des temps programme
unsigned long temps_pause = 0; //si le programme est en pause, compte le temps écoulé
unsigned long tempo1 = 0; // palier empâtage céréales non maltées
unsigned long tempo2 = 15; // palier protéinique
unsigned long tempo3 = 35; // palier de sacharification B amylase
unsigned long tempo4 = 30; // palier de sacharification A amylase
unsigned long tempo5 = 70; // ébullition
unsigned long tempo_A = 0; // pour l'affichage temporaire

float thetastart = 70; //température de préchauffe de la cuve
float theta1 = 40.25; //température empâtage céréales non maltées
float theta2 = 52.5; //température empâtage céréales non maltées
float theta3 = 62.0; //température de sacharification B amylase
float theta4 = 68.75; //température de sacharification A amylase
float theta5 = 100.0; //température d'ébullition
float thetaMO = 75.75; //température de mash out

unsigned long cooling = 0; // compteur du temps de refroidissement

float temp; //temporaire

int therm; //variable pour le relevé de température

double chauffe_reel = 0, tx_chauffe, theta_mesure, theta_PID, theta_objectif = 20;

//reglages par défaut du PID
// Ces constantes sont correctes pour une cuve de 27L 2500W non isolée type kitchen chef

double P_strong = 100;
double I_strong = 0.0;
double D_strong = 16;

double P_weak = 80;
double I_weak = 0.02;
double D_weak = 8;

double Kp = P_strong; // multiplicateur de l'erreur différentielle de température.
double Ki = I_strong; //coef de correction inverse
double Kd = D_strong; //coef de dérivée

float PID_OFFSET = 1; // on ruse le PID pour lui faire décaller la température cible d'une valeur constante par exemple +1° donc si l'utilisateur vise 62° et que la température mesurée est de 61° le PID croit qu'il a atteint les 62° et coupe la chauffe. Ainsi l'overshoot est limité.
// en théorie, avec les paramètres réglés au top, ce paramètre peut être remis à zéro.

PID myPID(&theta_PID, &tx_chauffe, &theta_objectif, Kp, Ki, Kd, DIRECT); //déclaration du PID

//pin du bipeur
int Beep_PIN = 40;
// variables utilisées pour faire marcher le bippeur
unsigned char Beep_state = 0;
unsigned char Beep = 0;
unsigned char small_Beep = 0;

//réglages des timers
Timer T;

//déclaratioon du fichier pour la SD
File myFile;
int check_data = 0;

void setup()
{
  //config pour SD
  String inString = "";
  float variable;

 pinMode(53, OUTPUT); //la pin 53 est normalement attribuée à la fonction SPI Slave Select (SS)Permet d'activer ou désactiver le périphérique
// Suivant les modules SD, il peut être nécessaire de commenter cette ligne (c'est le cas de mon module même si c'est pas logique)



  // ECRAN DE PRESENTATION ----------------------------
  lcd.begin(16, 2);     // déclaration de la bibliothèque LCD avec 16 caractères, 2 lignes
  lcd.home();           // met le curseur au début


  // création des caractères spéciaux
  byte degree[8] = {
    B00111,
    B00101,
    B00111,
    B00000,
    B00000,
    B00000,
    B00000,
  };
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

  lcd.createChar(0, degree);
  lcd.createChar(1, beer1);
  lcd.createChar(2, beer2);
  lcd.setCursor(0, 0);
  lcd.print("- GraiN.Master -"); 
  lcd.setCursor(0, 1);     
  lcd.write(byte(1));
  lcd.write(byte(2));
  lcd.print(version);
  lcd.write(byte(1));
  lcd.write(byte(2));

  delay(1000);


  //Effet de scrolling sympa
  for (int positionCounter = 0; positionCounter < 16; positionCounter++) {
    // scroll one position left:
    lcd.scrollDisplayLeft();
    // wait a bit:
    delay(100);
  }

  CLS();

  // set up des pins
  pinMode(DETECT, INPUT);     //détection du passage au pont zéro / zero cross detect
  digitalWrite(DETECT, HIGH); // enable pull-up resistor
  pinMode(GATE, OUTPUT);      //triac gate control
  pinMode(Beep_PIN, OUTPUT);
  BeepOFF();
  // démarrage du DAC 16 bits
  ads1115.begin();

  // set up Timer1
  //(see ATMEGA 328 data sheet pg 134 for more details)
  OCR1A = 25;      //initialize the comparator
  TIMSK1 = 0x03;    //enable comparator A and overflow interrupts
  TCCR1A = 0x00;    //timer control registers set for
  TCCR1B = 0x00;    //normal operation, timer disabled


  myPID.SetOutputLimits(0, 255);

  EEPROM.setMaxAllowedWrites(48);

  therm = gettemp();

 // pinMode(10, OUTPUT);
  lcd.home();           // move cursor to beginning of line "0"
  lcd.print("initialise  SD  "); // print a simple message
  delay(600);
  if (!SD.begin(4)) {
    lcd.home();           // move cursor to beginning of line "0"
    lcd.print("ERREUR SD CARD "); // print a simple message
    BeepON();
    delay(800);
    BeepOFF();
    //  return;
  }



  lcd.home();           // move cursor to beginning of line "0"
  lcd.print(" Lecture  param "); // print a simple message
  lcd.setCursor(0, 1);           // move to position 11 on the second line
  lcd.print("                ");



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
          //          Serial.print("THERMPIN2 trouve. valeur =");
          //         Serial.println(inString.toInt());
          THERM_PIN2 = inString.toInt();
          check_data++;

        }

        if (inString.startsWith("THERMPIN1"))
        {
          inString.remove(0, 10); // Remove 10 characters starting at index=0
          //         Serial.print("THERMPIN1 trouve. valeur =");
          //         Serial.println(inString.toInt());
          THERM_PIN = inString.toInt();
          check_data++;

        }
        if (inString.startsWith("OFSETPRBE"))
        {
          inString.remove(0, 10); // Remove 10 characters starting at index=0
          //         Serial.print("OFSETPRBE trouve. valeur =");
          //         Serial.println(inString.toInt());
          OFFSET_PROBE = inString.toInt();
          check_data++;
        }

        if (inString.startsWith("COEFPROBE"))
        {
          inString.remove(0, 10); // Remove 10 characters starting at index=0
          COEF_PROBE = inString.toInt();
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
        if (inString.startsWith("thetastrt"))
        {
          inString.remove(0, 10); // Remove 10 characters starting at index=0

          //          Serial.println(inString.toFloat());
          thetastart = inString.toFloat();
          check_data++;

        }
        if (inString.startsWith("theta0001"))
        {
          inString.remove(0, 10); // Remove 10 characters starting at index=0

          theta1 = inString.toFloat();
          check_data++;

        }
        if (inString.startsWith("tempo0001"))
        {
          inString.remove(0, 10); // Remove 10 characters starting at index=0
          tempo1 = inString.toInt();
          check_data++;

        }
        if (inString.startsWith("theta0002"))
        {
          inString.remove(0, 10); // Remove 10 characters starting at index=0
          theta2 = inString.toFloat();
          check_data++;

        }
        if (inString.startsWith("tempo0002"))
        {
          inString.remove(0, 10); // Remove 10 characters starting at index=0
          tempo2 = inString.toInt();
          check_data++;

        }
        if (inString.startsWith("theta0003"))
        {
          inString.remove(0, 10); // Remove 10 characters starting at index=0
          theta3 = inString.toFloat();
          check_data++;

        }
        if (inString.startsWith("tempo0003"))
        {
          inString.remove(0, 10); // Remove 10 characters starting at index=0
          tempo3 = inString.toInt();
          check_data++;

        }
        if (inString.startsWith("theta0004"))
        {
          inString.remove(0, 10); // Remove 10 characters starting at index=0
          theta4 = inString.toFloat();
          check_data++;

        }
        if (inString.startsWith("tempo0004"))
        {
          inString.remove(0, 10); // Remove 10 characters starting at index=0
          tempo4 = inString.toInt();
          check_data++;

        }
        if (inString.startsWith("theta0005"))
        {
          inString.remove(0, 10); // Remove 10 characters starting at index=0
          theta5 = inString.toFloat();
          check_data++;

        }
        if (inString.startsWith("tempo0005"))
        {
          inString.remove(0, 10); // Remove 10 characters starting at index=0
          tempo5 = inString.toInt();

          check_data++;

        }

        if (inString.startsWith("thetaMO00"))
        {
          inString.remove(0, 10); // Remove 10 characters starting at index=0
          thetaMO = inString.toFloat();
          check_data++;

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
          Beep_PIN = inString.toInt();
          check_data++;

        }
        if (inString.startsWith("PIDOFFSET"))
        {
          inString.remove(0, 10); // Remove 10 characters starting at index=0
          PID_OFFSET = inString.toInt();
          check_data++;

        }

        if (inString.startsWith("PROBETYPE"))
        {
          inString.remove(0, 10); // Remove 10 characters starting at index=0
          probe = inString.toInt();
          check_data++;

        }
         if (inString.startsWith("ONEWIRE00"))
        {
          inString.remove(0, 10); // Remove 10 characters starting at index=0
          ONE_WIRE_BUS = inString.toInt();
          check_data++;
        }





        inString = "";

      } //fin else


    } //fin while


    CLS();
    lcd.setCursor(0, 0);
    lcd.print(check_data);
    lcd.print("/32");
    lcd.setCursor(0, 1);
    lcd.print("parametres lus ");
    myFile.close();

    pinMode(DETECT, INPUT);     //zero cross detect
    digitalWrite(DETECT, HIGH); //enable pull-up resistor
    pinMode(GATE, OUTPUT);      //triac gate control
    pinMode(Beep_PIN, OUTPUT);
    pinMode(backLight, INPUT); //set backlight pin to input to avoid MCU overcurent on pin 10
 //   pinMode(11, OUTPUT);     // pin11 = MOSI
    pinMode(ledPin, OUTPUT);
    delay(1500);
  }
  else {

    lcd.setCursor(0, 1);
    lcd.print(" sur EEPROM ");
    delay(1500);
    tempo1 = EEPROM.readLong(0);
    tempo2 = EEPROM.readLong(7);
    tempo3 = EEPROM.readLong(15);
    tempo4 = EEPROM.readLong(23);
    tempo5 = EEPROM.readLong(31);
    theta1 = EEPROM.readFloat(39);
    theta2 = EEPROM.readFloat(47);
    theta3 = EEPROM.readFloat(55);
    theta4 = EEPROM.readFloat(63);
    theta5 = EEPROM.readFloat(71);
    thetaMO = EEPROM.readFloat(79);
    thetastart = EEPROM.readFloat(87);
  }

if (isnan(tempo1) || tempo1 < 0 || tempo1 > 300) tempo1 = 0;
if (isnan(tempo2) || tempo2 < 0 || tempo2 > 300) tempo2 = 15;
if (isnan(tempo3) || tempo3 < 0 || tempo3 > 300) tempo3 = 35;
if (isnan(tempo4) || tempo4 < 0 || tempo4 > 300) tempo4 = 30;
if (isnan(tempo5) || tempo5 < 0 || tempo5 > 300) tempo5 = 70;
if (isnan(theta1) || theta1 < 1 || theta1 > 110) theta1 = 40.25;
if (isnan(theta2) || theta2 < 1 || theta2 > 110) theta2 = 52.50;
if (isnan(theta3) || theta3 < 1 || theta3 > 110) theta3 = 62.00;
if (isnan(theta4) || theta4 < 1 || theta4 > 110) theta4 = 68.75;
if (isnan(theta5) || theta5 < 1 || theta5 > 110) theta5 = 95.75;
if (isnan(thetaMO) || thetaMO < 1 || thetaMO > 110) thetaMO = 76;
if (isnan(thetastart) || thetastart < 1 || thetastart > 110) thetastart = 70.75;

  // DEFINITION DES PINs
  pinMode(DETECT, INPUT);     //zero cross detect
  digitalWrite(DETECT, HIGH); //enable pull-up resistor
  pinMode(GATE, OUTPUT);      //triac gate control
  pinMode(Beep_PIN, OUTPUT);
  pinMode(backLight, INPUT); //set backlight pin to input to avoid MCU overcurent on pin 10
  // pinMode(11, OUTPUT);     // pin11 = MOSI sur uno
  pinMode(ledPin, OUTPUT);

  int T1 = T.every(1200, lecture); // constante de temps pour le timer 1 - lecture de la sonde
  int T2 = T.every(128, sel_menu); // constante de temps pour le timer 2 - lecture des touches
  int T3 = T.every(1024, LCD_upd); // timer 3 - mise à jour écran
  int T4 = T.every(1000, horloge); // timer 4 - compte les secondes et minutes
  int T5 = T.every(350, regle_chauffe); //timer 5 - fréquence de mise à jour de la chauffe
  int T6 = T.every(10000, logRecord); //enregiste sur la sd


  lcd.home();           // move to position 0 on the first line
  lcd.print("T de prechauffe ");
  lcd.setCursor(0, 1);           // move to position 0 on the second line
  lcd.print(">T");
  lcd.write(byte(0)); //affiche le caractère °
  lcd.print("C");


  for (int positionCounter = 0; positionCounter < 16; positionCounter++) {
    // scroll one position left:
    lcd.scrollDisplayRight();
    // wait a bit:
    delay(8);
  }

  OCR1A = 500;
  CLS();
  LCD_upd();
  // set up zero crossing interrupt
  attachInterrupt(0, zeroCrossingInterrupt, CHANGE);
  //IRQ0 is pin 2. Call zeroCrossingInterrupt
  //on rising signal

  menu = 0;

}

//Boucle principale ========================================

void loop()
{

  T.update();

  //gestion des saut d'étapes manuels
  if (annuler == - 1)
  {
    menu--; //on était au menu 9 on retourne au 8
    annuler = 0;
  }
  else {
    if (annuler == 1)
    {
      menu++;
      jump = 0; //par sécurité on ne veut pas de double saut au cas ou le passage au palier suivant était prévu au même moment
      annuler = 0;
    }
  }

  //gestion des sauts d'étapes automatiques
  if (jump == 1) {
    menu++;
    jump = 0;

  }


} //fin Boucle principale ===================================



void LCD_upd() // affiche les infos à l'écran *********************************************************
{
  switch (menu)
  {
    case 0:
      {
        CLS();
        lcd.home();           // move to position 0 on the first line
        lcd.print("T de prechauffe ");
        lcd.setCursor(0, 1);           // move to position 0 on the second line


        lcd.print(">T");
        lcd.print(thetastart);
        lcd.write(byte(0)); //affiche le caractère °
        lcd.print("C");


        break;
      }



    case 1:
      {
        CLS();
        lcd.home();           // move to position 0 on the first line
        lcd.print("Palier 1");
        lcd.setCursor(0, 1);           // move to position 0 on the second line
        // sous menu
        if (submenu == 0) {
          lcd.print(">T");
          lcd.print(theta1);
          lcd.write(byte(0)); //affiche le caractère °
          lcd.print(" ");
          lcd.print(tempo1);
          lcd.print("min");
        }
        else {
          lcd.print(" T");
          lcd.print(theta1);
          lcd.write(byte(0)); //affiche le caractère °
          lcd.print(">");
          lcd.print(tempo1);
          lcd.print("min");
        }
        //fin sous menu
        break;
      }
    case 2:
      {
        CLS();
        lcd.home();           // move to position 0 on the first line
        lcd.print("Palier 2");
        lcd.setCursor(0, 1);           // move to position 0 on the second line
        // sous menu
        if (submenu == 0) {
          lcd.print(">T");
          lcd.print(theta2);
          lcd.write(byte(0)); //affiche le caractère °
          lcd.print(" ");
          lcd.print(tempo2);
          lcd.print("min");
        }
        else {
          lcd.print(" T");
          lcd.print(theta2);
          lcd.write(byte(0)); //affiche le caractère °
          lcd.print(">");
          lcd.print(tempo2);
          lcd.print("min");
        }
        //fin sous menu
        break;
      }

    case 3:
      {
        CLS();
        lcd.home();           // move to position 0 on the first line
        lcd.print("Palier 3");
        lcd.setCursor(0, 1);           // move to position 0 on the second line
        // sous menu
        if (submenu == 0) {
          lcd.print(">T");
          lcd.print(theta3);
          lcd.write(byte(0)); //affiche le caractère °
          lcd.print(" ");
          lcd.print(tempo3);
          lcd.print("min");
        }
        else {
          lcd.print(" T");
          lcd.print(theta3);
          lcd.write(byte(0)); //affiche le caractère °
          lcd.print(">");
          lcd.print(tempo3);
          lcd.print("min");
        }
        //fin sous menu
        break;
      }

    case 4:
      {
        CLS();
        lcd.home();           // move to position 0 on the first line
        lcd.print("Palier 4");
        lcd.setCursor(0, 1);           // move to position 0 on the second line
        // sous menu
        if (submenu == 0) {
          lcd.print(">T");
          lcd.print(theta4);
          lcd.write(byte(0)); //affiche le caractère °
          lcd.print(" ");
          lcd.print(tempo4);
          lcd.print("min");
        }
        else {
          lcd.print(" T");
          lcd.print(theta4);
          lcd.write(byte(0)); //affiche le caractère °
          lcd.print(">");
          lcd.print(tempo4);
          lcd.print("min");
        }
        //fin sous menu
        break;
      }

    case 5:
      {
        CLS();
        lcd.home();           // move to position 0 on the first line
        lcd.print("Mash out");
        lcd.setCursor(0, 1);           // move to position 0 on the second line
        lcd.print(">T");
        lcd.print(thetaMO);
        lcd.write(byte(0)); //affiche le caractère °
        break;
      }


    case 6:
      {
        CLS();
        lcd.home();           // move to position 0 on the first line
        lcd.print("Ebullition");
        lcd.setCursor(0, 1);           // move to position 0 on the second line
        // sous menu
        if (submenu == 0) {
          lcd.print(">T");
          lcd.print(theta5);
          lcd.write(byte(0)); //affiche le caractère °
          lcd.print(" ");
          lcd.print(tempo5);
          lcd.print("min");
        }
        else {
          lcd.print(" T");
          lcd.print(theta5);
          lcd.write(byte(0)); //affiche le caractère °
          lcd.print(">");
          lcd.print(tempo5);
          lcd.print("min");
        }
        //fin sous menu
        break;
      }


    case 7:
      {
        CLS();
        lcd.home();           // move to position 0 on the first line
        lcd.print("Sauvegarder");
        lcd.setCursor(0, 1);           // move to position 0 on the second line
        lcd.print("app. sur SEL");
        break;
      }

    case 8: //préchauffage
      {
        CLS();
        lcd.home();           // move to position 0 on the first line
        lcd.print("PreCh->");
        lcd.print(thetastart);
        theta_objectif = thetastart;
        lcd.setCursor(0, 1);           // move to position 0 on the second line
        //        lcd.print("TReel=");
        lcd.print("TR=");
        lcd.print(theta_mesure);
        lcd.print("PID="); //pour debug
        lcd.print(tx_chauffe);

        break;
      }

    case 9:
      {
        CLS();
        lcd.home();           // move to position 0 on the first line
        lcd.print("Palier 1->");
        lcd.print(theta1);
        theta_objectif = theta1;



        lcd.setCursor(0, 1);           // move to position 0 on the second line
        //       lcd.print("T=");
        lcd.print(theta_mesure);
        lcd.write(byte(0)); //affiche le caractère °
        lcd.print("/");


        if (tempo1 > minutes) tempo_A = (tempo1 - minutes); else {
          jump = 1;  //vérification du temps écoulé. tempo1 = temps de palier 1 programmé. minutes = minutes écoulées depuis que la température est bonne
          small_Beep = 1;
        }
        lcd.print(tempo_A); //mettre ici le temps restant
        lcd.print("m Rest");
        break;
      }

    case 10:
      {
        CLS();
        lcd.home();           // move to position 0 on the first line
        lcd.print("Palier 2->");
        lcd.print(theta2);
        theta_objectif = theta2;
        lcd.setCursor(0, 1);           // move to position 0 on the second line
        //       lcd.print("T=");
        lcd.print(theta_mesure);
        lcd.write(byte(0)); //affiche le caractère °
        lcd.print("/");
        if ((tempo1 + tempo2) > minutes) tempo_A = ((tempo1 + tempo2) - minutes); else {
          jump = 1;
          small_Beep = 1;
        }
        lcd.print(tempo_A); //mettre ici le temps restant
        lcd.print("m Rest");
        break;
      }

    case 11:
      {
        CLS();
        lcd.home();           // move to position 0 on the first line
        lcd.print("Pal 3->");
        lcd.print(theta3);
        theta_objectif = theta3;
        lcd.setCursor(0, 1);           // move to position 0 on the second line
        //       lcd.print("T=");
        lcd.print(theta_mesure);
        lcd.write(byte(0)); //affiche le caractère °
        lcd.print("/");
        if ((tempo1 + tempo2 + tempo3) > minutes) tempo_A = ((tempo1 + tempo2 + tempo3) - minutes); else {
          jump = 1;
          small_Beep = 1;
        }
        lcd.print(tempo_A); //mettre ici le temps restant
        lcd.print("m Rest");
        if ( tempo_A <= 0) menu++;
        break;
      }

    case 12:
      {
        CLS();
        lcd.home();           // move to position 0 on the first line
        lcd.print("Pal 4->");
        lcd.print(theta4);
        theta_objectif = theta4;
        lcd.setCursor(0, 1);           // move to position 0 on the second line
        //        lcd.print("T=");
        lcd.print(theta_mesure);
        lcd.write(byte(0)); //affiche le caractère °
        lcd.print("/");
        if ((tempo1 + tempo2 + tempo3 + tempo4) > minutes) tempo_A = ((tempo1 + tempo2 + tempo3 + tempo4) - minutes); else {
          jump = 1;
          Beep = 1;
        }
        lcd.print((tempo_A)); //mettre ici le temps restant
        lcd.print("m Rest");
        if (tempo_A <= 0) menu++; //SUSPECT
        break;
      }


    case 13:
      {
        CLS();
        lcd.home();           // move to position 0 on the first line
        lcd.print("Mash out");
        lcd.print(thetaMO);
        theta_objectif = thetaMO;
        lcd.setCursor(0, 1);           // move to position 0 on the second line
        lcd.print("T =");
        lcd.print(theta_mesure);
        lcd.write(byte(0)); //affiche le caractère °
        lcd.print("C");
        break;
      }

    case 14:
      {
        CLS();
        lcd.home();           // move to position 0 on the first line
        lcd.print("Ebu  ->");
        lcd.print(theta5);
        theta_objectif = theta5;
        lcd.setCursor(0, 1);           // move to position 0 on the second line
        //        lcd.print("T=");
        lcd.print(theta_mesure);
        lcd.write(byte(0)); //affiche le caractère °
        lcd.print("/");
        if ((tempo1 + tempo2 + tempo3 + tempo4 + tempo5) > minutes) tempo_A = ((tempo1 + tempo2 + tempo3 + tempo4 + tempo5) - minutes); else {
          jump = 1;
          cooling = minutes;
          Beep = 1;
        }
        lcd.print((tempo_A)); //mettre ici le temps restant
        lcd.print("m Rest");
        if (tempo_A <= 0) menu++; //SUSPECT
        break;
      }

    case 15:
      {
        CLS();
        lcd.home();           // move to position 0 on the first line
        lcd.print("Refroidissement");
        theta_objectif = 0;
        lcd.setCursor(0, 1);           // move to position 0 on the second line
        lcd.print("T=");
        lcd.print(theta_mesure);
        lcd.write(byte(0)); //affiche le caractère °
        lcd.print("C");
        tempo_A =(minutes - cooling);
        lcd.print(tempo_A);
        lcd.print("min");
        break;
      }

    case 109:
      {
        CLS();
        lcd.home();           // move to position 0 on the first line
        lcd.print("ANNULE Palier1 ");
        lcd.setCursor(0, 1);           // move to position 0 on the second line
        lcd.print("SEL=OUI autre=NO");

        break;
      }
    case 110:
      {
        CLS();
        lcd.home();           // move to position 0 on the first line
        lcd.print("ANNULE Palier2 ");
        lcd.setCursor(0, 1);           // move to position 0 on the second line
        lcd.print("SEL=OUI autre=NO");

        break;
      }
    case 111:
      {
        CLS();
        lcd.home();           // move to position 0 on the first line
        lcd.print("ANNULE Palier3 ");
        lcd.setCursor(0, 1);           // move to position 0 on the second line
        lcd.print("SEL=OUI autre=NO");

        break;
      }
    case 112:
      {
        CLS();
        lcd.home();           // move to position 0 on the first line
        lcd.print("ANNULE Palier4 ");
        lcd.setCursor(0, 1);           // move to position 0 on the second line
        lcd.print("SEL=OUI autre=NO");

        break;
      }
    case 113:
      {
        CLS();
        lcd.home();           // move to position 0 on the first line
        lcd.print("ANNULE Mashout ?");
        lcd.setCursor(0, 1);           // move to position 0 on the second line
        lcd.print("SEL=OUI autre=NO");

        break;
      }
    case 114:
      {
        CLS();
        lcd.home();           // move to position 0 on the first line
        lcd.print("ANNULE Ebu ?");
        lcd.setCursor(0, 1);           // move to position 0 on the second line
        lcd.print("SEL=OUI autre=NO");

        break;
      }


    case 209:
      {
        CLS();
        lcd.home();           // move to position 0 on the first line
        lcd.print("SAUTER Palier1?");
        lcd.setCursor(0, 1);           // move to position 0 on the second line
        lcd.print("SEL=OUI autre=NO");

        break;
      }
    case 210:
      {
        CLS();
        lcd.home();           // move to position 0 on the first line
        lcd.print("SAUTER  Palier2?");
        lcd.setCursor(0, 1);           // move to position 0 on the second line
        lcd.print("SEL=OUI autre=NO");

        break;
      }
    case 211:
      {
        CLS();
        lcd.home();           // move to position 0 on the first line
        lcd.print("SAUTER  Palier3?");
        lcd.setCursor(0, 1);           // move to position 0 on the second line
        lcd.print("SEL=OUI autre=NO");

        break;
      }
    case 212:
      {
        CLS();
        lcd.home();           // move to position 0 on the first line
        lcd.print("SAUTER  Palier4?");
        lcd.setCursor(0, 1);           // move to position 0 on the second line
        lcd.print("SEL=OUI autre=NO");

        break;
      }
    case 213:
      {
        CLS();
        lcd.home();           // move to position 0 on the first line
        lcd.print("Passer a l'Ebu?");
        lcd.setCursor(0, 1);           // move to position 0 on the second line
        lcd.print("SEL=OUI autre=NO");

        break;
      }
    case 214:
      {
        CLS();
        lcd.home();           // move to position 0 on the first line
        lcd.print("SAUTER  Ebu?");
        lcd.setCursor(0, 1);           // move to position 0 on the second line
        lcd.print("SEL=OUI autre=NO");

        break;
      }
    default :
      {
        CLS();
        lcd.home();           // move to position 0 on the first line
        lcd.print("Erreur de menu");
        lcd.setCursor(0, 1);           // move to position 0 on the second line
        lcd.print("menu ");
        lcd.print(menu);
        break;
      }

  }

} //************************************************************************************************************************************************************************FIN LCD

// read the buttons
int read_LCD_buttons() //cette fonction renvoie la touche appuyée
{
  adc_key_in = analogRead(0);      // read the value from the sensor
  // my [Mark Bramwell's] buttons when read are centered at these valies: 0, 144, 329, 504, 741
  // we add approx 50 to those values and check to see if we are close
  if (adc_key_in > 1000) return btnNONE; // We make this the 1st option for speed reasons since it will be the most likely result
  if (adc_key_in < 50)   return btnRIGHT;
  if (adc_key_in < 195)  return btnUP;
  if (adc_key_in < 380)  return btnDOWN;
  if (adc_key_in < 555)  return btnLEFT;
  if (adc_key_in < 790)  return btnSELECT;
  return btnNONE;  // when all others fail, return this...
}

float gettemp()
{
  myPID.Compute(); //on Lance un calcul du PID. le calcul se fait en même temps que le rafraichissement des variables de température
 
  int16_t therm, therm2;
  float ttt, ttt2;
  
switch (probe)
    {


case 0:
{
  // Le DAC à 2 entrées reliées à la même sonde. Pour déparasiter, on fait une moyenne des deux lectures
  therm = ads1115.readADC_SingleEnded(THERM_PIN);
  therm2 = ads1115.readADC_SingleEnded(THERM_PIN2);
  therm -= OFFSET_PROBE;
  therm2 -= OFFSET_PROBE;
  ttt = float(therm);
  ttt2 = float(therm2);
  ttt = ttt / COEF_PROBE;
  ttt2 = ttt2 / COEF_PROBE;
  ttt = (ttt + ttt2) / 2;
break;
}

  case 2:
  {
    DS18B20.requestTemperatures(); 
    ttt = DS18B20.getTempCByIndex(0);    
  }
break;
    }
   
  return ttt;
}

void blink() { //fait clignotter la LED
  // if the LED is off turn it on and vice-versa:
  if (ledState == LOW)
    ledState = HIGH;
  else
    ledState = LOW;
  // set the LED with the ledState of the variable:
  digitalWrite(ledPin, ledState);
}

void CLS() //efface l'écran
{
  lcd.clear();
}

void CLS(char ligne) //efface une seule ligne (surcharge de la fonction CLS) 1 = ligne 1 2 = ligne 2
{
  ligne--;
  if (ligne > 1) ligne = 1;
  lcd.setCursor(0, ligne);           // move to position 0 on the first line
  lcd.print("                ");
}

void horloge() //fonction appelée une fois par seconde . On profite de cette fonction pour certaines fonctionalités actualisées une fois par seconde
{
  //pour faire sonner le bipeur 1 seconde lorsque la variable est à 1

  if ( Beep_state >= 1) {
    BeepOFF();
    Beep_state = 0;
  }


  if (Beep == 1)
  {
    digitalWrite(Beep_PIN, 1);
    Beep_state = 1;
    Beep = 0;

  }

  secondes_reel++;


  if (theta_objectif == 0) //cas uniquement présent lors du refroidissement
  {
    secondes++;
  }
  if (theta_mesure > (theta_objectif - hysteresis_Neg) && theta_mesure < (theta_objectif + hysteresis_Pos)) // si on est compris entre les deux valeurs d'hysteresis, on est dans la bonne plage de température. on réduit la force du PID
  {
    Kp = P_weak; // multiplicateur de l'erreur différentielle de température.
    Ki = I_weak; //coef de correction intégrale
    Kd = D_weak; //coef de dérivée

    //
    secondes++; //on compte le temps écoulé pendant que le palier est en cours

    blink(); //on fait clignoter une fois par seconde pour montrer que le palier est lancé

    if (secondes >= 60 ) //on incrémente le compteur minutes toutes les 60s
    {
      secondes = 0;
      if (menu >= 9 && menu != 13 && menu < 100) {
        minutes++;
      }
    }
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
if (theta_mesure > (theta_objectif - hysteresis_Neg) && theta_mesure < (theta_objectif + hysteresis_Pos)){
  theta_PID = theta_mesure; // si on est proche de la température de consigne, on supprime l'offset de sécurité
}
else {
  if (menu !=14) { theta_PID = theta_mesure + PID_OFFSET; }else{theta_PID = theta_mesure;}
  
}
}

// +++++++++++++++++++++++++++++++++++++++++       GESTION DES MENUS                   ++++++++++++++++++++++++++++++++++++++++++++++
void sel_menu()
{


  lcd_key = read_LCD_buttons();  // read the buttons



  if (menu >= 100) { // gestion des sauts de programme en cas d'annulation la valeur est >100  en cas de saut saut > 200
    switch (lcd_key)
    {

      case btnSELECT:
        {
          switch (menu)
          {
            case 108:
              {
                CLS();
                lcd.setCursor(0, 0);
                lcd.print("saut erreur 108");
                menu = 8;
                delay(150);
                break;
              }
            case 109: //on a demandé à annuler l'étape palier 1
              {
                CLS();
                annuler = - 1; //on était au menu 9 on retourne au 8
                minutes = 0; //avant le palier 1 (préchauffage) le temps écoulé était à 0
                delay(150);
                break;
              }
            case 110: //on a demandé à annuler l'étape palier 2
              {
                CLS();
                annuler = -1;
                minutes = 0; //avant le palier 2 (palier 1) le temps écoulé était égal à 0
                delay(150);
                break;
              }
            case 111: //on a demandé à annuler l'étape palier 3
              {
                CLS();
                annuler = - 1; //on était au menu 9 on retourne au 8
                minutes = (tempo1); //avant le palier 3 le temps écoulé était égal au temps en début de palier 2 (fin de palier 1)
                delay(150);
                break;
              }
            case 112: //on a demandé à annuler l'étape palier 4
              {
                CLS();
                annuler = - 1;
                minutes = (tempo1 + tempo2);
                delay(150);
                break;
              }
            case 113: //on a demandé à annuler l'étape rincage mash out
              {
                CLS();
                annuler = - 1;
                minutes = (tempo1 + tempo2 + tempo3);
                delay(150);
                break;
              }
            case 114: //on a demandé à annuler l'étape ébu
              {
                CLS();
                annuler = - 1;
                minutes = (tempo1 + tempo2 + tempo3 + tempo4);
                delay(150);
                break;
              }
            case 115: //on a demandé à annuler l'étape refroidissement/whirlpool
              {
                CLS();
                annuler = - 1;
                minutes = (tempo1 + tempo2 + tempo3 + tempo4 + tempo5);
                cooling = 0;
                delay(150);
                break;
              }
            case 116: //on a demandé à annuler l'étape X
              {
                CLS();
                annuler = - 1;
                minutes = (tempo1 + tempo2 + tempo3 + tempo4 + tempo5);
                delay(150);
                break;
              }


            case 208: //on a demandé à sauter l'étape
              {

                CLS();
                lcd.setCursor(0, 0);
                lcd.print("saut erreur 208");
                menu = 8;
                delay(1000);
                LCD_upd();
                break;
              }

            case 209: //on a demandé à sauter l'étape palier 1
              {
                minutes = tempo1;
                jump = 0;
                annuler = 1;
                delay(150);
                break;
              }
            case 210: //on a demandé à sauter l'étape palier 2
              {
                minutes = (tempo1 + tempo2);
                jump = 0;
                annuler = 1;
                delay(150);
                break;
              }
            case 211: //on a demandé à sauter l'étape palier 3
              {
                minutes = (tempo1 + tempo2 + tempo3);
                jump = 0;
                annuler = 1;
                delay(150);
                break;
              }
            case 212: //on a demandé à sauter l'étape palier 4
              {
                minutes = (tempo1 + tempo2 + tempo3 + tempo4);
                jump = 0;
                annuler = 1;
                delay(150);
                break;
              }
            case 213: //on a demandé à sauter l'étape mash out
              {
                minutes = (tempo1 + tempo2 + tempo3 + tempo4);
                jump = 0;
                annuler = 1;
                delay(150);
                break;
              }
            case 214: //on a demandé à sauter l'étape ebullition
              {
                minutes = (tempo1 + tempo2 + tempo3 + tempo4 + tempo5);
                jump = 0;
                cooling = minutes;
                annuler = 1;
                delay(150);
                break;
              }
              break;
          } //fin switch menu
        }//fin bouton select

      case btnUP: // on annule le saut
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
      case btnDOWN: // on annule le saut
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
      case btnLEFT: // on annule le saut
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
      case btnRIGHT:
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
  } else { //Si on a pas demandé de saut

    // gestion des menus
    switch (lcd_key)               // depending on which button was pushed, we perform an action
    {
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
            if (menu > 7) //sinon on peut aller plus haut
            {


              if (menu >= 9 && menu < 15) //si on est au menu 9 ou + on considère que c'est un saut de programme
              {
                small_Beep = 1;

                menu += 200;
              }

              delay(150);
              LCD_upd();
            }
            if (menu == 8) menu ++; //si on est dans le menu 8 (préchauffe) , le saut est autorisé sans condition
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

              // DEMANDER SI ON SOUHAITE ANNULER L'ETAPE EN COURS
              small_Beep = 1;
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
              {
                thetastart -= 0.25;
                LCD_upd();
                break;
              }


            case 1: //menu réglage palier 1
              {
                if (submenu == 0) {
                  theta1 -= 0.25;
                }
                else {
                  if (tempo1 > 0) tempo1--;
                }
                LCD_upd();
                break;
              }


            case 2: //menu réglage palier 2
              {
                if (submenu == 0) {
                  theta2 -= 0.25;
                }
                else {
                  if (tempo2 > 0) tempo2--;
                }
                LCD_upd();
                break;
              }

            case 3: //menu réglage palier 3
              {
                if (submenu == 0) {
                  theta3 -= 0.25;
                }
                else {
                  if (tempo3 > 0) tempo3--;
                }
                LCD_upd();
                break;

              }
            case 4: //menu réglage palier 4
              {
                if (submenu == 0) {
                  theta4 -= 0.25;
                }
                else {
                  if (tempo4 > 0) tempo4--;
                }
                LCD_upd();
                break;
              }
            case 5:  //menu réglage Mash out - rinçage
              {
                thetaMO -= 0.25;
                LCD_upd();
                break;
              }
            case 6: // ébullition
              {
                if (submenu == 0) {
                  theta5 -= 0.25;
                }
                else {
                  if (tempo5 > 0) tempo5--;
                }
                LCD_upd();
                break;
              }
            case 7: //sauvegarde
              {
                break;
              }
            case 8: //préchauffe
              {
                if (thetastart > 10) thetastart -= 0.25;
                LCD_upd();
                break;
              }
            case 9: //palier 1
              {
                if (theta1 > 10) theta1 -= 0.25;
                LCD_upd();
                break;
              }
            case 10://palier 2
              {
                if (theta2 > 10) theta2 -= 0.25;
                LCD_upd();
                break;
              }
            case 11://palier 3
              {
                if (theta3 > 10) theta3 -= 0.25;
                LCD_upd();
                break;
              }
            case 12://palier 4
              {
                if (theta4 > 10) theta4 -= 0.25;
                LCD_upd();
                break;
              }
            case 13://rinçage
              {
                if (thetaMO > 10) thetaMO -= 0.25;
                LCD_upd();
                break;
              }
            case 14://ébullition
              {
                if (theta5 > 10) theta5 -= 0.25;
                LCD_upd();
                break;
              }
            case 15://mode manuel
              {
                break;
              }
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
              {
                if (thetastart <= 99.75) thetastart += 0.25;
                LCD_upd();
                break;
              }


            case 1: //menu réglage palier 1
              {
                if (submenu == 0) {
                  if (theta1 < 99.75) theta1 += 0.25;
                }
                else {
                  tempo1++;
                }
                LCD_upd();
                break;
              }


            case 2: //menu réglage palier 2
              {
                if (submenu == 0) {
                  if (theta2 < 99.75) theta2 += 0.25;
                }
                else {
                  tempo2++;
                }
                LCD_upd();
                break;
              }

            case 3: //menu réglage palier 3
              {
                if (submenu == 0) {
                  if (theta3 < 99.75) theta3 += 0.25;
                }
                else {
                  tempo3++;
                }
                LCD_upd();
                break;

              }
            case 4: //menu réglage palier 4
              {
                if (submenu == 0) {
                  if (theta4 < 99.75) theta4 += 0.25;
                }
                else {
                  tempo4++;
                }
                LCD_upd();
                break;
              }
            case 5:  //menu réglage Mash out - rinçage
              {
                if (thetaMO < 99.75) thetaMO += 0.25;
                LCD_upd();
                break;
              }
            case 6: // ébullition
              {
                if (submenu == 0) {
                  if (theta5 < 110) theta5 += 0.25;
                }
                else {
                  tempo5++;
                }
                LCD_upd();
                break;
              }
            case 7: //sauvegarde
              {
                break;
              }
            case 8: //préchauffe
              {
                if (thetastart < 99.75) thetastart += 0.25;
                LCD_upd();
                break;
              }
            case 9: //palier 1
              {
                if (theta1 < 99.75) theta1 += 0.25;
                LCD_upd();
                break;
              }
            case 10://palier 2
              {
                if (theta2 < 99.75) theta2 += 0.25;
                LCD_upd();
                break;
              }
            case 11://palier 3
              {
                if (theta3 < 99.75) theta3 += 0.25;
                LCD_upd();
                break;
              }
            case 12://palier 4
              {
                if (theta4 < 99.75) theta4 += 0.25;
                LCD_upd();
                break;
              }
            case 13://rinçage mash out
              {
                if (thetaMO < 99.75) thetaMO += 0.25;
                LCD_upd();
                break;
              }
            case 14://ébullition
              {
                if (theta5 < 110) theta5 += 0.25;
                LCD_upd();
                break;
              }
            case 15://mode manuel
              {
                break;
              }
            case 16://mode erreur
              {
                break;
              }
          }
          break;
        }
      case btnSELECT:
        {
          switch (menu)
          {

            case 0://menu température de préchauffe
              {
                LCD_upd();
                break;
              }

            case 1://menu palier 1
              {
                if (submenu == 0) submenu = 1; else submenu = 0;
                LCD_upd();
                break;
              }
            case 2: //menu palier 2
              {
                if (submenu == 0) submenu = 1; else submenu = 0;
                LCD_upd();
                break;
              }
            case 3: //menu palier 3
              {
                if (submenu == 0) submenu = 1; else submenu = 0;
                LCD_upd();
                break;
              }

            case 4://menu palier 4
              {
                if (submenu == 0) submenu = 1; else submenu = 0;
                LCD_upd();
                break;
              }
            case 5: //menu rinçage
              {
                LCD_upd();
                break;
              }
            case 6: //ébullition
              {
                if (submenu == 0) submenu = 1; else submenu = 0;
                LCD_upd();
                break;
              }

            case 7: //sauvegarde
              {
                
                digitalWrite(ledPin, HIGH);
                // sauvegarde des variables :
                EEPROM.updateLong(0, tempo1);
                EEPROM.updateLong(7, tempo2);
                EEPROM.updateLong(15, tempo3);
                EEPROM.updateLong(23, tempo4);
                EEPROM.updateLong(31, tempo5);
                EEPROM.updateFloat(39, theta1);
                EEPROM.updateFloat(47, theta2);
                EEPROM.updateFloat(55, theta3);
                EEPROM.updateFloat(63, theta4);
                EEPROM.updateFloat(71, theta5);
                EEPROM.updateFloat(79, thetaMO);
                EEPROM.updateFloat(87, thetastart);
                sauve_param();

                myFile = SD.open("datalog.txt", FILE_WRITE);
                if (myFile) {
                  myFile.print("----GraiN.Master----\n");
                  myFile.print(version);
                  myFile.print("\nTempérature de départ :");
                  myFile.print(thetastart);
                  myFile.print("°C\n");
                  myFile.print("Palier 1 :");
                  myFile.print(tempo1);
                  myFile.print("min - ");
                  myFile.print(theta1);
                  myFile.print("°C\n");
                  myFile.print("Palier 2 :");
                  myFile.print(tempo2);
                  myFile.print("min - ");
                  myFile.print(theta2);
                  myFile.print("°C\n");
                  myFile.print("Palier 3 :");
                  myFile.print(tempo3);
                  myFile.print("min - ");
                  myFile.print(theta3);
                  myFile.print("°C\n");
                  myFile.print("Palier 4 :");
                  myFile.print(tempo4);
                  myFile.print("min - ");
                  myFile.print(theta4);
                  myFile.print("°C\n");
                  myFile.print("Mash out :");
                  myFile.print(thetaMO);
                  myFile.print("°C\n");
                  myFile.print("Ebullition :");
                  myFile.print(tempo5);
                  myFile.print("min - ");
                  myFile.print(theta5);
                  myFile.print("°C\n");
                  myFile.print("\n\n");
                  myFile.print("Temps (s),Température cible °C,Température °C,PID (0-255)\n");
                  myFile.close();
                }

                lcd.home();           // move to position 0 on the first line
                lcd.print("    PROGRAMME   ");
                lcd.setCursor(0, 1);           // move to position 0 on the second line
                lcd.print("   SAUVEGARDE ! ");


                delay(600);
                digitalWrite(ledPin, LOW);
                myPID.SetMode(AUTOMATIC);

                //////////////////////////////////
                // on compte le temps que va prendre le programme
                startprog = 1;
                total_time = tempo1 + tempo2 + tempo3 + tempo4 + tempo5;


                lcd.home();           // move to position 0 on the first line
                lcd.print("    BRASSAGE    ");
                lcd.setCursor(0, 1);           // move to position 0 on the second line
                lcd.print("    IMMINENT !  ");
                delay(800);
                CLS();
                lcd.home();
                lcd.print("Duree prevue :  ");
                lcd.setCursor(0, 1);           // move to position 0 on the second line
                lcd.print(total_time);
                lcd.print(" Min");
                delay(1000);
                CLS();


                digitalWrite(ledPin, LOW);

                menu = 8; //saute au menu suivant (départ programme)
                LCD_upd();
                break;

              }
            case 8: //palier 1
              {

                break;
              }
            case 9: //palier 2
              {

                break;
              }
            case 10: //palier 3
              {

                break;
              }

              break;
          } //fin switch menu
          break;
        } //fin buton select


      case btnNONE:
        {
          break;
        }

        break;
    } //fin switch lcd key
  } //fin du else (pas de saut)
}

void regle_chauffe()
{
  if (small_Beep == 1)
  {
    digitalWrite(Beep_PIN, 1);
    small_Beep = 0;
    Beep_state = 1;
  } else
  {
    if ( Beep_state > 0) {
      Beep_state = 0;
      BeepOFF();
    }
  }

  if (tx_chauffe >= 250)
  {
    FULL_ON();
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
  detachInterrupt(0);
  digitalWrite(GATE, HIGH);
  //IRQ0 is pin 2
}
void FULL_OFF()
{
  // set up zero crossing interrupt
  detachInterrupt(0);
  digitalWrite(GATE, LOW);
  //IRQ0 is pin 2
}

void PROPORTIONNAL()

{
  attachInterrupt(0, zeroCrossingInterrupt, CHANGE);
}

void logRecord()
{
  digitalWrite(ledPin, 1);
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.print(secondes_reel);
    dataFile.print(";");
    dataFile.print(theta_objectif);
    dataFile.print(";");
    dataFile.print(theta_mesure);
    dataFile.print(";");
    dataFile.print(tx_chauffe);
    dataFile.print('\n');
    dataFile.close();
    digitalWrite(ledPin, 0);
  }
}


void sauve_param()
{
  digitalWrite(ledPin, 1);

  if (SD.exists("setup.grm")) {
    SD.remove("setup.grm");
  }

  File dataFile = SD.open("setup.grm", FILE_WRITE);
  if (dataFile) {
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
    dataFile.print("hysterPos");
    dataFile.print("=");
    dataFile.print(hysteresis_Pos);
    dataFile.print(",");
    dataFile.print("hysterNeg");
    dataFile.print("=");
    dataFile.print(hysteresis_Neg);
    dataFile.print(",");
    dataFile.print("thetastrt");
    dataFile.print("=");
    dataFile.print(thetastart);
    dataFile.print(",");
    dataFile.print("theta0001");
    dataFile.print("=");
    dataFile.print(theta1);
    dataFile.print(",");
    dataFile.print("tempo0001");
    dataFile.print("=");
    dataFile.print(tempo1);
    dataFile.print(",");
    dataFile.print("theta0002");
    dataFile.print("=");
    dataFile.print(theta2);
    dataFile.print(",");
    dataFile.print("tempo0002");
    dataFile.print("=");
    dataFile.print(tempo2);
    dataFile.print(",");
    dataFile.print("theta0003");
    dataFile.print("=");
    dataFile.print(theta3);
    dataFile.print(",");
    dataFile.print("tempo0003");
    dataFile.print("=");
    dataFile.print(tempo3);
    dataFile.print(",");
    dataFile.print("theta0004");
    dataFile.print("=");
    dataFile.print(theta4);
    dataFile.print(",");
    dataFile.print("tempo0004");
    dataFile.print("=");
    dataFile.print(tempo4);
    dataFile.print(",");
    dataFile.print("theta0005");
    dataFile.print("=");
    dataFile.print(theta5);
    dataFile.print(",");
    dataFile.print("tempo0005");
    dataFile.print("=");
    dataFile.print(tempo5);
    dataFile.print(",");
    dataFile.print("thetaMO00");
    dataFile.print("=");
    dataFile.print(thetaMO);
    dataFile.print(",");
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
    dataFile.print("BEEPPIN00");
    dataFile.print("=");
    dataFile.print(Beep_PIN);
    dataFile.print(",");
    dataFile.print("PIDOFFSET");
    dataFile.print("=");
    dataFile.print(PID_OFFSET);
    dataFile.print(",");
    dataFile.print("PROBETYPE");
    dataFile.print("=");
    dataFile.print(probe);
    dataFile.print(",");
    dataFile.print("ONEWIRE00");
    dataFile.print("=");
    dataFile.print(ONE_WIRE_BUS);
    dataFile.print(",");   
    dataFile.close();
  }
  digitalWrite(ledPin, 0);
}
void BeepON(){
digitalWrite(Beep_PIN, BeepONState);
}

void BeepOFF(){
digitalWrite(Beep_PIN, BeepOFFState);
}
