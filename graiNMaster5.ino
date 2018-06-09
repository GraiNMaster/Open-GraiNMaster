/*******************************************************
  GraiNMaster
  système de contrôle de température de la cuve
  Débuté le 24/11/2015 par Nicolas Morival
  Testé et compilé sur Arduino IDE v1.6.8

note sur les indices de version : 
VA.BCD 
A = version majeure, incrémenté si le fonctionnement du programme est fortement modifié ou qu'une fonction majeure est ajoutée
B = Mise à jour fonctionnelle ou ajout de fonctions visibles par l'utilisateur
C = Mise à jour majeure du code sans changement pour l'utilisateur
D = correction de bugs

  v1.000 le 14/05/2016
  V1.002 le 08/10/2016
  nettoyage du code, suppression des morceaux de code inutiles (relatifs à la SD)
  suppression du paramètre sur SD detect : cette broche est une interruption fixée par le hardware
  ajout de la possibilité d'utiliser une sonde DS18B20
  v1.003 le 09/11/2017
  correction de bugs sur l'initialisation de l'eeprom (bug relevé par artiche)
  mise en fonction du buzzer (suggestion artiche)
  en cas de problème avec le module SD, voir ligne 166 - 167
  v1.010 le 14/11/2017
  Refonte majeure du code par  mise en fonction des portions redondantes de code (contribution de lasiusSp)
  
*******************************************************/
#define version " --V1.010-- "

//bibliothèques
#include <SPI.h>
#include <SD.h>
#include <PID_v1.h>             //gestion de l'algo PID
#include <avr/pgmspace.h>
#include <EEPROMex.h>
#include <EEPROMVar.h>          // gestion des enregistrrements en EEPROM
#include "Timer.h"              // gestion des timers
#include <LiquidCrystal.h>      //gestion du LCD
#include <avr/io.h>
#include <avr/interrupt.h>      //gestion des interruptions
#include <Wire.h>
#include <Adafruit_ADS1015.h>   // gestion du CAN 16 bits
#include <OneWire.h>
#include <DallasTemperature.h>

int DETECT = 2, GATE = 3;       //Broches utilisées pour le pilotage du triac -> detect : impulsion du passage au point zéro, gate : gachette du triac
int PULSE = 4;                  //triac gate pulse : largeur de l'impulsion pour activer le triac, en nombre de cycles d'horloge
int HALF_WAVE = 560;            //nombre de "tics" d'horloge à chaque demi onde 50Hz . 625 théoriques, mais 560 réels .
int probe = 2;                  // probe = 0 si on utilise le dac externe, probe = 2 pour une sonde DS18B20
int ONE_WIRE_BUS = 30;          // DS18B20 pin

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);
Adafruit_ADS1115 ads1115(0x48);       // déclaration de l'ADC 16 bits
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);  //  pins utilisées par le LCD

#define BeepONState 1
#define BeepOFFState 0
// constantes pour les touches reliées au module LCD
#define btnRIGHT 0
#define btnUP 1
#define btnDOWN 2
#define btnLEFT 3
#define btnSELECT 4
#define btnNONE 5

// pour la lecture de la sonde, on lit sur deux entrées dont les filtres passe bas sont légèrement différents, afin de faire une moyenne et augmenter la précision
int THERM_PIN2 = 1;             // entrée n°1 du dac 16bits
int THERM_PIN = 3;              // entrée n°3 du dac 16bits
int OFFSET_PROBE = 138;         // correction constante de la température
int COEF_PROBE = 260;           // correction proportionelle
// Température calculée = (mesure du DAC  - OFFSET_PROBE) / COEF_PROBE

//valeurs de l'hysteresis pour passage en PID fin ou grossier
int hysteresis_Pos = 1;
int hysteresis_Neg = 1;

// valeurs utilisées pour configurer le LCD
int backLight = 10;     // LCD Panel Backlight LED connected to digital pin 10
int lcd_key = 0;
int adc_key_in = 0;
int ledPin = 13;
int ledState = LOW;

// variables du programme
volatile unsigned int menu = 0;
unsigned char submenu = 0;
unsigned char startprog = 0;      //marqueur pour indiquer que le programme s'éxécute
unsigned char jump = 0;           //pour indiquer qu'on va sauter un menu
char annuler = 0;                 //pour annuler les opérations 0 = fonctionnement normal  / -1 on recule d'une étape / 1 on avance d'une étape

unsigned char secondes = 0;
unsigned long secondes_reel = 0;  //temps écoulé depuis le début du programme, utilisé pour le data log
unsigned int minutes = 0;         //minutes écoulées
unsigned long total_time = 0;     //cumul des temps programme
unsigned long temps_pause = 0;    //si le programme est en pause, compte le temps écoulé
int tempo1 = 0;         // palier empâtage céréales non maltées
int tempo2 = 15;        // palier protéinique
int tempo3 = 35;        // palier de sacharification B amylase
int tempo4 = 30;        // palier de sacharification A amylase
int tempo5 = 70;        // ébullition
int tempo_A = 0;        // pour l'affichage temporaire

float thetastart = 70;  //température de préchauffe de la cuve
float theta1 = 40.25;   //température empâtage céréales non maltées
float theta2 = 52.5;    //température empâtage céréales non maltées
float theta3 = 62.0;    //température de sacharification B amylase
float theta4 = 68.75;   //température de sacharification A amylase
float theta5 = 100.0;   //température d'ébullition
float thetaMO = 75.75;  //température de mash out

unsigned long cooling = 0;  // compteur du temps de refroidissement - non utilisé pour l'instant
float temp;                 //temporaire
int therm;                  //variable pour le relevé de température

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

PID myPID( & theta_PID, & tx_chauffe, & theta_objectif, Kp, Ki, Kd, DIRECT); //déclaration du PID

// variables utilisées pour faire marcher le bippeur
int Beep_PIN = 40;      // pin du bipeur
unsigned char Beep_state = 0;
unsigned char Beep = 0;
unsigned char small_Beep = 0;

Timer T;        //réglages des timers
File myFile;    //déclaratioon du fichier pour la SD
int check_data = 0;

void initIntVar(int* var, String str, int* chk) {
    str.remove(0, 10);
    *(var) = str.toInt();
    *(chk)++;
}

void initFloatVar(float* var, String str, int* chk) {
    str.remove(0, 10);
    *(var) = str.toFloat();
    *(chk)++;
}

void initDoubleVar(double* var, String str, int* chk) {
    str.remove(0, 10);
    *(var) = str.toFloat();
    *(chk)++;
}

void setup() {
    //config pour SD
    String inString = "";
    float variable;

    pinMode(53, OUTPUT); //la pin 53 est normalement attribuée à la fonction SPI Slave Select (SS)Permet d'activer ou désactiver le périphérique
    // Suivant les modules SD, il peut être nécessaire de commenter cette ligne (c'est le cas de mon module même si c'est pas logique)

    // ECRAN DE PRESENTATION ----------------------------
    lcd.begin(16, 2); // déclaration de la bibliothèque LCD avec 16 caractères, 2 lignes
    lcd.home(); // met le curseur au début

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
        lcd.scrollDisplayLeft();
        delay(100);
    }

    lcd.clear();
    // set up des pins
    pinMode(DETECT, INPUT);     //détection du passage au pont zéro / zero cross detect
    digitalWrite(DETECT, HIGH); // enable pull-up resistor
    pinMode(GATE, OUTPUT);      //triac gate control
    pinMode(Beep_PIN, OUTPUT);
    BeepOff();
    ads1115.begin();            // démarrage du DAC 16 bits

    // set up Timer1
    //(see ATMEGA 328 data sheet pg 134 for more details)
    OCR1A = 25;     //initialize the comparator
    TIMSK1 = 0x03;  //enable comparator A and overflow interrupts
    TCCR1A = 0x00;  //timer control registers set for
    TCCR1B = 0x00;  //normal operation, timer disabled

    myPID.SetOutputLimits(0, 255);
    EEPROM.setMaxAllowedWrites(48);

    therm = gettemp();

    lcd.home();                     // move cursor to beginning of line "0"
    lcd.print("initialise  SD  ");  // print a simple message
    delay(600);
    if (!SD.begin(4)) {
        printLineOne("ERREUR SD CARD ");
        BeepOn();
        delay(800);
        BeepOff();
    }

    printLineOne(" Lecture  param ");  // print a simple message
    printLineTwo("                ");

    if (SD.exists("setup.grm")) {
        printLineTwo(" sur carte SD ");
        delay(1500);
        myFile = SD.open("setup.grm");

        while (myFile.available()) {
            int inChar = myFile.read();
            if ((inChar != ',')) {
                inString += (char) inChar;
            } else { //on a récupéré la première ligne sous forme "variable=000.00"
                if (inString.startsWith("THERMPIN2")) initIntVar(&THERM_PIN2, inString, &check_data);
                if (inString.startsWith("THERMPIN1")) initIntVar(&THERM_PIN, inString, &check_data);
                if (inString.startsWith("OFSETPRBE")) initIntVar(&OFFSET_PROBE, inString, &check_data);
                if (inString.startsWith("COEFPROBE")) initIntVar(&COEF_PROBE, inString, &check_data);
                if (inString.startsWith("hysterPos")) initIntVar(&hysteresis_Pos, inString, &check_data);
                if (inString.startsWith("hysterNeg")) initIntVar(&hysteresis_Neg, inString, &check_data);
                if (inString.startsWith("thetastrt")) initFloatVar(&thetastart, inString, &check_data);
                if (inString.startsWith("theta0001")) initFloatVar(&theta1, inString, &check_data);
                if (inString.startsWith("theta0002")) initFloatVar(&theta2, inString, &check_data);
                if (inString.startsWith("theta0003")) initFloatVar(&theta3, inString, &check_data);
                if (inString.startsWith("theta0004")) initFloatVar(&theta4, inString, &check_data);
                if (inString.startsWith("theta0005")) initFloatVar(&theta5, inString, &check_data);
                if (inString.startsWith("tempo0001")) initIntVar(&tempo1, inString, &check_data);
                if (inString.startsWith("tempo0002")) initIntVar(&tempo2, inString, &check_data);
                if (inString.startsWith("tempo0003")) initIntVar(&tempo3, inString, &check_data);
                if (inString.startsWith("tempo0004")) initIntVar(&tempo4, inString, &check_data);
                if (inString.startsWith("tempo0005")) initIntVar(&tempo5, inString, &check_data);
                if (inString.startsWith("thetaMO00")) initFloatVar(&thetaMO, inString, &check_data);
                if (inString.startsWith("Pstrong00")) initDoubleVar(&P_strong, inString, &check_data);
                if (inString.startsWith("Istrong00")) initDoubleVar(&I_strong, inString, &check_data);
                if (inString.startsWith("Dstrong00")) initDoubleVar(&D_strong, inString, &check_data);
                if (inString.startsWith("Pweak0000")) initDoubleVar(&P_weak, inString, &check_data);
                if (inString.startsWith("Iweak0000")) initDoubleVar(&I_weak, inString, &check_data);
                if (inString.startsWith("Dweak0000")) initDoubleVar(&D_weak, inString, &check_data);
                if (inString.startsWith("LEDPIN000")) initIntVar(&ledPin, inString, &check_data);
                if (inString.startsWith("GATE00000")) initIntVar(&GATE, inString, &check_data);
                if (inString.startsWith("PULSE0000")) initIntVar(&PULSE, inString, &check_data);
                if (inString.startsWith("HALFWAVE0")) initIntVar(&HALF_WAVE, inString, &check_data);
                if (inString.startsWith("BEEPPIN00")) initIntVar(&Beep_PIN, inString, &check_data);
                if (inString.startsWith("PIDOFFSET")) initFloatVar(&PID_OFFSET, inString, &check_data);
                if (inString.startsWith("PROBETYPE")) initIntVar(&probe, inString, &check_data);
                if (inString.startsWith("ONEWIRE00")) initIntVar(&ONE_WIRE_BUS, inString, &check_data);
                inString = "";
            }
        }

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(check_data);
        lcd.print("/32");
        printLineTwo("parametres lus ");
        myFile.close();

        pinMode(DETECT, INPUT);     //zero cross detect
        digitalWrite(DETECT, HIGH); //enable pull-up resistor
        pinMode(GATE, OUTPUT);      //triac gate control
        pinMode(Beep_PIN, OUTPUT);
        pinMode(backLight, INPUT);  //set backlight pin to input to avoid MCU overcurent on pin 10
        //   pinMode(11, OUTPUT);     // pin11 = MOSI
        pinMode(ledPin, OUTPUT);
        delay(1500);
    } else {
        printLineTwo(" sur EEPROM ");
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

    checkTempo(&tempo1, 0);
    checkTempo(&tempo2, 15);
    checkTempo(&tempo3, 35);
    checkTempo(&tempo4, 30);
    checkTempo(&tempo5, 70);
    checkTheta(&theta1, 40.25);
    checkTheta(&theta2, 52.50);
    checkTheta(&theta3, 62.00);
    checkTheta(&theta4, 68.75);
    checkTheta(&theta5, 95.75);
    checkTheta(&thetaMO, 76);
    checkTheta(&thetastart, 70.75);

    // DEFINITION DES PINs
    pinMode(DETECT, INPUT);     //zero cross detect
    digitalWrite(DETECT, HIGH); //enable pull-up resistor
    pinMode(GATE, OUTPUT);      //triac gate control
    pinMode(Beep_PIN, OUTPUT);
    pinMode(backLight, INPUT);  //set backlight pin to input to avoid MCU overcurent on pin 10
    // pinMode(11, OUTPUT);     // pin11 = MOSI sur uno
    pinMode(ledPin, OUTPUT);

    int T1 = T.every(1200, lecture);      // constante de temps pour le timer 1 - lecture de la sonde
    int T2 = T.every(128, sel_menu);      // constante de temps pour le timer 2 - lecture des touches
    int T3 = T.every(1024, LCD_upd);      // timer 3 - mise à jour écran
    int T4 = T.every(1000, horloge);      // timer 4 - compte les secondes et minutes
    int T5 = T.every(350, regle_chauffe); //timer 5 - fréquence de mise à jour de la chauffe
    int T6 = T.every(10000, logRecord);   //enregiste sur la sd

    printLineOne("T de prechauffe ");
    printLineTwo(">T");
    printDegreeChar();
    lcd.print("C");

    for (int positionCounter = 0; positionCounter < 16; positionCounter++) {
        lcd.scrollDisplayRight();
        delay(8);
    }

    OCR1A = 500;
    lcd.clear();
    LCD_upd();
    // set up zero crossing interrupt
    attachInterrupt(0, zeroCrossingInterrupt, CHANGE);
    //IRQ0 is pin 2. Call zeroCrossingInterrupt
    //on rising signal
    menu = 0;
}

void checkTempo(int* tempo, int value) {
    if (isnan(*(tempo)) || *(tempo) < 0 || *(tempo) > 300) *(tempo) = value;
}

void checkTheta(float* theta, float value) {
    if (isnan(*(theta)) || *(theta) < 1 || *(theta) > 110) *(theta) = value;
}

//Boucle principale ========================================
void loop() {
    T.update();
    //gestion des saut d'étapes manuels
    if (annuler == -1) {
        menu--; //on était au menu 9 on retourne au 8
        annuler = 0;
    } else if (annuler == 1) {
        menu++;
        jump = 0;   //par sécurité on ne veut pas de double saut au cas ou le passage au palier suivant était prévu au même moment
        annuler = 0;
    }
    //gestion des sauts d'étapes automatiques
    if (jump == 1) {
        menu++;
        jump = 0;
    }
} //fin Boucle principale ===================================

void printDegreeChar() {
    lcd.write(byte(0)); //affiche le caractère °
}

void printLineOne(const char* str) {
    lcd.clear();
    lcd.home();
    lcd.print(str);
}

void printLineTwo(const char* str) {
    lcd.setCursor(0, 1);
    lcd.print(str);
}

void printLinePalier(int submenu, float theta, int tempo) {
    lcd.setCursor(0, 1);
    lcd.print(submenu == 0 ? ">T" : " T");
    lcd.print(theta);
    printDegreeChar();
    lcd.print(submenu == 0 ? " " : ">");
    lcd.print(tempo);
    lcd.print("min");
}

void printScreenQuery(const char* str) {
    lcd.clear();
    lcd.home();
    lcd.print(str);
    lcd.setCursor(0, 1);
    lcd.print("SEL=OUI autre=NO");
}

void checkStepJump(const char* label, float theta, int tempo) {
    printLineOne(label);
    lcd.print(theta);
    theta_objectif = theta;
    lcd.setCursor(0, 1);
    lcd.print(theta_mesure);
    printDegreeChar();
    lcd.print("/");
    if (tempo > minutes) {
        tempo_A = tempo - minutes;
    } else {
        jump = 1;
        small_Beep = 1;
    }
    lcd.print(tempo_A);
    lcd.print("m Rest");
}

// affiche les infos à l'écran *********************************************************
void LCD_upd() {
    switch (menu) {
        case 0:
            printLineOne("T de prechauffe ");
            printLineTwo(">T");
            lcd.print(thetastart);
            printDegreeChar();
            lcd.print("C");
            break;
        case 1:
            printLineOne("Palier 1");
            printLinePalier(submenu, theta1, tempo1);
            break;
        case 2:
            printLineOne("Palier 2");
            printLinePalier(submenu, theta2, tempo2);
            break;
        case 3:
            printLineOne("Palier 3");
            printLinePalier(submenu, theta3, tempo3);
            break;
        case 4:
            printLineOne("Palier 4");
            printLinePalier(submenu, theta4, tempo4);
            break;
        case 5:
            printLineOne("Mash out");
            printLineTwo(">T");
            lcd.print(thetaMO);
            printDegreeChar();
            break;
        case 6:
            printLineOne("Ebullition");
            printLinePalier(submenu, theta5, tempo5);
            break;
        case 7:
            printLineOne("Sauvegarder");
            printLineTwo("app. sur SEL");
            break;
        case 8: //préchauffage
            printLineOne("PreCh->");
            lcd.print(thetastart);
            theta_objectif = thetastart;
            printLineTwo("TR=");
            lcd.print(theta_mesure);
            lcd.print("PID="); //pour debug
            lcd.print(tx_chauffe);
            break;
        case 9:
            checkStepJump("Pal 1->", theta1, tempo1);
            break;
        case 10:
            checkStepJump("Pal 2->", theta2, tempo1 + tempo2);
            break;
        case 11:
            checkStepJump("Pal 3->", theta3, tempo1 + tempo2 + tempo3);
            break;
        case 12:
            checkStepJump("Pal 3->", theta4, tempo1 + tempo2 + tempo3 + tempo4);
            if (tempo_A <= 0) menu++; //SUSPECT
            break;
        case 13:
            printLineOne("Mash out");
            lcd.print(thetaMO);
            theta_objectif = thetaMO;
            printLineTwo("T =");
            lcd.print(theta_mesure);
            printDegreeChar();
            lcd.print("C");
            break;
        case 14:
            checkStepJump("Ebu ->", theta5, tempo1 + tempo2 + tempo3 + tempo4 + tempo5);
            if (tempo_A <= 0) menu++; //SUSPECT
            break;
        case 15:
            printLineOne("Refroidissement");
            theta_objectif = 0;
            printLineTwo("T=");
            lcd.print(theta_mesure);
            printDegreeChar();
            lcd.print("C");
            lcd.print(minutes - cooling);
            lcd.print("min");
            break;
        case 109:
            printScreenQuery("ANNULE Palier1 ");
            break;
        case 110:
            printScreenQuery("ANNULE Palier2 ");
            break;
        case 111:
            printScreenQuery("ANNULE Palier3 ");
            break;
        case 112:
            printScreenQuery("ANNULE Palier4 ");
            break;
        case 113:
            printScreenQuery("ANNULE Mashout ?");
            break;
        case 114:
            printScreenQuery("ANNULE Ebu ?");
            break;
        case 209:
            printScreenQuery("SAUTER Palier1?");
            break;
        case 210:
            printScreenQuery("SAUTER  Palier2?");
            break;
        case 211:
            printScreenQuery("SAUTER  Palier3?");
            break;
        case 212:
            printScreenQuery("SAUTER  Palier4?");
            break;
        case 213:
            printScreenQuery("Passer a l'Ebu?");
            break;
        case 214:
            printScreenQuery("SAUTER  Ebu?");
            break;
        default:
            printLineOne("Erreur de menu");
            printLineTwo("menu ");
            lcd.print(menu);
            break;
    }
} //************************************************************************************************************************************************************************FIN LCD

// read the buttons : cette fonction renvoie la touche appuyée
int read_LCD_buttons() {
    adc_key_in = analogRead(0); // read the value from the sensor
    // my [Mark Bramwell's] buttons when read are centered at these valies: 0, 144, 329, 504, 741
    // we add approx 50 to those values and check to see if we are close
    if (adc_key_in > 1000) return btnNONE; // We make this the 1st option for speed reasons since it will be the most likely result
    if (adc_key_in < 50) return btnRIGHT;
    if (adc_key_in < 195) return btnUP;
    if (adc_key_in < 380) return btnDOWN;
    if (adc_key_in < 555) return btnLEFT;
    if (adc_key_in < 790) return btnSELECT;
    return btnNONE; // when all others fail, return this...
}

float gettemp() {
    int16_t therm, therm2;
    float ttt;
    myPID.Compute(); //on Lance un calcul du PID. le calcul se fait en même temps que le rafraichissement des variables de température

    switch (probe) {
        case 0:
            // Le DAC à 2 entrées reliées à la même sonde. Pour déparasiter, on fait une moyenne des deux lectures
            therm = ads1115.readADC_SingleEnded(THERM_PIN);
            therm2 = ads1115.readADC_SingleEnded(THERM_PIN2);
            ttt = ((float(therm - OFFSET_PROBE) + float(therm2 - OFFSET_PROBE)) / COEF_PROBE) / 2;
            break;
        case 2:
            DS18B20.requestTemperatures();
            ttt = DS18B20.getTempCByIndex(0);
            break;
    }

    return ttt;
}

void blink() { //fait clignoter la LED
    ledState = ledState == LOW ? HIGH : LOW;
    digitalWrite(ledPin, ledState);
}

void BeepOn() {
    digitalWrite(Beep_PIN, BeepONState);
}

void BeepOff() {
    digitalWrite(Beep_PIN, BeepOFFState);
}

void horloge() { //fonction appelée une fois par seconde . On profite de cette fonction pour certaines fonctionalités actualisées une fois par seconde
    //pour faire sonner le bipeur 1 seconde lorsque la variable est à 1
    if (Beep_state >= 1) {
        BeepOff();
        Beep_state = 0;
    }
    if (Beep == 1) {
        BeepOn();
        Beep_state = 1;
        Beep = 0;
    }
    secondes_reel++;
    if (theta_objectif == 0) { //cas uniquement présent lors du refroidissement
        secondes++;
    }
    if (theta_mesure > (theta_objectif - hysteresis_Neg) && theta_mesure < (theta_objectif + hysteresis_Pos)) { // si on est compris entre les deux valeurs d'hysteresis, on est dans la bonne plage de température. on réduit la force du PID
        Kp = P_weak;  // multiplicateur de l'erreur différentielle de température.
        Ki = I_weak;  //coef de correction intégrale
        Kd = D_weak;  //coef de dérivée
        secondes++;   //on compte le temps écoulé pendant que le palier est en cours
        blink();      //on fait clignoter une fois par seconde pour montrer que le palier est lancé

        if (secondes >= 60) { //on incrémente le compteur minutes toutes les 60s
            secondes = 0;
            if (menu >= 9 && menu != 13 && menu < 100) {
                minutes++;
            }
        }
    } else {
        Kp = P_strong; // multiplicateur de l'erreur différentielle de température. 1° d'écart = Kp% de chauffe
        Ki = I_strong; //coef de correction intégrale
        Kd = D_strong; //coef de dérivée
    }
    myPID.SetTunings(Kp, Ki, Kd);
}

void lecture() {
    theta_mesure = gettemp();
    if (theta_mesure > (theta_objectif - hysteresis_Neg) && theta_mesure < (theta_objectif + hysteresis_Pos)) {
        theta_PID = theta_mesure; // si on est proche de la température de consigne, on supprime l'offset de sécurité
    } else if (menu != 14) {
            theta_PID = theta_mesure + PID_OFFSET;
    } else {
            theta_PID = theta_mesure;
    }
}

void cancel() {
    lcd.clear();
    annuler = -1;
    delay(150);
}

void jumpStep() {
    jump = 0;
    annuler = 1;
    delay(150);
}

// +++++++++++++++++++++++++++++++++++++++++       GESTION DES MENUS                   ++++++++++++++++++++++++++++++++++++++++++++++
void sel_menu() {
    lcd_key = read_LCD_buttons();   // read the buttons
    if (menu >= 100) {              // gestion des sauts de programme en cas d'annulation la valeur est >100  en cas de saut saut > 200
        switch (lcd_key) {
            case btnSELECT:
                switch (menu) {
                    case 108:
                        printLineOne("saut erreur 108");
                        menu = 8;
                        delay(150);
                        break;
                    case 109: //on a demandé à annuler l'étape palier 1
                    case 110: //on a demandé à annuler l'étape palier 2
                        minutes = 0; //avant le palier 1 ou palier 2 (prechauffage ou palier 1) le temps écoulé était égal à 0
                        cancel();
                        break;
                    case 111: //on a demandé à annuler l'étape palier 3
                        minutes = (tempo1); //avant le palier 3 le temps écoulé était égal au temps en début de palier 2 (fin de palier 1)
                        cancel();
                        break;
                    case 112: //on a demandé à annuler l'étape palier 4
                        minutes = (tempo1 + tempo2);
                        cancel();
                        break;
                    case 113: //on a demandé à annuler l'étape rincage mash out
                        minutes = (tempo1 + tempo2 + tempo3);
                        cancel();
                        break;
                    case 114: //on a demandé à annuler l'étape ébu
                        minutes = (tempo1 + tempo2 + tempo3 + tempo4);
                        cancel();
                        break;
                    case 115: //on a demandé à annuler l'étape refroidissement/whirlpool
                        minutes = (tempo1 + tempo2 + tempo3 + tempo4 + tempo5);
                        cooling = 0;
                        cancel();
                        break;
                   case 116: //on a demandé à annuler l'étape X
                        minutes = (tempo1 + tempo2 + tempo3 + tempo4 + tempo5);
                        cancel();
                        break;
                   case 208: //on a demandé à sauter l'étape
                        printLineOne("saut erreur 208");
                        menu = 8;
                        delay(1000);
                        LCD_upd();
                        break;
                    case 209: //on a demandé à sauter l'étape palier 1
                        minutes = tempo1;
                        jumpStep();
                        break;
                    case 210: //on a demandé à sauter l'étape palier 2
                        minutes = (tempo1 + tempo2);
                        jumpStep();
                        break;
                    case 211: //on a demandé à sauter l'étape palier 3
                        minutes = (tempo1 + tempo2 + tempo3);
                        jumpStep();
                        break;
                    case 212: //on a demandé à sauter l'étape palier 4
                        minutes = (tempo1 + tempo2 + tempo3 + tempo4);
                        jumpStep();
                        break;
                    case 213: //on a demandé à sauter l'étape mash out
                        minutes = (tempo1 + tempo2 + tempo3 + tempo4);
                        jumpStep();
                        break;
                    case 214: //on a demandé à sauter l'étape ebullition
                        minutes = (tempo1 + tempo2 + tempo3 + tempo4 + tempo5);
                        cooling = minutes;
                        jumpStep();
                        break;
                } //fin switch menu
                break;
            case btnUP: // on annule le saut
            case btnDOWN: // on annule le saut
            case btnLEFT: // on annule le saut
            case btnRIGHT:
                lcd.clear();
                jump = 0;
                delay(500);
                if (menu > 199) {
                    menu -= 200;
                } else {
                    menu -= 100;
                }
                break;
        } // fin switch lcd key
    } else { //Si on a pas demandé de saut
        // gestion des menus
        switch (lcd_key) {
            case btnUP:
                if (startprog == 0) { //si on est encore en phase de programation, on peut naviguer du menu 0 à 7
                    if (menu < 7) {
                        menu++;
                        delay(150);
                        LCD_upd();
                    }
                } else { //le programme est lancé
                    if (menu > 7) { //sinon on peut aller plus haut
                        if (menu >= 9 && menu < 15) { //si on est au menu 9 ou + on considère que c'est un saut de programme
                            small_Beep = 1;
                            menu += 200;
                        }
                        delay(150);
                        LCD_upd();
                    }
                    if (menu == 8) menu++; //si on est dans le menu 8 (préchauffe) , le saut est autorisé sans condition
                } //fin else
                break;
        case btnDOWN:
            if (startprog == 0) { //si on est encore en phase de programation, on peut naviguer du 7 à 0
                if (menu > 0) {
                    menu--;
                    delay(150);
                    LCD_upd();
                }
             } else {
                if (menu > 8) { //si on est en phase brassage (a partir de menu 9)
                    // DEMANDER SI ON SOUHAITE ANNULER L'ETAPE EN COURS
                    small_Beep = 1;
                    menu += 100;
                    delay(150);
                    LCD_upd();
                }
            }
            break;
        case btnLEFT:
            switch (menu) {
                case 0: //menu température de préchauffe
                    thetastart -= 0.25;
                    LCD_upd();
                    break;
                case 1: //menu réglage palier 1
                    if (submenu == 0) {
                        theta1 -= 0.25;
                    } else {
                        if (tempo1 > 0) tempo1--;
                    }
                    LCD_upd();
                    break;
                case 2: //menu réglage palier 2
                    if (submenu == 0) {
                        theta2 -= 0.25;
                    } else {
                        if (tempo2 > 0) tempo2--;
                    }
                    LCD_upd();
                    break;
                case 3: //menu réglage palier 3
                    if (submenu == 0) {
                        theta3 -= 0.25;
                    } else {
                        if (tempo3 > 0) tempo3--;
                    }
                    LCD_upd();
                    break;
                case 4: //menu réglage palier 4
                    if (submenu == 0) {
                        theta4 -= 0.25;
                    } else {
                        if (tempo4 > 0) tempo4--;
                    }
                    LCD_upd();
                    break;
                case 5: //menu réglage Mash out - rinçage
                    thetaMO -= 0.25;
                    LCD_upd();
                    break;
                case 6: // ébullition
                    if (submenu == 0) {
                        theta5 -= 0.25;
                    } else {
                        if (tempo5 > 0) tempo5--;
                    }
                    LCD_upd();
                    break;
                case 8: //préchauffe
                    if (thetastart > 10) thetastart -= 0.25;
                    LCD_upd();
                    break;
                case 9: //palier 1
                    if (theta1 > 10) theta1 -= 0.25;
                    LCD_upd();
                    break;
                case 10: //palier 2
                    if (theta2 > 10) theta2 -= 0.25;
                    LCD_upd();
                    break;
                case 11: //palier 3
                    if (theta3 > 10) theta3 -= 0.25;
                    LCD_upd();
                    break;
                case 12: //palier 4
                    if (theta4 > 10) theta4 -= 0.25;
                    LCD_upd();
                    break;
                case 13: //rinçage
                    if (thetaMO > 10) thetaMO -= 0.25;
                    LCD_upd();
                    break;
                case 14: //ébullition
                    if (theta5 > 10) theta5 -= 0.25;
                    LCD_upd();
                    break;
                case 7: //sauvegarde
                case 15: //mode manuel
                case 16: //mode erreur
                    break;
            }
            break;
        case btnRIGHT:
            switch (menu) { //faire un switch menu
                case 0: //menu température de préchauffe
                    if (thetastart <= 99.75) thetastart += 0.25;
                    LCD_upd();
                    break;
                case 1: //menu réglage palier 1
                    if (submenu == 0) {
                        if (theta1 < 99.75) theta1 += 0.25;
                    } else {
                        tempo1++;
                    }
                    LCD_upd();
                    break;
                case 2: //menu réglage palier 2
                    if (submenu == 0) {
                        if (theta2 < 99.75) theta2 += 0.25;
                    } else {
                        tempo2++;
                    }
                    LCD_upd();
                    break;
                case 3: //menu réglage palier 3
                    if (submenu == 0) {
                        if (theta3 < 99.75) theta3 += 0.25;
                    } else {
                        tempo3++;
                    }
                    LCD_upd();
                    break;
                case 4: //menu réglage palier 4
                    if (submenu == 0) {
                        if (theta4 < 99.75) theta4 += 0.25;
                    } else {
                        tempo4++;
                    }
                    LCD_upd();
                    break;
                case 5: //menu réglage Mash out - rinçage
                    if (thetaMO < 99.75) thetaMO += 0.25;
                    LCD_upd();
                    break;
                case 6: // ébullition
                    if (submenu == 0) {
                        if (theta5 < 110) theta5 += 0.25;
                    } else {
                        tempo5++;
                    }
                    LCD_upd();
                    break;
                case 8: //préchauffe
                    if (thetastart < 99.75) thetastart += 0.25;
                    LCD_upd();
                    break;
                case 9: //palier 1
                    if (theta1 < 99.75) theta1 += 0.25;
                    LCD_upd();
                    break;
                case 10: //palier 2
                    if (theta2 < 99.75) theta2 += 0.25;
                    LCD_upd();
                    break;
                case 11: //palier 3
                    if (theta3 < 99.75) theta3 += 0.25;
                    LCD_upd();
                    break;
                case 12: //palier 4
                    if (theta4 < 99.75) theta4 += 0.25;
                    LCD_upd();
                    break;
                case 13: //rinçage mash out
                    if (thetaMO < 99.75) thetaMO += 0.25;
                    LCD_upd();
                    break;
                case 14: //ébullition
                    if (theta5 < 110) theta5 += 0.25;
                    LCD_upd();
                    break;
                case 7: //sauvegarde
                case 15: //mode manuel
                case 16: //mode erreur
                    break;
            }
            break;
        case btnSELECT:
            switch (menu) {
                case 0: //menu température de préchauffe
                case 5: //menu rinçage
                    LCD_upd();
                    break;
                case 1: //menu palier 1
                case 2: //menu palier 2
                case 3: //menu palier 3
                case 4: //menu palier 4
                case 6: //ébullition
                    submenu = submenu == 0 ? 1 : 0;
                    LCD_upd();
                    break;
                case 7: //sauvegarde
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
                        printLogLineInFile(&myFile, "Palier 1 :", tempo1, theta1);
                        printLogLineInFile(&myFile, "Palier 2 :", tempo2, theta2);
                        printLogLineInFile(&myFile, "Palier 3 :", tempo3, theta3);
                        printLogLineInFile(&myFile, "Palier 4 :", tempo4, theta4);
                        myFile.print("Mash out :");
                        myFile.print(thetaMO);
                        myFile.print("°C\n");
                        printLogLineInFile(&myFile, "Ebullition :", tempo5, theta5);
                        myFile.print("\n\n");
                        myFile.print("Temps (s),Température cible °C,Température °C,PID (0-255)\n");
                        myFile.close();
                    }

                    printLineOne("    PROGRAMME   ");
                    printLineTwo("   SAUVEGARDE ! ");
                    delay(600);
                    digitalWrite(ledPin, LOW);
                    myPID.SetMode(AUTOMATIC);
                    
                    //////////////////////////////////
                    // on compte le temps que va prendre le programme
                    startprog = 1;
                    total_time = tempo1 + tempo2 + tempo3 + tempo4 + tempo5;

                    printLineOne("    BRASSAGE    ");
                    printLineTwo("    IMMINENT !  ");
                    delay(800);
                    printLineOne("Duree prevue :  ");
                    lcd.setCursor(0, 1); // move to position 0 on the second line
                    lcd.print(total_time);
                    lcd.print(" Min");
                    delay(1000);
                    lcd.clear();
                    digitalWrite(ledPin, LOW);
                    menu = 8; //saute au menu suivant (départ programme)
                    LCD_upd();
                    break;
                case 8: //palier 1
                case 9: //palier 2
                case 10: //palier 3
                    break;
            } //fin switch menu
            break;
        case btnNONE:
            break;
        default:
            break;
        } //fin switch lcd key
    } //fin du else (pas de saut)
}

void printLogLineInFile(File* file, const char* label, int duration, float temp) {
     file->print(label);
     file->print(duration);
     file->print("min - ");
     file->print(temp);
     file->print("°C\n");
}

void regle_chauffe() {
    if (small_Beep == 1) {
        BeepOn();
        small_Beep = 0;
        Beep_state = 1;
    } else if (Beep_state > 0) {
        Beep_state = 0;
        BeepOff();
    }

    if (tx_chauffe >= 250) {
        FULL_ON();
    } else if (tx_chauffe <= 5) {
        FULL_OFF();
    } else {
        attachInterrupt(0, zeroCrossingInterrupt, CHANGE); // Proportionnal
        Triac_pilot(Power_out(tx_chauffe));
    }
}

void zeroCrossingInterrupt() { //zero cross detect
    TCCR1B = 0x04;  //start timer with divide by 256 input
    TCNT1 = 0;      //reset timer - count from zero
}

ISR(TIMER1_COMPA_vect) { //comparator match
    digitalWrite(GATE, HIGH); //set triac gate to high
    TCNT1 = 65536 - PULSE;    //trigger pulse width
}

ISR(TIMER1_OVF_vect) { //timer1 overflow
    digitalWrite(GATE, LOW);  //turn off triac gate
    TCCR1B = 0x00;            //disable timer stopd unintended triggers
}

void Triac_pilot(int conduction_time) { // pilote le triac
    if (conduction_time > HALF_WAVE) conduction_time = HALF_WAVE;
    OCR1A = conduction_time;            //set the compare register brightness desired.
}

int Power_out(int Power) { //calcule le temps de conduction nécessaire pour obtenir la puissance demandée. Power va de 0 à 255
    return 540 - (Power * 2);
}

void FULL_ON() {
    detachInterrupt(0);       // set up zero crossing interrupt
    digitalWrite(GATE, HIGH); //IRQ0 is pin 2
}

void FULL_OFF() {
    detachInterrupt(0);       // set up zero crossing interrupt
    digitalWrite(GATE, LOW);  //IRQ0 is pin 2
}

void logRecord() {
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

void printParamInFile(File* file, const char* label, float value) {
    file->print(label);
    file->print("=");
    file->print(value);
    file->print(",");
}

void sauve_param() {
    digitalWrite(ledPin, 1);
    if (SD.exists("setup.grm")) {
        SD.remove("setup.grm");
    }
    File dataFile = SD.open("setup.grm", FILE_WRITE);
    if (dataFile) {
        printParamInFile(&dataFile, "THERMPIN2", THERM_PIN2);
        printParamInFile(&dataFile, "THERMPIN1", THERM_PIN);
        printParamInFile(&dataFile, "OFSETPRBE", OFFSET_PROBE);
        printParamInFile(&dataFile, "COEFPROBE", COEF_PROBE);
        printParamInFile(&dataFile, "hysterPos", hysteresis_Pos);
        printParamInFile(&dataFile, "hysterNeg", hysteresis_Neg);
        printParamInFile(&dataFile, "thetastrt", thetastart);
        printParamInFile(&dataFile, "theta0001", theta1);
        printParamInFile(&dataFile, "theta0002", theta2);
        printParamInFile(&dataFile, "theta0003", theta3);
        printParamInFile(&dataFile, "theta0004", theta4);
        printParamInFile(&dataFile, "theta0005", theta5);
        printParamInFile(&dataFile, "tempo0001", tempo1);
        printParamInFile(&dataFile, "tempo0002", tempo2);
        printParamInFile(&dataFile, "tempo0003", tempo3);
        printParamInFile(&dataFile, "tempo0004", tempo4);
        printParamInFile(&dataFile, "tempo0005", tempo5);
        printParamInFile(&dataFile, "thetaMO00", thetaMO);
        printParamInFile(&dataFile, "Pstrong00", P_strong);
        printParamInFile(&dataFile, "Istrong00", I_strong);
        printParamInFile(&dataFile, "Dstrong00", D_strong);
        printParamInFile(&dataFile, "Pweak0000", P_weak);
        printParamInFile(&dataFile, "Iweak0000", I_weak);
        printParamInFile(&dataFile, "Dweak0000", D_weak);
        printParamInFile(&dataFile, "LEDPIN000", ledPin);
        printParamInFile(&dataFile, "GATE00000", GATE);
        printParamInFile(&dataFile, "PULSE0000", PULSE);
        printParamInFile(&dataFile, "HALFWAVE0", HALF_WAVE);
        printParamInFile(&dataFile, "BEEPPIN00", Beep_PIN);
        printParamInFile(&dataFile, "PIDOFFSET", PID_OFFSET);
        printParamInFile(&dataFile, "PROBETYPE", probe);
        printParamInFile(&dataFile, "ONEWIRE00", ONE_WIRE_BUS);
    }
    digitalWrite(ledPin, 0);
}
