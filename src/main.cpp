#include <Arduino.h>
#include "MeAuriga.h"
#include "MeRGBLineFollower.h"

// mame robota cislo 11

// Levý motor
const int pwmMotorPravy = 11;
const int inMotorPravy1 = 49;
const int inMotorPravy2 = 48;

// Pravý motor
const int pwmMotorLevy = 10;
const int inMotorLevy1 = 47;
const int inMotorLevy2 = 46;

int rychlostJizdy = 200;
int minRychlost = 100;
int maxRychlost = 255;

// Ultrazvukovy snimac
// pouziti: vzdalenost = sonar.distanceCm()
MeUltrasonicSensor sonar(PORT_10);

// Snimac cary
// pouziti: linState = RGBLineFollower.getPositionState();
//          lineOffset = RGBLineFollower.getPositionOffset();
MeRGBLineFollower RGBLineFollower(PORT_9);

// Servo
const byte servoPin = 68;
const byte servoMin = 13;
const byte servoMax = 137;
Servo servo;

// narazniky
const byte pravyNaraznik = 67;
const byte levyNaraznik = 62;
volatile boolean narazVpravo = false;
volatile boolean narazVlevo = false;

// promenne pro enkodery
// enkoder pro pravy motor
const byte pravyEnkoderA = 19; // INT2
const byte pravyEnkoderB = 42; // no interrupts :(

// enkoder pro levy motor
const byte levyEnkoderA = 18; // INT3
const byte levyEnkoderB = 43; // no interrupts :(

const int eckoderPulseNumber = 9;
const float motorGearRatio = 39.267;

volatile long pulseCountVlevo = 0;
volatile long pulseCountVpravo = 0;

// RGB LED ring
const byte numberOfLEDs = 12;
const byte rgbLEDringPin = 44;
#define RINGALLLEDS        0
MeRGBLed ledRing(0, numberOfLEDs );

#define amber      255,194,000
#define orange     255,165,000
#define vermillion 227,066,052
#define red        255,000,000
#define magenta    255,000,255
#define purple     128,000,128
#define indigo     075,000,130
#define blue       000,000,255
#define aquamarine 127,255,212
#define green      000,255,000
#define chartreuse 127,255,000
#define yellow     255,255,000
#define white      000,000,000
#define black      255,255,255

void LED(byte num, int R, int G, int B){
  ledRing.setColor(num, R,G,B);
  ledRing.show();
}


// bzučák
const byte buzzerPin = 45;
MeBuzzer buzzer;

// Gyro
MeGyro gyro(1,0x69);

volatile bool encMSG = false;
volatile int pulseCountR = 0;
volatile bool stateAR = false;
volatile bool stateBR = false;
volatile bool oldStateAR = false;
volatile int pulseCountL = 0;
volatile bool stateAL = false;
volatile bool stateBL = false;
volatile bool oldStateAL = false;
// osetreni preruseni od kanalu A enkoderu na pravem motoru - POZOR - enkoder se toci opacne
void pravyEncoderAInt() {
  cli(); // zakaz zpracovani dalsich preruseni
  stateAR = digitalRead(pravyEnkoderA); // nacteni stanu kanalu A
  if (digitalRead(pravyEnkoderB)){ // vyhodnot pro B=HIGH
  if ( (oldStateAR - stateAR) < 0) // jaky je typ hrany na kanalu A
  pulseCountR--; // vzestupna -> pricti puls
  else
  pulseCountR++; // sestupna -> odecti puls
  } else { // vyhodnot pro B=LOW
  if ( (oldStateAR - stateAR) > 0) // jaky je typ hrany na kanalu A
  pulseCountR--; // sestupna -> pricti puls
  else
  pulseCountR++; // vzestupna -> odecti puls
  }
  oldStateAR = stateAR; // uchovej stav kanalu A pro pristi test hrany
  encMSG = true;
  sei(); // opet povol zpracovani dalsich preruseni
}


// osetreni preruseni od kanalu A enkoderu na levem motoru
void levyEncoderAInt() { 
  cli(); // zakaz zpracovani dalsich preruseni
  stateAL = digitalRead(levyEnkoderA); // nacteni stanu kanalu A
  if (digitalRead(levyEnkoderB)){ // vyhodnot pro B=HIGH
  if ( (oldStateAL - stateAL) < 0) // jaky je typ hrany na kanalu A
  pulseCountL++; // vzestupna -> pricti puls
  else
  pulseCountL--; // sestupna -> odecti puls
  } else { // vyhodnot pro B=LOW
  if ( (oldStateAL - stateAL) > 0) // jaky je typ hrany na kanalu A
  pulseCountL++; // sestupna -> pricti puls
  else
  pulseCountL--; // vzestupna -> odecti puls
  }
  oldStateAL = stateAL; // uchovej stav kanalu A pro pristi test hrany
  encMSG = true;
  sei(); // opet povol zpracovani dalsich preruseni
}

void levyMotorVpred(int rychlost) {
  digitalWrite(inMotorLevy1, HIGH);
  digitalWrite(inMotorLevy2, LOW);
  analogWrite(pwmMotorLevy, rychlost);
}

void levyMotorVzad(int rychlost) {
  digitalWrite(inMotorLevy1, LOW);
  digitalWrite(inMotorLevy2, HIGH);
  analogWrite(pwmMotorLevy, rychlost);
}

void levyMotorStop() {
  analogWrite(pwmMotorLevy, 0);
}

void pravyMotorVpred(int rychlost) {
  digitalWrite(inMotorPravy1, LOW);
  digitalWrite(inMotorPravy2, HIGH);
  analogWrite(pwmMotorPravy, rychlost);
}

void pravyMotorVzad(int rychlost) {
  digitalWrite(inMotorPravy1, HIGH);
  digitalWrite(inMotorPravy2, LOW);
  analogWrite(pwmMotorPravy, rychlost);
}

void pravyMotorStop() {
  analogWrite(pwmMotorPravy, 0);
}


void setup() {
  // nastav piny narazniku
  pinMode(pravyNaraznik,INPUT_PULLUP);
  pinMode(levyNaraznik,INPUT_PULLUP);

  // nastavení ovladacích pinů motorů jako výstupní
  pinMode(pwmMotorPravy,OUTPUT);
  pinMode(inMotorPravy1,OUTPUT);
  pinMode(inMotorPravy2,OUTPUT);

  pinMode(pwmMotorLevy,OUTPUT);
  pinMode(inMotorLevy1,OUTPUT);
  pinMode(inMotorLevy2,OUTPUT);

  // Nastavení frekvencep pwm na 8KHz pro řízení DC motorů
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  // inicializace enkoderu
  pinMode(pravyEnkoderA,INPUT_PULLUP);
  pinMode(pravyEnkoderB,INPUT_PULLUP);
  pinMode(levyEnkoderA,INPUT_PULLUP);
  pinMode(levyEnkoderB,INPUT_PULLUP);

  // inicializace obsluhy preruseni od kanalů A enkoderů
  attachInterrupt(digitalPinToInterrupt(pravyEnkoderA),&pravyEncoderAInt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(levyEnkoderA),&levyEncoderAInt, CHANGE);

  // pripoj a omez servo
  servo.attach(servoPin);//,servoMin,servoMax);
  servo.write(90);

  // inicializace RGB LED ringu
  // pro ovládání slouží metoda
  // bool MeRGBLed::setColor(uint8_t index, uint8_t red, uint8_t green, uint8_t blue)
  ledRing.setpin( rgbLEDringPin );
  ledRing.setColor( RINGALLLEDS, 0, 0, 0);
  ledRing.show();

  // nastavení bzučáku
  buzzer.setpin(buzzerPin);
  buzzer.noTone();

  // inicializace gyra
  //gyro.begin();
  
  // inicializace sledovani cary
  RGBLineFollower.begin();
  RGBLineFollower.setKp(1);

  // inicializace sériového kanálu
  Serial.begin(9600);

  while (digitalRead(levyNaraznik)) {
    // nepokracuj dokud neni stiknut levy naraznik
  }
  
  // pohyb(100, 100);

}

void pohyb(int rychlostL, int rychlostR){ // pro otaceni doleva - levy zaporny
                                          // pro otaceni doprava - pravy zaporny

  if(rychlostL < 0){
    levyMotorVzad(abs(rychlostL));
    }
  else{
    levyMotorVpred(rychlostL);
    }

  if(rychlostR < 0){
    pravyMotorVzad(abs(rychlostR));
    }
  else{
    pravyMotorVpred(rychlostR);
    }
}


void otacej_dokud_nenajdes_caru(byte position){ 
  // 1001 jsme na care
  // 1111 jsme mimo caru
  // 0000 krizovatka
  // byte cara = 0b1001;
  // obsolete => otacej dle offsetu 
  boolean leva = (position & 0b01000); 
  boolean stred1 = (position & 0b00100);
  boolean stred2 = (position & 0b00010);
  boolean prava = (position & 0b00001);
  if ( leva & !stred1 & !stred2 & prava ){
    pohyb(0,0);
  } else {
    pohyb(-120, 120);
  }
}

void otacej_dle_offsetu(int offset){
  if ( (offset < 10) & (-10 < offset)){
    pohyb(0,0);
  } else {
    pohyb(-120, 120);
  }

}

void svit(byte position){
  if(0b00001000 & position){
      LED(5, red);
      }
    else{
      LED(5, blue);
      }

    if(0b00000100 & position){
      LED(4, red);
      }
    else{
      LED(4, blue);
      }

    if(0b00000010 & position){
      LED(2, red);
      }
    else{
      LED(2, blue);
      }
    if(0b00000001 & position){
      LED(1, red);
      }
    else{
      LED(1, blue);
    }
}
  /*
  LED(1, amber*0.5); // 300
  LED(2, orange*0.5); // 330
  LED(3, vermillion*0.5); // 0
  LED(4, red*0.5);  // 30
  LED(5, magenta*0.5); // 60
  LED(6, purple*0.5); // 90
  LED(7, indigo*0.5); // 120
  LED(8, blue*0.5);   // 150
  LED(9, aquamarine*0.5); // 180
  LED(10, green*0.5);     // 210
  LED(11, chartreuse*0.5);// 240
  LED(12, yellow*0.5);    // 270
  */

  byte position = 0;
  float jas =  0;
  int offset = 0;


void loop() {
  // sejmutí dat z detektoru cary
  RGBLineFollower.loop();

  delay(10);
  
  position = RGBLineFollower.getPositionState();
  offset = RGBLineFollower.getPositionOffset();

  svit(position);
  // otacej_dokud_nenajdes_caru(position);
  otacej_dle_offsetu(offset);
  
}