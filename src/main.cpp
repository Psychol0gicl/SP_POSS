#include <Arduino.h>
#include "MeAuriga.h"
#include "MeRGBLineFollower.h"
#include "TimerThree.h"
#include "ArduinoSTL.h"
#include <stack>

#include "movement.h"
#include "encoder.h"
#include "regulation.h"
#include "krizovatky.h"

// mame robota cislo 11

// Parametry spojiteho PID regulatoru - paralelni forma s filtrovanou D slozkou, pro rychlost 120
volatile float Kp = 0.3;
float Ki = 1;
float Kd = 0;   // D slozka je vypnuta
float Tf = 0; 
float Ts = 0.01; // perioda vzorkovani pro vypocet param regulatoru

//Fyzikalni parametry PID na interni diskretni zesileni
volatile const float ci=Ki*Ts/2; 
volatile const float cd1=-(Ts-2*Tf)/(Ts+2*Tf); 
volatile const float cd2=2*Kd/(Ts+2*Tf);

volatile float wk = 0;    // pozad hodnota
volatile float yk = 0;    // hodnota ze zpetne vazby - pravdepodobne offset z cidla
volatile float ek = 0;    // reg odchylka
volatile float ekm1 = 0;  // minula hodnota reg odchylky

//globalni promenne pro interni stav integratoru a derivatoru
volatile float yi = 0; 
volatile float yd = 0;
volatile float yp = 0;
volatile float uk = 0; // vystup regulatoru

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

// typy krizovatek: + (kriz), T (tecko), 3 (tecko doleva), E (tecko doprava),
//  > (doprava - tam kam sipka ukazuje), < (doleva - tam kam sipka ukazuje) 
#define kriz '+';
#define tecko 'T';
#define rovne_a_doleva  '3';
#define rovne_a_doprava 'E';
#define zatacka_L '<';
#define zatacka_P '>';

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

/*
int offset = 0;

float ci = 0;
float cd1 = 0;
float cd2 = 0;

float wk = 0;
float wkm1 = 0;
float yk = 0;
float ykm1 = 0;
float ek = 0;
float ekm1 = 0;

//globalni promenne pro interni stav integratoru a derivatoru
float yi = 0; 
float yd = 0;

// Parametry spojiteho PID regulatoru - paralelni forma s filtrovanou D slozkou
float Kp = 33;
float Ki = 38;
float Kd = 6.6;
float Tf = 0.04;

float calc_pid(float wk, float wkm1, float yk, float ykm1){
  //Fyzikalni parametry PID na interni diskretni zesileni
  ci=Ki*Ts/2; 
  cd1=-(Ts-2*Tf)/(Ts+2*Tf); 
  cd2=2*Kd/(Ts+2*Tf);
    
  //Reg. odchylka
  ek = wk - yk;
  ekm1 = wkm1 - ykm1;
    
  //Akcni zasah po slozkach
  yi = yi + ci*(ek+ekm1);
  yd = cd1*yd + cd2*(ek-ekm1);
  yp = Kp*ek;

  return(yp+yi+yd); //celkovy aktualni akcni zasah - vystup regulatoru
}
*/

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

  // inicializace casovace pro regulator
  Timer3.initialize(int(Ts*1000000));
  Timer3.attachInterrupt(calc_pid); 
  

  while (digitalRead(levyNaraznik)) {
    // nepokracuj dokud neni stiknut levy naraznik
  }
  Timer3.start(); 
  // pohyb(100, 100);

  std::stack<char> krizovatky;
  
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