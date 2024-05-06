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
volatile int rozdilPasu = 0; // hodnota, ktera se pricte k jedne rychlosti a od druhe se odecte

// Levý motor
const int pwmMotorPravy = 11;
const int inMotorPravy1 = 49;
const int inMotorPravy2 = 48;

// Pravý motor
const int pwmMotorLevy = 10;
const int inMotorLevy1 = 47;
const int inMotorLevy2 = 46;

int rychlostJizdy = 140;
int8_t smerJizdy = 1; // pro spravnou regulaci pri jizde rovne
int minRychlost = 100;
int maxRychlost = 255;

// typy krizovatek: + (kriz), T (tecko), 3 (tecko doleva), E (tecko doprava),
//  > (doprava - tam kam sipka ukazuje), < (doleva - tam kam sipka ukazuje) 
#define kriz '+'
#define tecko 'T'
#define rovne_a_doleva  '3'
#define rovne_a_doprava 'E' // define bez stredniku
#define zatacka_L '<'
#define zatacka_P '>'
#define rovne '|'

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
#define black      000,000,000
#define white      255,255,255

void LED(byte num, int R, int G, int B){
  ledRing.setColor(num, R,G,B);
  ledRing.show();
}


// bzučák
const byte buzzerPin = 45;
MeBuzzer buzzer;

// Gyro
MeGyro gyro(1,0x69);

// pro enkodery
volatile bool encMSG = false;
volatile int pulseCountR = 0;
volatile bool stateAR = false;
volatile bool stateBR = false;
volatile bool oldStateAR = false;
volatile int pulseCountL = 0;
volatile bool stateAL = false;
volatile bool stateBL = false;
volatile bool oldStateAL = false;

byte current = -1;
byte previous = -1;
std::stack<char> krizovatky;
std::stack<char> temp;

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
  RGBLineFollower.setKp(0.8);

  // inicializace sériového kanálu
  Serial.begin(9600);

  while (digitalRead(levyNaraznik)) {
    RGBLineFollower.loop();
    svit(RGBLineFollower.getPositionState());
    // nepokracuj dokud neni stiknut levy naraznik
  }

  // inicializace casovace pro regulator
  // Timer3.initialize(int(Ts*1000000));
  // Timer3.attachInterrupt(calc_pid); 
}


byte position = 0;
float jas =  0;
int offset = 0;


#define forward     0
#define backward    1
#define crossroads  2
#define turnRight   3
#define turnLeft    4

byte state = forward;
bool mapping = true;
bool returning = false;

long start = 0;
bool started = false;
bool crossEnter = false;
bool firstCross = true;
int uMax = 50;

void loop() {
  // sejmutí dat z detektoru cary
  RGBLineFollower.loop();
  
  position = RGBLineFollower.getPositionState();

  svit(position);
  if(returning){LED(8, red); LED(10,red);}
  else{LED(8, black); LED(10,black);}

  switch(position){// necessary evil
    case 0b1000: position = 0b0001; break; 
    case 0b0001: position = 0b1000; break; 
  }

  offset = RGBLineFollower.getPositionOffset();
  yk = offset;

  if(abs(rozdilPasu) >= 20){LED(9, yellow);}
  else {LED(9, black);}

  // otacej_dokud_nenajdes_caru(position);
  // otacej_dle_offsetu(offset);
  
  if(mapping){// mapovaci rezim

    switch(state){

      case forward: //=============================================================================
        //Timer3.resume(); 
        smerJizdy = 1;
        if(offset > uMax){offset = uMax;}
        else if(offset < -uMax){offset = -uMax;}
        pohyb(smerJizdy*rychlostJizdy + smerJizdy*offset, smerJizdy*rychlostJizdy - smerJizdy*offset);

        if((position == 0b00000001) || (position == 0b00001000) || (position == 0b00000000)){ // krizovatka
          //Timer3.stop();
          crossEnter = true;
          pohyb(rychlostJizdy,rychlostJizdy);
          start = millis();
          distReset();
          state = crossroads;
        }
        else if(position == 0b00001111){ // slepa
          //Timer3.stop();
          pohyb(0,0);
          returning = true;
          state = turnRight;
        }
      break;

      case backward:  //=============================================================================
        //Timer3.resume();
        smerJizdy = -1;
      break;

      case crossroads:  //=============================================================================

        if(crossEnter){
          while(getDist() < 1.0){}
          current = position; 
          crossEnter = false;
        }

        if(millis() - start > 100 && previous == 0b00000000){ // cil nalezen
          pohyb(-150, 150);
          for (int i =1; i<=12;i++){
            LED(i, blue); 
          }
          delay(2000);
          for (int i =1; i<=12;i++){
            LED(i, red);  
          }    
          delay(2000);
          for (int i =1; i<=12;i++){
            LED(i, black);  
          }  
          pohyb(0,0);
          mapping = false;
          state = forward;

          while(!krizovatky.empty()){ // preskladani zasobniku
            temp.push(krizovatky.top());
            krizovatky.pop();
          }
          while(!temp.empty()){
            krizovatky.push(krizovatky.top());
            krizovatky.pop();
          }

          while(digitalRead(pravyNaraznik)){}// cekani na pravy naraznik
          break;
        }
        if(position == 0b00001011){break;}
        if(position == 0b00001101){break;} // mezistavy - neni plne z krizovatky, ale ohlasil by zmenu
        previous = current;
        current = position;

        Serial.print(previous, BIN);
        Serial.print("   ");
        Serial.println(current, BIN);
        if(detekce_zmeny_od_position(previous, current)){
          if(firstCross){state = forward; firstCross = false; break;}
          pohyb(rychlostJizdy,rychlostJizdy);
          while(getDist() < 15.0){}
          pohyb(0,0);
          char krizovatka = detekce_krizovatky(previous, current);
          Serial.println(krizovatka);
          if(returning){

            switch(krizovatka){
              case zatacka_L: state = turnLeft; break;
              case zatacka_P: state = turnRight; break;

              default:
                krizovatka = krizovatky.top();
                krizovatky.pop();
                switch(krizovatka){
                  case zatacka_L: state = turnRight; break;
                  case zatacka_P: state = turnLeft; break;
                  case rovne: state = forward; break; 

                  case tecko:
                    krizovatky.push(zatacka_L);
                    returning = false;
                    state = forward;
                  break;

                  case kriz:
                    krizovatky.push(rovne_a_doleva);
                    returning = false;
                    state = turnRight;
                  break;

                  case rovne_a_doleva:
                    krizovatky.push(zatacka_L);
                    returning = false;
                    state = turnRight;
                  break;

                  case rovne_a_doprava:
                    krizovatky.push(rovne);
                    returning = false;
                    state = turnRight;
                  break;
                }
              break;
            }
    
          }
          else{
            
            switch(krizovatka){
              case zatacka_L: state = turnLeft; break;
              case zatacka_P: state = turnRight; break;

              case rovne_a_doleva: 
                krizovatky.push(krizovatka);
                state = forward; 
              break;

              default:
                krizovatky.push(krizovatka);
                state = turnRight;
              break;
            }

          }
        }
      break;

      case turnRight: //=============================================================================
        if(!started){turn(85, 1); started = true;}
        if( otacej_dokud_nenajdes_caru(position, 1) ){started = false; state = forward;}
      break;

      case turnLeft: //=============================================================================
        if(!started){turn(85, -1); started = true;}
        if( otacej_dokud_nenajdes_caru(position, -1) ){started = false; state = forward; }
      break;
    }









  }
  else{
    switch(state){
    }
  }
}