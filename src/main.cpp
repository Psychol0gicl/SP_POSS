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

volatile int rozdilPasu = 0; // hodnota, ktera se pricte k jedne rychlosti a od druhe se odecte

// Levý motor
const int pwmMotorPravy = 11;
const int inMotorPravy1 = 49;
const int inMotorPravy2 = 48;

// Pravý motor
const int pwmMotorLevy = 10;
const int inMotorLevy1 = 47;
const int inMotorLevy2 = 46;

//int rychlostJizdy = 80; //80-90 je sweet spot pro vetsinu robotu it seems
int crossSpeed = 80;
int forwardSpeed = 80;
int rychlostJizdy = forwardSpeed;
int zavodniRychlost = 150;
int rychlostOtaceni = 120 ;
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
std::stack<char> finished;
std::stack<int> tmp;

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

void dispCrossroad(char crossroad){
  LED(3, black);  // 0
  LED(6, black);  // 90
  //LED(9, black);  // 180
  LED(12, black); // 270
  switch(crossroad){
    case tecko: 
      LED(6, yellow);
      LED(12, yellow);
    break;
    case kriz:
      LED(3, yellow);
      LED(6, yellow);
      LED(12, yellow);
    break;
    case rovne_a_doleva:
      LED(3, yellow);
      LED(12, yellow);
    break;
    case rovne_a_doprava:
      LED(3, yellow);
      LED(6, yellow);
    break;
    case zatacka_L:
      LED(12, yellow);
    break;
    case zatacka_P:
      LED(6, yellow);
    break;
    case 'x': //prusvih
      for (int i =1; i<=12;i++){
        LED(i, red);  
      } 
    break;
  }
}

void vitezny_tanecek(){
  pohyb(0,0);
  buzzerOn();
  buzzer.tone(880 , 1000);

  for (int i =1; i<=12;i++){
    LED(i, blue); 
  }
  buzzerOff();
  delay(2000);
  for (int i =1; i<=12;i++){
    LED(i, red);  
  }
  buzzerOn();  
  buzzer.tone(440, 1000); 
  delay(2000);
  for (int i =1; i<=12;i++){
    LED(i, black);  
  }
  buzzerOn();  
  buzzer.tone(660, 1000);
  buzzerOff();
  pohyb(0,0);
}

int findMostFrequent(std::stack<int>& stack) {
    const int MAX_VALUE = 17; // Adjust this according to the range of integers in your stack
    int intCounts[MAX_VALUE] = {0}; // Initialize array to store counts

    // Count occurrences of each integer
    while (!stack.empty()) {
        int currentInt = stack.top();
        intCounts[currentInt]++;
        stack.pop();
    }

    // Find the most frequent integer
    int mostFrequentInt;
    int maxCount = 0;
    for (int i = 0; i < MAX_VALUE; ++i) {
        if (intCounts[i] > maxCount) {
            maxCount = intCounts[i];
            mostFrequentInt = i;
        }
    }

    return mostFrequentInt;
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
//   finished.push(kriz);
//   finished.push(zatacka_L);
//   finished.push(zatacka_L);
//   finished.push(rovne_a_doleva);
// finished.push(zatacka_P);
// finished.push(zatacka_P);
// finished.push(zatacka_L);
// finished.push(zatacka_P);
// finished.push(zatacka_L);
// finished.push(rovne_a_doleva);
// mapping = false;
}


byte position = 0;
float jas =  0;
int offset = 0;


#define forward     0
#define backward    1
#define crossroads  2
#define turnRight   3
#define turnLeft    4
#define turn180     5

byte state = forward;
bool mapping = true;

bool returning = false;

long start = 0;
bool started = false;
bool crossEnter = false;
bool firstCross = true;
int uMax = 60;

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

  if(mapping){// mapovaci rezim

    switch(state){

      case forward: //=============================================================================
        smerJizdy = 1;

        if(offset > uMax ) {offset = uMax;} //regulace
        else if(offset < -uMax) {offset = -uMax;}
        pohyb(smerJizdy*rychlostJizdy + smerJizdy*offset, smerJizdy*rychlostJizdy - smerJizdy*offset);

        if((position == 0b00000001) || (position == 0b00001000) || (position == 0b00000000)){ // krizovatka
          rychlostJizdy = crossSpeed;
          pohyb(rychlostJizdy,rychlostJizdy);
          distReset();
          previous = position;
          state = crossroads;
        }
        else if(position == 0b00001111){ // slepa
          pohyb(0,0);
          returning = true;
          state = turn180;
        }
      break;
      
      case crossroads:  //=============================================================================
        if(position == 0b00001101 || position == 0b00001011){ //chceme regulovat
          if(offset > uMax){offset = uMax;}
          else if(offset < -uMax){offset = -uMax;}
          pohyb(smerJizdy*rychlostJizdy + smerJizdy*offset, smerJizdy*rychlostJizdy - smerJizdy*offset);
          break;
        } // mezistavy - neni plne z krizovatky, ale ohlasil by zmenu

        if(previous == 0b00000000 && (position == 0b00001000 || position == 0b00000001)){break;} // nedetekoval by kriz, ktery tam ve skutecnosti je
        //if(position == 0b00000000 && (previous == 0b00001000 || previous == 0b00000001) && getDist() > 1){break;}
        if(position == 0b00001000 && previous == 0b00000001){break;}
        if(position == 0b00000001 && previous == 0b00001000){break;} // nove nevalidni stavy
        
        if(position == 0b00001001 || position == 0b00001111){ //vyjeli jsme z krizovatky
          pohyb(rychlostJizdy, rychlostJizdy);
          while(getDist() < 75){} 
          // delay(200);
          rychlostJizdy = forwardSpeed;
          if(firstCross){
            state = forward; 
            firstCross = false; 
            Serial.println("first"); 
            previous = -1;
            while(!tmp.empty()){
              tmp.pop();
            }
            break;
          } else{pohyb(0,0);}

          if(returning){ //mod vraceni se ze slepe --.--.--.--.--.--.--.--.--.--.--.--.--.--.--.--.--.--.--.--.--.--.--.--.--.--.--.--.--.--
            previous = position; 
            while(!tmp.empty()){
              tmp.pop();
            }
            char krizovatka = krizovatky.top();
            Serial.print("Returning Krizovatka: ");
            Serial.println(krizovatka);
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
            break; // konec case crossroads

          }
          else{ // normalni mod bez vraceni --.--.--.--.--.--.--.--.--.--.--.--.--.--.----.--.--.--.--.--.--.--.--.--.--.--.--.--.--
            previous = findMostFrequent(tmp); //nemam tuseni jestli tohle bude fungovat, ale za pokus nic nedam    
            Serial.print("Final Previous: ");
            Serial.print(previous, BIN);
            Serial.print("   ");
            Serial.print("Final Position: ");
            Serial.println(position, BIN);
            char krizovatka = detekce_krizovatky(previous, position);
            dispCrossroad(krizovatka);
            Serial.println(krizovatka);
            krizovatky.push(krizovatka);
            switch(krizovatka){
              case zatacka_L: state = turnLeft; break;
              case zatacka_P: state = turnRight; break;
              case rovne_a_doleva: state = forward; break;
              default: state = turnRight; break;
            }
            break; // konec case crossroads
          }
        }
        Serial.print("Previous: ");
        Serial.print(previous, BIN);
        Serial.print("   ");
        Serial.print("Position: ");
        Serial.println(position, BIN);

        previous = position;
        // if(!firstCross){
        //   tmp.push(previous);    
        // }
        tmp.push(previous);     // trying this instead of -^

        // previous = position; //proc je tohle az tady dole?
        
        if(getDist() > 50 && previous == 0b00000000){ // cil nalezen
          vitezny_tanecek();
          firstCross = true;
          pohyb(0,0);
          mapping = false;
          state = forward;
          rychlostJizdy = crossSpeed;

          while(!krizovatky.empty()){ // preskladani zasobniku
            finished.push(krizovatky.top());
            krizovatky.pop();
          }

          while(digitalRead(levyNaraznik)){ //pravy nefunguje
            position = RGBLineFollower.getPositionState();
            svit(position);
            LED(7, green); 
            LED(11, green);  
            }
          break;
        }

      break; // konec case crossroads

      case turnRight: //=============================================================================
        // offset = RGBLineFollower.getPositionOffset();
        if(!started){
          turn(45, 1); 
          started = true; 
          RGBLineFollower.loop();
          position = RGBLineFollower.getPositionState(); // HERE - ale nemelo by to funkci  otacej_dokud_nenajdes_caru() vadit
        }  

        // if(offset > uMax){offset = uMax;}
        // else if(offset < -uMax){offset = -uMax;}
        // pohyb(smerJizdy*rychlostJizdy + smerJizdy*offset, smerJizdy*rychlostJizdy - smerJizdy*offset);  
        if( otacej_dokud_nenajdes_caru(position, 1) ){started = false; state = forward; }
      break;

      case turnLeft: //=============================================================================
        // offset = RGBLineFollower.getPositionOffset();
        if(!started){
          turn(45, -1); 
          started = true; 
          RGBLineFollower.loop();
          position = RGBLineFollower.getPositionState(); // HERE
        } 
        // if(offset > uMax){offset = uMax;}
        // else if(offset < -uMax){offset = -uMax;}
        // pohyb(smerJizdy*rychlostJizdy + smerJizdy*offset, smerJizdy*rychlostJizdy - smerJizdy*offset);
        if( otacej_dokud_nenajdes_caru(position, -1) ){started = false; state = forward; }        
      break;

      case turn180:
        // offset = RGBLineFollower.getPositionOffset();
        if(!started){
          turn(160, 1); 
          started = true; 
          RGBLineFollower.loop();
          position = RGBLineFollower.getPositionState();
        }  
        // if(offset > uMax){offset = uMax;}
        // else if(offset < -uMax){offset = -uMax;}
        // pohyb(smerJizdy*rychlostJizdy + smerJizdy*offset, smerJizdy*rychlostJizdy - smerJizdy*offset);  
        if( otacej_dokud_nenajdes_caru(position, 1) ){started = false; state = forward; }

      break;
    }
  }
  
  // Zavodni rezim ---------------------------------------------------------------------------------
  else{   
    LED(7, aquamarine); 
    LED(11, aquamarine);  
   
    switch(state){

      case forward: //=============================================================================
        smerJizdy = 1;
        if(offset > uMax ) {offset = uMax;} //regulace
        else if(offset < -uMax) {offset = -uMax;}
        pohyb(smerJizdy*zavodniRychlost + smerJizdy*offset, smerJizdy*zavodniRychlost - smerJizdy*offset);

        if((position == 0b00000001) || (position == 0b00001000) || (position == 0b00000000)){ // krizovatka
          pohyb(zavodniRychlost,zavodniRychlost);
          distReset();
          previous = position;
          state = crossroads;
        }
        else if(position == 0b00001111){ // slepa
          pohyb(0,0);
          returning = true;
          state = turnRight;
        }
      break;

      
      case crossroads:  //=============================================================================        
        if(position == 0b00001101 || position == 0b00001011){ //chceme regulovat
          if(offset > uMax){offset = uMax;}
          else if(offset < -uMax){offset = -uMax;}
          pohyb(smerJizdy*rychlostJizdy + smerJizdy*offset, smerJizdy*rychlostJizdy - smerJizdy*offset);
          break;
        } // mezistavy - neni plne z krizovatky, ale ohlasil by zmenu
        if(previous == 0b00000000 && (position == 0b00001000 || position == 0b00000001)){break;} // nedetekoval by kriz, ktery tam ve skutecnosti je
        if(position == 0b00001000 && previous == 0b00000001){break;}
        if(position == 0b00000001 && previous == 0b00001000){break;} // nove nevalidni stavy
        if(position == 0b00001001 || position == 0b00001111){ //vyjeli jsme z krizovatky
          pohyb(rychlostJizdy, rychlostJizdy);
          while(getDist() < 75){} 
          if(firstCross){
            state = forward; 
            firstCross = false; 
            Serial.println("first"); 
            previous = -1;
            while(!tmp.empty()){ //tady je to jedno
              tmp.pop();
            }
            break;
            
          } else{pohyb(0,0);}
           // normalni mod bez vraceni --.--.--.--.--.--.--.--.--.--.--.--.--.--.----.--.--.--.--.--.--.--.--.--.--.--.--.--.--
            Serial.print("Final Previous: ");
            Serial.print(previous, BIN);
            Serial.print("   ");
            Serial.print("Final Position: ");
            Serial.println(position, BIN);
            char krizovatka =  finished.top();
            krizovatky.push(krizovatka);
            finished.pop();
            dispCrossroad(krizovatka);
            Serial.println(krizovatka);
            switch(krizovatka){
              case zatacka_L: state = turnLeft; break;
              case zatacka_P: state = turnRight; break;
              case rovne_a_doleva: state = forward; break;
              default: state = turnRight; break;
            }
            // delay(300);
            break; // konec case crossroads
          
        }
        Serial.print("Previous: ");
        Serial.print(previous, BIN);
        Serial.print("   ");
        Serial.print("Position: ");
        Serial.println(position, BIN);
        if(finished.empty() && position == 0b00000){ // cil nalezen
          vitezny_tanecek();
          pohyb(0,0);
          mapping = false;
          state = forward;
          firstCross = true;
          while(!krizovatky.empty()){ // preskladani zasobniku
            finished.push(krizovatky.top());
            krizovatky.pop();
          }

          while(digitalRead(levyNaraznik)){ //pravy nefunguje
          svit(position);
            LED(7, green); 
            LED(11, green);  
          }
             for (int i =1; i<=12;i++){
                    LED(i, black); 
            }
            svit(position);
          break;
        }

      break; // konec case crossroads

      case turnRight: //=============================================================================
        position = RGBLineFollower.getPositionState(); // HERE
        // offset = RGBLineFollower.getPositionOffset();
        if(!started){
          turn(45, 1); 
          started = true; 
          RGBLineFollower.loop();
          position = RGBLineFollower.getPositionState();
        }
        // if(offset > uMax){offset = uMax;}
        // else if(offset < -uMax){offset = -uMax;}
        // pohyb(smerJizdy*rychlostJizdy + smerJizdy*offset, smerJizdy*rychlostJizdy - smerJizdy*offset);
        if( otacej_dokud_nenajdes_caru(position, 1) ){started = false; state = forward; }
      break;

      case turnLeft: //=============================================================================
        position = RGBLineFollower.getPositionState(); // HERE
        // offset = RGBLineFollower.getPositionOffset(); 
        if(!started){
          turn(45, -1); 
          started = true; 
          RGBLineFollower.loop();
          position = RGBLineFollower.getPositionState();
        }
        // if(offset > uMax){offset = uMax;}
        // else if(offset < -uMax){offset = -uMax;}
        // pohyb(smerJizdy*rychlostJizdy + smerJizdy*offset, smerJizdy*rychlostJizdy - smerJizdy*offset);
        if( otacej_dokud_nenajdes_caru(position, -1) ){started = false; state = forward; }  
      break;

      case turn180:
        position = RGBLineFollower.getPositionState();
        // offset = RGBLineFollower.getPositionOffset();
        if(!started){
          turn(160, 1); 
          started = true; 
          RGBLineFollower.loop();
          position = RGBLineFollower.getPositionState();
        }  
        // if(offset > uMax){offset = uMax;}
        // else if(offset < -uMax){offset = -uMax;}
        // pohyb(smerJizdy*rychlostJizdy + smerJizdy*offset, smerJizdy*rychlostJizdy - smerJizdy*offset);  
        if( otacej_dokud_nenajdes_caru(position, 1) ){started = false; state = forward; }        
      break;
    }
  }
}