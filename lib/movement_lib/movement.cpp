#include "Arduino.h"
#include "movement.h"



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


void pohyb(int rychlostL, int rychlostR){ // doleva - levy opacny
                                          // doprava - pravy opacny
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

void otacej_dokud_nenajdes_caru(byte position, int8_t smer){ // 1 - doleva, -1 - doprava
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
    pohyb(-120*smer, 120*smer);
  }
}

void otacej_dle_offsetu(int offset){
  if ( (offset < 10) & (-10 < offset)){
    pohyb(0,0);
  } else {
    pohyb(-120, 120);
  }

}
