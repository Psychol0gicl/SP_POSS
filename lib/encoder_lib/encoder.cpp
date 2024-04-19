#include "Arduino.h"
#include "encoder.h"

// enkoder pro pravy motor


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
