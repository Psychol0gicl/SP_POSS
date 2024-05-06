#include "Arduino.h"

extern volatile bool encMSG;
extern volatile int pulseCountR;
extern volatile bool stateAR;
extern volatile bool stateBR;
extern volatile bool oldStateAR;
extern volatile int pulseCountL;
extern volatile bool stateAL;
extern volatile bool stateBL;
extern volatile bool oldStateAL;

extern const byte pravyEnkoderA; // INT2
extern const byte pravyEnkoderB; // no interrupts :(

// enkoder pro levy motor
extern const byte levyEnkoderA; // INT3
extern const byte levyEnkoderB; // no interrupts :(

extern const int eckoderPulseNumber;
extern const float motorGearRatio;

extern volatile long pulseCountVlevo;
extern volatile long pulseCountVpravo;

void levyEncoderAInt();
void pravyEncoderAInt();
double getDist();