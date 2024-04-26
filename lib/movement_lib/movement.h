#include "MeAuriga.h"

// Levý motor
extern const int pwmMotorPravy;
extern const int inMotorPravy1;
extern const int inMotorPravy2;

// Pravý motor
extern const int pwmMotorLevy;
extern const int inMotorLevy1;
extern const int inMotorLevy2;

extern int rychlostJizdy;
extern int minRychlost;
extern int maxRychlost;

extern MeGyro gyro;

void levyMotorVpred(int rychlost);
void levyMotorVzad(int rychlost);
void levyMotorStop();
void pravyMotorVpred(int rychlost);
void pravyMotorVzad(int rychlost);
void pravyMotorStop();
void pohyb(int rychlostL, int rychlostR);
bool otacej_dokud_nenajdes_caru(byte position, int8_t smer);
void otacej_dle_offsetu(int offset);
void turn(int angle, int8_t smer);