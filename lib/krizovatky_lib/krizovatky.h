#include "MeRGBLineFollower.h"

#define kriz '+';
#define tecko 'T';
#define rovne_a_doleva  '3';
#define rovne_a_doprava 'E';
#define zatacka_L '<';
#define zatacka_P '>';

extern MeRGBLineFollower RGBLineFollower;
char detekce_krizovatky(byte on, byte off);
byte detekce_zmeny(byte position);
boolean detekce_zmeny_od_position(byte position, byte current);