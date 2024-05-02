
#include "MeRGBLineFollower.h"

#define kriz '+';
#define tecko 'T';
#define rovne_a_doleva  '3';
#define rovne_a_doprava 'E';
#define zatacka_L '<';
#define zatacka_P '>';

extern MeRGBLineFollower RGBLineFollower;
char detece_krizovatky(byte position);
byte detekce_zmeny(byte position);
byte detekce_zmeny_od_previous(byte previous, byte current);