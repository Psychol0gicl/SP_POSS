
#include "MeRGBLineFollower.h"
extern MeRGBLineFollower RGBLineFollower;

//Fyzikalni parametry PID na interni diskretni zesileni - bude vypocteno jen jednou pri inicializaci
extern volatile const float ci;
extern volatile const float cd1;
extern volatile const float cd2;

extern volatile float wk;    // pozad hodnota
//extern volatile float wkm1;  // minula pozad hodnota
extern volatile float yk;    // hodnota ze zpetne vazby - pravdepodobne offset z cidla
//extern volatile float ykm1;  // minula hodnota z cidla
extern volatile float ek;    // reg odchylka
extern volatile float ekm1;  // minula hodnota reg odchylky

//globalni promenne pro interni stav integratoru a derivatoru
extern volatile float yi; 
extern volatile float yd;
extern volatile float yp;

// Parametry spojiteho PID regulatoru - paralelni forma s filtrovanou D slozkou - potreba jen proporcionalni slozka
extern volatile float Kp;

extern volatile float uk; // vystup
extern volatile int rozdilPasu; // hodnota, ktera se pricte k jedne rychlosti a od druhe se odecte
extern byte smerJizdy;

void calc_pid(); // bude se pocitat pri timer interruptu

// zaloha puvodniho kodu =========================================================================
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
