
#include "regulation.h"


void calc_pid(){    // bude se pocitat pri timer interruptu
  cli(); // zakaz zpracovani dalsich preruseni
  //Reg. odchylka
  //yk = RGBLineFollower.getPositionOffset(); // <-512, 512>, nevim, jestli minus znamena vychylku doleva
  ek = wk - yk;
  //ekm1 = wkm1 - ykm1;   netreba znova pocitat, jsou to minule hodnoty
    
  //Akcni zasah po slozkach
  yi = yi + ci*(ek+ekm1);
  yd = cd1*yd + cd2*(ek-ekm1);
  yp = Kp*ek;

  uk = yp+yi+yd; //celkovy aktualni akcni zasah - vystup regulatoru
  ekm1 = ek;
  rozdilPasu = (int)(uk/2.0);
  pohyb(smerJizdy*rychlostJizdy - smerJizdy*rozdilPasu, smerJizdy*rychlostJizdy + smerJizdy*rozdilPasu);
  sei(); // opet povol zpracovani dalsich preruseni
}