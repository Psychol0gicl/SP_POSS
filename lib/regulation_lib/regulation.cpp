#include "Arduino.h"
#include "regulation.h"
#include "MeRGBLineFollower.h"

void calc_pid(){    // bude se pocitat pri timer interruptu
  cli(); // zakaz zpracovani dalsich preruseni
  //Reg. odchylka
  yk = RGBLineFollower.getPositionOffset(); // zatim nevim, jestli to zjistovat zde nebo mimo
  ek = wk - yk;
  //ekm1 = wkm1 - ykm1;   netreba znova pocitat, jsou to minule hodnoty
    
  //Akcni zasah po slozkach
  yi = yi + ci*(ek+ekm1);
  yd = cd1*yd + cd2*(ek-ekm1);
  yp = Kp*ek;

  uk = yp+yi+yd; //celkovy aktualni akcni zasah - vystup regulatoru
  ekm1 = ek;
  sei(); // opet povol zpracovani dalsich preruseni
}