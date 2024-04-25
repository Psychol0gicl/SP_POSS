// typy krizovatek: + (kriz), T (tecko), 3 (tecko doleva), E (tecko doprava), > (doprava - tam kam sipka ukazuje), < (doleva - tam kam sipka ukazuje) 
#include "Arduino.h"
#include "krizovatky.h"
#include "MeRGBLineFollower.h"



char detekce_krizovatky(byte on, byte off){ //on = co bylo na krizovatce, off = co bylo mimo krizovatku
    boolean on_leva = (on & 0b01000); 
    boolean on_stred1 = (on & 0b00100);
    boolean on_stred2 = (on & 0b00010);
    boolean on_prava = (on & 0b00001);

    boolean off_leva = (off & 0b01000); 
    boolean off_stred1 = (off & 0b00100);
    boolean off_stred2 = (off & 0b00010);
    boolean off_prava = (off & 0b00001);


    if (on == off ){ // jsme stele stejne
        return -1; // nevim co jsem dat za char tak tam je zatim -1
    } 
    if(!on_leva & !on_stred1 & !on_stred2 & !on_prava){ // vidim jen cernou
        if (off_leva & !off_stred1 & !off_stred2 & off_prava){ // vidim caru
            return kriz;
        }
        else if (off_leva & off_stred1 & off_stred2 & off_prava) //vidim bilo
        {
            return tecko;
        }
    }
    //TODO: ostatni kombinace
}

 
byte detekce_zmeny(byte position){ 
    boolean leva = (position & 0b01000); 
    boolean stred1 = (position & 0b00100);
    boolean stred2 = (position & 0b00010);
    boolean prava = (position & 0b00001);
    if (leva & !stred1 & !stred2 & prava){ // jsme na care
        return -1;
    }
    else{ //doslo ke zmene
        position = RGBLineFollower.getPositionState();
        return position;
    }
        
}

byte detekce_zmeny_od_previous(byte previous, byte current){
    if(previous == current){
        return -1;
    } else{
        return current;
    }
}
    