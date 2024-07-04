
// PAA3905 & HUB5168+ arduino demo program
// design by Tom,Yen 2024/03/29

#include "hub5168p.h"
#include "PAA3905_lib.h"

#define DEBUG

#ifdef DEBUG
#ifndef DEBUG_H
#define DEBUG_H
  #include <stdio.h> 
  #include <stdarg.h>

  #define DBGI Serial.begin(115200)
  #define BRK {char bugSbuf[128]; \
    sprintf(bugSbuf,"Break @%s:%d",__FILE__,__LINE__); \
    Serial.println((String)bugSbuf); \
    while (0==Serial.available()); \
    while (Serial.available()) Serial.read(); \
    Serial.println("Go!!");}
  
  #define DBG(...) {char bugSbuf[250];sprintf(bugSbuf,__VA_ARGS__);Serial.println((String)bugSbuf);}
#endif  // DEBUG_H 
#endif  // DEBUG

#define BRUST_READ 1

PAA3905 paa;
long timer = 0;

void setup() {
    Serial.begin(115200);
        
    paa.begin();
    
    delay(500);

    Serial.print("PAA3905");
    Serial.println();
}

void loop() {   
 
 if(millis() - timer > 100){ // print data every 0.1 second
    int dx,dy,op;   // op 0: nothing, 1: high light 2: low light 3:very low light 
    uint8_t sql;
    uint32_t sht;

#ifndef BRUST_READ
    if (op=paa.readMotion(&dx,&dy)){
        paa.getSqualShutter(&sql,&sht);

    }

#else

    motionDataT mD;
        
    if (op=paa.readMotion(&mD)){
        paa.getSqualShutter(&sql,&sht);
        dx = ((int16_t)mD.deltaX_H << 8) | mD.deltaX_L; 
        dy = ((int16_t)mD.deltaY_H << 8) | mD.deltaY_L;
        
        if (((uint16_t)dx & 0x8000) )
        {
            dx =  (uint16_t)dx-65535;
            
        }

        if (((uint16_t)dy & 0x8000) )
        {
            dy =  (uint16_t)dy-65535;
            
        }        

    } 
    
#endif
    if(op)
        DBG("OP=%d Sql=%d Sht=%d Dx=%d Dy=%d",op,sql,sht,dx,dy);
 
        timer = millis();
 }
    //delay(24);

}
