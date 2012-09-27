#include <stdlib.h>

#include "ArduinoClass.h"
#include "ArduinoSingle.h"
#include "OneWireClass.h"
#include "LiquidCrystalClass.h"
#include "MethodDescriptor.h"
#include "Utilities.h"
#include "SlimArray.h"

SlimArray <ArduinoClass*> classes(2);

int pin, value;

MethodDescriptor *m = NULL;

void elaborate()
{

    if(m != NULL) {
        delete(m);
        m = NULL;
    }

    if (Serial.available() > 0) {

        m = new MethodDescriptor();

        for(int i = 0 ; i < classes.getSize() ; i++)
            classes[i]->elaborate(m);

    }

}

void setup() {
    classes.insert(ArduinoSingle::getInstance());
    classes.insert(new OneWireClass());
    classes.insert(new LiquidCrystalClass());
    Serial.begin(9600);
    while (Serial.available() <= 0) {
        delay(300);
    }
}

void loop(){
    elaborate();
}

