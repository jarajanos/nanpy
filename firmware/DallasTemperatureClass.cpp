#include <Arduino.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include "DallasTemperatureClass.h"
#include "MethodDescriptor.h"
#include <stdlib.h>

DallasTemperatureClass::DallasTemperatureClass():v(2) {

};

void DallasTemperatureClass::elaborate( MethodDescriptor* m ) {
    if (strcmp(m->getClass(), "DallasTemperature") == 0) {

        if (strcmp(m->getName(),"new") == 0) {       
            int prm = 0;
            OneWire* wr = new OneWire(m->getInt(0));
            v.insert(new DallasTemperature(&(*wr)));
            Serial.println(v.getLastIndex());
        }

        if (strcmp(m->getName(), "begin") == 0) {
            v[m->getObjectId()]->begin();
            Serial.println("0");
        }

        if (strcmp(m->getName(), "requestTemperatures") == 0) {
            v[m->getObjectId()]->requestTemperatures();
            Serial.println("0");
        }

        if (strcmp(m->getName(), "getTempCByIndex") == 0) {
            v[m->getObjectId()]->getTempCByIndex(m->getInt(0));
            Serial.println("0");
        }

        if (strcmp(m->getName(), "getTempFByIndex") == 0) {
            v[m->getObjectId()]->getTempFByIndex(m->getInt(0));
            Serial.println("0");
        }

        if (strcmp(m->getName(), "remove") == 0) {
            delete(v[m->getObjectId()]);
            v.remove(m->getObjectId());
            Serial.println("0");
        }

    }
};