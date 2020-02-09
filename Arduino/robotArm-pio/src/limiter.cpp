#include "limiter.h"
#include <Arduino.h>

bool Limiter::isAvailable(float rot, float low, float high){
    if (rot< -PI/2.0 || rot > PI/2.0){
        return false;
    }
    if (low <PI/4 || low>PI*3/4){
        return false;
    }
    if (high < PI*11/18 || high>0){
        return false;
    }
    if (high - PI/2.0 < low){
        return false;
    }
    if (high + PI/2.0 > low){
        return false;
    }
}