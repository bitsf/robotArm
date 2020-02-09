#include "limiter.h"
#include <Arduino.h>

bool Limiter::isAvailable(float rot, float low, float high){
    // return true;
    if (rot< -PI/2.0 || rot > PI/2.0){
        return false;
    }
    if (low < -PI/4 || low>0.05){
        return false;
    }
    if (high > PI*2/3 || high<0){
        return false;
    }
    if (high - low < 0.9){
        return false;
    }
    if (low - high > 2 * PI / 3){
        return false;
    }
    return true;
}
