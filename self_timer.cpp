#include <MsTimer2.h>
#include "self_timer.hpp"


static int is_timer_over;

/* Constructor */
SelfTimer::SelfTimer(){};

/* DeConstructor */
SelfTimer::~SelfTimer(){};

void SelfTimer::create(int delta_t){
    // SelfTimer set
    MsTimer2::set(delta_t, SelfTimer::elapsed);
};

void SelfTimer::start(void){
    // SelfTimer start
    MsTimer2::start();
};

void SelfTimer::elapsed(void){
    is_timer_over = true;
}

int SelfTimer::is_over(void){
    return is_timer_over;
}

void SelfTimer::reset(void){
    is_timer_over = false;
    return;
}
