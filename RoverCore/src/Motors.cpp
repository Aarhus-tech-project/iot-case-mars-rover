#include "Motors.h"
#include <iostream>

#if defined(_WIN32) || defined(__APPLE__) || defined(__x86_64__)
// Minimal stub for PC/Mac: do nothing but allow compilation
inline int gpioInitialise() { return 0; }
inline void gpioTerminate() {}
inline void gpioSetMode(int, int) {}
inline void gpioWrite(int, int) {}
#define PI_OUTPUT 1
#else
#include <pigpio.h>
#endif

Motors::Motors() {
    // Initialize pigpio (does nothing on PC/Mac)
    if (gpioInitialise() < 0) {
        std::cerr << "[motors] pigpio initialization failed!" << std::endl;
        return;
    }

    // BCM GPIO pins
    IA1 = 4;   // GPIO4
    IA2 = 17;  // GPIO17
    IB1 = 23;  // GPIO23
    IB2 = 24;  // GPIO24

    // Set pins as output
    gpioSetMode(IA1, PI_OUTPUT);
    gpioSetMode(IA2, PI_OUTPUT);
    gpioSetMode(IB1, PI_OUTPUT);
    gpioSetMode(IB2, PI_OUTPUT);

    stop(); // start with motors off
}

Motors::~Motors() {
    stop();
#if !(defined(_WIN32) || defined(__APPLE__) || defined(__x86_64__))
    gpioTerminate();
#endif
}

void Motors::forward() {
    gpioWrite(IA1, 1); gpioWrite(IA2, 0);
    gpioWrite(IB1, 1); gpioWrite(IB2, 0);
}

void Motors::reverse() {
    gpioWrite(IA1, 0); gpioWrite(IA2, 1);
    gpioWrite(IB1, 0); gpioWrite(IB2, 1);
}

void Motors::left() {
    gpioWrite(IA1, 0); gpioWrite(IA2, 1); // reverse left
    gpioWrite(IB1, 1); gpioWrite(IB2, 0); // run right
}

void Motors::right() {
    gpioWrite(IA1, 1); gpioWrite(IA2, 0); // run left
    gpioWrite(IB1, 0); gpioWrite(IB2, 1); // reverse right
}

void Motors::stop() {
    gpioWrite(IA1, 0); gpioWrite(IA2, 0);
    gpioWrite(IB1, 0); gpioWrite(IB2, 0);
}
