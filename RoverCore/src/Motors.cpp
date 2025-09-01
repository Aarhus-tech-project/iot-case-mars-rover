#include "Motors.h"
#include <iostream>
#include <thread>
#include <chrono>

#if defined(_WIN32) || defined(__APPLE__) || defined(__x86_64__)
inline int gpioInitialise() { return 0; }
inline void gpioTerminate() {}
inline void gpioSetMode(int, int) {}
inline void gpioWrite(int, int) {}
inline void gpioPWM(int, int) {}
#define PI_OUTPUT 1
#else
#include <pigpio.h>
#endif

Motors::Motors() {
    if (gpioInitialise() < 0) {
        std::cerr << "[motors] pigpio initialization failed!" << std::endl;
        return;
    }

    IA1 = 4;
    IA2 = 17;
    IB1 = 23;
    IB2 = 24;

    gpioSetMode(IA1, PI_OUTPUT);
    gpioSetMode(IA2, PI_OUTPUT);
    gpioSetMode(IB1, PI_OUTPUT);
    gpioSetMode(IB2, PI_OUTPUT);

    stop();
}

Motors::~Motors() {
    stop();
#if !(defined(_WIN32) || defined(__APPLE__) || defined(__x86_64__))
    gpioTerminate();
#endif
}

// Forward
void Motors::forward() {
    gpioWrite(IA1, 1);
    gpioWrite(IA2, 0);
    gpioWrite(IB1, 1);
    gpioWrite(IB2, 0);
}

// Reverse
void Motors::reverse() {
    gpioWrite(IA1, 0);
    gpioWrite(IA2, 1);
    gpioWrite(IB1, 0);
    gpioWrite(IB2, 1);
}

// Left turn
void Motors::left() {
    gpioWrite(IA1, 0);
    gpioWrite(IA2, 1); // reverse left
    gpioWrite(IB1, 1); // forward right
    gpioWrite(IB2, 0);
}

// Right turn
void Motors::right() {
    gpioWrite(IA1, 1); // forward left
    gpioWrite(IA2, 0);
    gpioWrite(IB1, 0);
    gpioWrite(IB2, 1); // reverse right
}

// Stop
void Motors::stop() {
    gpioWrite(IA1, 0);
    gpioWrite(IA2, 0);
    gpioWrite(IB1, 0);
    gpioWrite(IB2, 0);
}
