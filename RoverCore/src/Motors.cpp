#include "Motors.h"
#if defined(_WIN32) || defined(__APPLE__) || defined(__x86_64__)
#include "wiringPi_stub.h"
#else
#include "wiringPi_stub.h"
//#include <wiringPi.h>
#endif
#include <iostream>

Motors::Motors() {
    // WiringPi setup
    if (wiringPiSetup() == -1) {
        std::cerr << "[motors] wiringPi setup failed!" << std::endl;
        return;
    }

    // Assign wiringPi pin numbers (mapped from BCM GPIOs)
    IA1 = 7; // GPIO4  (pin 7)
    IA2 = 0; // GPIO17 (pin 11)
    IB1 = 4; // GPIO23 (pin 16)
    IB2 = 5; // GPIO24 (pin 18)

    pinMode(IA1, OUTPUT);
    pinMode(IA2, OUTPUT);
    pinMode(IB1, OUTPUT);
    pinMode(IB2, OUTPUT);

    stop(); // start with motors off
}

Motors::~Motors() {
    stop();
}

void Motors::forward() {
    digitalWrite(IA1, HIGH); digitalWrite(IA2, LOW);
    digitalWrite(IB1, HIGH); digitalWrite(IB2, LOW);
}

void Motors::reverse() {
    digitalWrite(IA1, LOW); digitalWrite(IA2, HIGH);
    digitalWrite(IB1, LOW); digitalWrite(IB2, HIGH);
}

void Motors::left() {
    digitalWrite(IA1, LOW); digitalWrite(IA2, HIGH);  // reverse left
    digitalWrite(IB1, HIGH); digitalWrite(IB2, LOW); // run right
}

void Motors::right() {
    digitalWrite(IA1, HIGH); digitalWrite(IA2, LOW); // run left
    digitalWrite(IB1, LOW); digitalWrite(IB2, HIGH);  // reverse right
}

void Motors::stop() {
    digitalWrite(IA1, LOW); digitalWrite(IA2, LOW);
    digitalWrite(IB1, LOW); digitalWrite(IB2, LOW);
}
