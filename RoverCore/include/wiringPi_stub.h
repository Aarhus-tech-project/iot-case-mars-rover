#ifndef WIRINGPI_STUB_H
#define WIRINGPI_STUB_H

#include <iostream>

#define OUTPUT 1
#define INPUT 0
#define HIGH 1
#define LOW 0

inline int wiringPiSetup() {
    std::cout << "[stub] wiringPiSetup()" << std::endl;
    return 0;
}

inline void pinMode(int pin, int mode) {
    std::cout << "[stub] pinMode(" << pin << ", " << mode << ")" << std::endl;
}

inline void digitalWrite(int pin, int value) {
    std::cout << "[stub] digitalWrite(" << pin << ", " << value << ")" << std::endl;
}

#endif
