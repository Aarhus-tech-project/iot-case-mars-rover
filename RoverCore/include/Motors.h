#ifndef MOTORS_H
#define MOTORS_H

class Motors {
public:
    Motors();
    ~Motors();

    void forward();
    void reverse();
    void left();
    void right();
    void stop();

private:
    int IA1, IA2; // Motor A GPIO pins
    int IB1, IB2; // Motor B GPIO pins
};

#endif