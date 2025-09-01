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
    int IA1, IA2, IB1, IB2;    // direction pins
    int speed;                  // current speed (0-255)
};
