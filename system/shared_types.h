struct AudioFrame {
};

struct PitchResult {
    float frequencyHz;
    unsigned long timestampMs;
};

struct MotorCommand {
    bool enabled;
    int direction = 0;
    int pwm = 0;
};