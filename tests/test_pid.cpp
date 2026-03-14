#include "../control/pid_controller.h"
#include <cassert>
#include <cmath>
#include <cstdio>

// Helper to create a mock PitchResult
PitchResult makePitch(float freqHz, unsigned long timestampMs) {
    PitchResult p{};
    p.frequencyHz = freqHz;
    p.centsDeviation = 0.0f;
    p.timestampMs = timestampMs;
    p.nearestNote[0] = '\0';
    return p;
}

int main() {
    PIDController pid;
    initPIDController(pid, 10.0f, 4.0f, 8.0f, -255.0f, 255.0f);

    // Test 1: First run - pitch below target (string too loose)
    // Target 82 Hz (E2), actual 80 Hz → positive error → positive output
    float out1 = computePID(pid, 82.0f, makePitch(80.0f, 100));
    assert(out1 > 0 && "Should output positive (tighten) when below target");
    printf("Test 1 pass: freq 80 vs target 82 -> output %.1f (positive)\n", out1);

    // Test 2: Pitch above target (string too tight)
    resetPIDController(pid);
    float out2 = computePID(pid, 82.0f, makePitch(85.0f, 200));
    assert(out2 < 0 && "Should output negative (loosen) when above target");
    printf("Test 2 pass: freq 85 vs target 82 -> output %.1f (negative)\n", out2);

    // Test 3: At target → near zero output
    resetPIDController(pid);
    float out3 = computePID(pid, 82.0f, makePitch(82.0f, 300));
    assert(std::fabs(out3) < 1.0f && "Should be ~0 when at target");
    printf("Test 3 pass: freq 82 vs target 82 -> output %.3f (~0)\n", out3);

    // Test 4: buildMotorCommand - deadband
    MotorCommand cmd_small = buildMotorCommand(5.0f);  // Below deadband of 8
    assert(!cmd_small.enabled && "Below deadband should not enable motor");
    printf("Test 4 pass: output 5 (below deadband) -> disabled\n");

    // Test 5: buildMotorCommand - above deadband
    MotorCommand cmd_large = buildMotorCommand(50.0f);
    assert(cmd_large.enabled && cmd_large.pwm == 50 && cmd_large.direction == 1);
    printf("Test 5 pass: output 50 -> enabled, pwm=50, dir=+1\n");

    // Test 6: buildMotorCommand - negative output
    MotorCommand cmd_neg = buildMotorCommand(-100.0f);
    assert(cmd_neg.enabled && cmd_neg.pwm == 100 && cmd_neg.direction == -1);
    printf("Test 6 pass: output -100 -> enabled, pwm=100, dir=-1\n");

    printf("\nAll PID controller tests passed.\n");
    return 0;
}
