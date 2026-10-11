#include <Arduino.h>
#include <driver/gpio.h>
#include <stdlib.h>
#include <string.h>
#include "robot_config.h"
#include "motion_math.h"

namespace {
struct Encoder {
    int a, b, sign;
    volatile uint32_t ticks = 0;
    volatile unsigned state = 0;
    Encoder(int pinA, int pinB, int polarity) : a(pinA), b(pinB), sign(polarity) {}
};
Encoder encoders[2] = {{config::ENC_A[0], config::ENC_B[0], config::ENCODER_SIGN[0]},
                       {config::ENC_A[1], config::ENC_B[1], config::ENCODER_SIGN[1]}};
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
// Positive sequence: 00 -> 01 -> 11 -> 10 -> 00. RAM for IRAM handler.
DRAM_ATTR const int8_t transitions[16] = {0, 1, -1, 0, -1, 0, 0, 1,
                                        1, 0, 0, -1, 0, -1, 1, 0};
void IRAM_ATTR encoderInterrupt(void *arg) {
    auto *e = static_cast<Encoder *>(arg);
    portENTER_CRITICAL_ISR(&encoderMux);
    const unsigned state = (gpio_get_level(static_cast<gpio_num_t>(e->a)) << 1) |
                           gpio_get_level(static_cast<gpio_num_t>(e->b));
    e->ticks += transitions[(e->state << 2) | state] * e->sign;
    e->state = state;
    portEXIT_CRITICAL_ISR(&encoderMux);
}
void readEncoders(uint32_t (&counts)[2]) {
    portENTER_CRITICAL(&encoderMux);
    for (int i = 0; i < 2; ++i) counts[i] = encoders[i].ticks;
    portEXIT_CRITICAL(&encoderMux);
}
enum class Mode { Stopped, Pwm };
Mode mode = Mode::Stopped;
float target[2] = {}, measured[2] = {};
int output[2] = {};
uint32_t previousCounts[2] = {}, lastCommand = 0, lastControl = 0;
int previousDirection[2] = {};
const char *fault = nullptr;
bool telemetry = false;
void drive(int wheel, int value) {
    value = constrain(value, -config::MAX_PWM, config::MAX_PWM);
    output[wheel] = value;
    value *= config::MOTOR_SIGN[wheel];
    ledcWrite(wheel, 0);
    digitalWrite(config::IN1[wheel], value > 0 ? HIGH : LOW);
    digitalWrite(config::IN2[wheel], value < 0 ? HIGH : LOW);
    ledcWrite(wheel, abs(value));
}
void stop() {
    digitalWrite(config::STBY, LOW);
    mode = Mode::Stopped;
    for (int i = 0; i < 2; ++i) {
        drive(i, 0); target[i] = 0; previousDirection[i] = 0;
    }
}
void trip(const char *reason) { stop(); fault = reason; Serial.printf("FAULT %s\n", reason); }
void setTarget(float left, float right) {
    if (fault) { Serial.println("ERR fault latched; inspect then clear"); return; }
    motion::limitPair(left, right, config::MAX_PWM);
    if (left == 0 && right == 0) { stop(); return; }
    const uint32_t now = millis();
    target[0] = left; target[1] = right;
    mode = Mode::Pwm; lastCommand = now;
}
void version() {
    Serial.printf("firmware=%s motor_sign=%d,%d\n", config::FIRMWARE_VERSION,
                  config::MOTOR_SIGN[0], config::MOTOR_SIGN[1]);
}
void status() {
    version();
    uint32_t counts[2]; readEncoders(counts);
    Serial.printf("mode=%s fault=%s ticks=%ld,%ld tps=%.1f,%.1f target=%.1f,%.1f pwm=%d,%d",
                  mode == Mode::Stopped ? "stop" : "pwm",
                  fault ? fault : "none", static_cast<long>(static_cast<int32_t>(counts[0])),
                  static_cast<long>(static_cast<int32_t>(counts[1])), measured[0], measured[1],
                  target[0], target[1], output[0], output[1]);
    Serial.printf(" rpm=%.2f,%.2f", measured[0] * 60 / config::COUNTS_PER_REV,
                                 measured[1] * 60 / config::COUNTS_PER_REV);
    Serial.println();
}
void help() {
    version();
    Serial.println("Commands (newline terminated, repeat motion commands every 100 ms):\n"
                   "w/s/a/d: forward/back/left/right at PWM 80; x or stop: stop\n"
                   "pwm LEFT RIGHT: signed duty [-180,180], open loop\n"
                   "version | status | stream 0/1 | clear | help\n"
                   "500 ms command watchdog; boot is stopped; clear does not restart.");
}
bool parseNumber(const char *token, float &result) {
    if (!token) return false;
    char *end = nullptr;
    result = strtof(token, &end);
    return end != token && *end == '\0' && isfinite(result) && fabsf(result) <= 1e6f;
}
void command(char *line) {
    char *args[5] = {}; unsigned count = 0; char *context = nullptr;
    for (char *s = strtok_r(line, " \t", &context); s && count < 5;
         s = strtok_r(nullptr, " \t", &context)) args[count++] = s;
    if (!count) return;
    const char *name = args[0];
    if (count == 1) {
        if (!strcmp(name, "stop") || !strcmp(name, "x")) { stop(); return; }
        if (!strcmp(name, "status")) { status(); return; }
        if (!strcmp(name, "version")) { version(); return; }
        if (!strcmp(name, "help")) { help(); return; }
        if (!strcmp(name, "clear")) { stop(); fault = nullptr; Serial.println("OK cleared"); return; }

        const float jog = config::JOG_PWM;
        if (!strcmp(name, "w")) { setTarget(jog, jog); return; }
        if (!strcmp(name, "s")) { setTarget(-jog, -jog); return; }
        if (!strcmp(name, "a")) { setTarget(-jog, jog); return; }
        if (!strcmp(name, "d")) { setTarget(jog, -jog); return; }
    }
    if (count == 2 && !strcmp(name, "stream") && (!strcmp(args[1], "0") || !strcmp(args[1], "1"))) {
        telemetry = args[1][0] == '1'; Serial.println("OK"); return;
    }
    float left, right;
    if (count == 3 && parseNumber(args[1], left) && parseNumber(args[2], right)) {
        if (!strcmp(name, "pwm")) { setTarget(left, right); return; }
    }
    stop(); Serial.println("ERR invalid command; stopped");
}
void readSerial() {
    static char buffer[96]; static unsigned used = 0; static bool discard = false;
    // Bound work so flooding the serial port cannot starve control.
    for (unsigned budget = 0; budget < 64 && Serial.available(); ++budget) {
        const char c = Serial.read();
        if (c == '\r' || c == '\n') {
            if (!discard && used) { buffer[used] = '\0'; command(buffer); }
            used = 0; discard = false;
        } else if (!discard) {
            if (c < 32 || c > 126 || used == sizeof(buffer) - 1) {
                discard = true; stop(); Serial.println("ERR invalid/long line; stopped");
            } else buffer[used++] = c;
        }
    }
}
void control(uint32_t now) {
    const uint32_t elapsed = now - lastControl;
    if (elapsed < config::CONTROL_MS) return;
    lastControl = now;
    const float dt = elapsed / 1000.0f;
    uint32_t counts[2]; readEncoders(counts);
    int32_t delta[2];
    for (int i = 0; i < 2; ++i) {
        delta[i] = static_cast<int32_t>(counts[i] - previousCounts[i]);
        previousCounts[i] = counts[i]; measured[i] = delta[i] / dt;
    }

    if (mode == Mode::Stopped) return;
    if (now - lastCommand >= config::COMMAND_TIMEOUT_MS) { stop(); Serial.println("STOP timeout"); return; }
    if (elapsed > 100) { trip("control_overrun"); return; }
    for (int i = 0; i < 2; ++i) {
        const int value = lroundf(target[i]);
        const int direction = (value > 0) - (value < 0);
        // One zero-output control period when reversing direction.
        if (direction && previousDirection[i] && direction != previousDirection[i]) {
            drive(i, 0); previousDirection[i] = 0;
        } else { drive(i, value); previousDirection[i] = direction; }
    }
    digitalWrite(config::STBY, HIGH);
}
}
void setup() {
    pinMode(config::STBY, OUTPUT); digitalWrite(config::STBY, LOW);
    Serial.begin(115200);
    for (int i = 0; i < 2; ++i) {
        pinMode(config::IN1[i], OUTPUT); pinMode(config::IN2[i], OUTPUT);
        ledcSetup(i, config::PWM_HZ, 8); ledcAttachPin(config::PWM[i], i);
        drive(i, 0);
        auto &e = encoders[i];
        pinMode(e.a, INPUT_PULLUP); pinMode(e.b, INPUT_PULLUP);
        e.state = (digitalRead(e.a) << 1) | digitalRead(e.b);
        attachInterruptArg(e.a, encoderInterrupt, &e, CHANGE);
        attachInterruptArg(e.b, encoderInterrupt, &e, CHANGE);
    }
    readEncoders(previousCounts); lastControl = millis(); help();
}
void loop() {
    control(millis()); readSerial();
    static uint32_t lastReport = 0;
    if (telemetry && millis() - lastReport >= 200) { lastReport = millis(); status(); }
    delay(1);
}
