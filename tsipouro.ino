#include <math.h>
// LiquidCrystal I2C - Version: Latest
#include <LiquidCrystal_I2C.h>
// Timer - https://github.com/brunocalou/Timer
#include <timer.h>

// Useful Constants
#define SECS_PER_MIN            (60UL)
#define SECS_PER_HOUR           (3600UL)
#define SECS_PER_DAY            (SECS_PER_HOUR * 24L)
#define DELTA_LCD_MIN           (999UL)
#define LCD_CHARS               (20)
#define LINES                   (4)
#define TEMPERATURE_INTERVAL    (1000UL)
#define PROCESS_INTERVAL        (1000UL)
#define DEBOUNCE_DELAY          (50UL)
#define BUZZER_FREQUENCY        (700)
#define BUZZER_PERIOD           (500)
// Debug Macro
#define SERIAL_DEBUG            (1)
#if SERIAL_DEBUG
    #define TEST_INTERVAL       (10*1000UL)
#endif

bool nextStep;
unsigned int stepCounter = 0;

Timer processTimer;
Timer stepTimer;
#if SERIAL_DEBUG
    Timer testTimer;
#endif

/* Get number of Seconds out of Millis. */
unsigned int numberOfSeconds(unsigned long t) {
  return (t % SECS_PER_MIN);
}

/* Get number of Minutes out of Millis. */
unsigned int numberOfMinutes(unsigned long t) {
  return ((t / SECS_PER_MIN) % SECS_PER_MIN);
}

/* Get number of Hours out of Millis. */
unsigned int numberOfHours(unsigned long t) {
  return (( t % SECS_PER_DAY) / SECS_PER_HOUR);
}

/* Get number of Minutes out of Millis for Delta LCD display. */
unsigned int numberOfMinutesForDelta (unsigned long t) {
    return ((t / SECS_PER_MIN) % DELTA_LCD_MIN);
}

/* timer callback to update process timespan on LCD. */
void updateProcessTime() {
    unsigned long span = processTimer.getElapsedTime() / 1000;
    char spanStr[9] = { '\0' };

    sprintf(spanStr, "%02d:%02d:%02d", numberOfHours(span), numberOfMinutes(span), numberOfSeconds(span));
#if SERIAL_DEBUG
    Serial.print("process time: ");
    Serial.println(spanStr);
#endif
}

#if SERIAL_DEBUG
void updateTestTime() {
    nextStep = true;
}
#endif

/* Manage Start/Stop of the process and its intermediate steps. */
void manageProcess() {
    if (nextStep) {
        nextStep = false;
        switch (stepCounter = (stepCounter + 1) % 8) {
            case 0:
#if SERIAL_DEBUG
                Serial.println("Stopped");
#endif
                processTimer.stop();
                stepTimer.stop();
                break;
            case 1:
#if SERIAL_DEBUG
                Serial.println("Started");
#endif
                processTimer.start();
                stepTimer.start();
                break;
            default: {
                unsigned long delta = stepTimer.getElapsedTime() / 1000;
                char deltaStr[7] = { '\0' };

                sprintf(deltaStr, "%03d:%02d", numberOfMinutesForDelta(delta), numberOfSeconds(delta));
#if SERIAL_DEBUG
                Serial.print("step time: ");
                Serial.println(deltaStr);
#endif
                stepTimer.reset();
            }
        }
    }
}

void setup () {
#if SERIAL_DEBUG
    Serial.begin(9600);
#endif
    processTimer.setInterval(PROCESS_INTERVAL);
    processTimer.setCallback(updateProcessTime);
#if SERIAL_DEBUG
    testTimer.setInterval(TEST_INTERVAL);
    testTimer.setCallback(updateTestTime);
#endif
}

void loop () {
    processTimer.update();
    manageProcess();
}