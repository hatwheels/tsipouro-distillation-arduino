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
#define SERIAL_DEBUG            (0)

const int buttonPin = 2;
const int buzzerPin = 9;
const int relayPin = 10;
const int ledOnPin = 7;
const int ledOffPin = 8;

double temperature;
int buttonState = LOW;
bool nextStep = false;
unsigned int stepCounter = 0;

double Thermistor(int rawADC) {
    double temperature = log(10000.0 * ((1024.0 / rawADC - 1)));

    temperature = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * temperature * temperature)) * temperature);
    temperature -= 273.15;
    temperature = (temperature * 9.0) / 5.0 + 32.0;
    temperature = (temperature - 32) * 5 / 9;
    
    return (round(temperature * 10.0) / 10.0);
}

LiquidCrystal_I2C lcd(0x27, LCD_CHARS, LINES);

Timer debounceTimer;
Timer temperatureTimer;
Timer processTimer;
Timer stepTimer;

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

/*
    Clear one line of I2C LCD.
    args: line - number of line to be cleared.
*/
void clearLcdLine (uint8_t line) {
    lcd.setCursor(0, line);
    for (uint8_t i = 0; i < LCD_CHARS; i++) {
        lcd.write((char) 254);
    }
}

/* timer callback to debounce button. */
void debounceButton() {
    int reading = digitalRead(buttonPin);
#if SERIAL_DEBUG
    Serial.println("debouncing callback");
#endif
    if (reading != buttonState && (buttonState = reading) == HIGH) {
        nextStep = true;
    }
}

/* timer callback to update temperature on LCD. */
void updateTemperature() {
    double currentTemperature = Thermistor(analogRead(0));
#if SERIAL_DEBUG
    Serial.println("Temperature updated");
#endif
    if (temperature != currentTemperature) {
        temperature = currentTemperature;
        if (temperature >= 30.0) {
            digitalWrite(relayPin, HIGH);
            digitalWrite(ledOnPin, HIGH);
            digitalWrite(ledOffPin, LOW);
        } else if (temperature < 20.0) {
            digitalWrite(relayPin, LOW);
            digitalWrite(ledOnPin, LOW);
            digitalWrite(ledOffPin, HIGH);
        }

        lcd.setCursor(7, 0);
        lcd.print(String(temperature, 1));
        lcd.write((char) 223);
        lcd.write('C');
    }
}

/* timer callback to update process timespan on LCD. */
void updateProcessTime() {
    unsigned long span = processTimer.getElapsedTime() / 1000;
    char spanStr[9] = { '\0' };

    sprintf(spanStr, "%02d:%02d:%02d", numberOfHours(span), numberOfMinutes(span), numberOfSeconds(span));
    lcd.setCursor(7, 1);
    lcd.print(spanStr);
#if SERIAL_DEBUG
    Serial.print("process time: ");
    Serial.println(spanStr);
#endif
}

/* Manage Start/Stop of the process and its intermediate steps. */
void manageProcess() {
    if (nextStep) {
        nextStep = false;
        tone(buzzerPin, BUZZER_FREQUENCY, BUZZER_PERIOD);

        switch (stepCounter = (stepCounter + 1) % 8) {
            case 0:
                clearLcdLine(1);
                clearLcdLine(2);
                clearLcdLine(3);

                processTimer.stop();
                stepTimer.stop();
                break;
            case 1:
                lcd.setCursor(7, 1);
                lcd.print("00:00:00");

                processTimer.start();
                stepTimer.start();
                break;
            default: {
                unsigned long delta = stepTimer.getElapsedTime() / 1000;
                char deltaStr[7] = { '\0' };

                sprintf(deltaStr, "%03d:%02d", numberOfMinutesForDelta(delta), numberOfSeconds(delta));
                lcd.setCursor(((stepCounter - 2) % 3) * 7, stepCounter > 4 ? 3 : 2);
                lcd.print(deltaStr);

                stepTimer.reset();
            }
        }
    }
}

void setup () {
#if SERIAL_DEBUG
    Serial.begin(9600);
#endif

    pinMode(buttonPin, INPUT);
    pinMode(relayPin, OUTPUT);
    pinMode(buzzerPin, OUTPUT);
    pinMode(ledOnPin, OUTPUT);
    pinMode(ledOffPin, OUTPUT);

    lcd.init();
    lcd.backlight();

    temperature = Thermistor(analogRead(0));

    temperatureTimer.setInterval(TEMPERATURE_INTERVAL);
    temperatureTimer.setCallback(updateTemperature);
    temperatureTimer.start();
    
    processTimer.setInterval(PROCESS_INTERVAL);
    processTimer.setCallback(updateProcessTime);
    
    debounceTimer.setInterval(DEBOUNCE_DELAY);
    debounceTimer.setCallback(debounceButton);
    debounceTimer.start();
}

void loop () {
    temperatureTimer.update();
    debounceTimer.update();
    processTimer.update();
    stepTimer.update();
    manageProcess();
}