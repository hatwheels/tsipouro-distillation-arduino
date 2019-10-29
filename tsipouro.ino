#include <math.h>
// LiquidCrystal I2C - https://github.com/johnrickman/LiquidCrystal_I2C
#include <LiquidCrystal_I2C.h>

// Useful Constants
#define SECS_PER_MIN            (60UL)
#define SECS_PER_HOUR           (3600UL)
#define SECS_PER_DAY            (SECS_PER_HOUR * 24L)
#define DELTA_LCD_MIN           (1000UL)
#define LCD_CHARS               (20)
#define LINES                   (4)
#define TEMPERATURE_INTERVAL    (1000UL)
#define PROCESS_INTERVAL        (1000UL)
#define STEP_INTERVAL           (1000UL)
#define DEBOUNCE_DELAY          (100UL)
#define BUZZER_FREQUENCY        (700)
#define BUZZER_PERIOD           (500)
// Debug Macro
#define SERIAL_DEBUG            (0)

const int buttonPin = 2;
const int buzzerPin = 9;
const int relayPin = 10;
const int ledOnPin = 7;
const int ledOffPin = 8;

unsigned long debounceTime, temperatureTime, processTime, stepTime;
unsigned long processStartTime, stepStartTime;
bool isProcessOn, isStepOn;

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

/* Get number of Seconds out of Millis. */
int numberOfSeconds(unsigned long t) {
  return (t % SECS_PER_MIN);
}

/* Get number of Minutes out of Millis. */
int numberOfMinutes(unsigned long t) {
  return ((t / SECS_PER_MIN) % SECS_PER_MIN);
}

/* Get number of Hours out of Millis. */
int numberOfHours(unsigned long t) {
  return (( t % SECS_PER_DAY) / SECS_PER_HOUR);
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
  Serial.println("debounce callback");
#endif SERIAL_DEBUG
    if (reading != buttonState && (buttonState = reading) == HIGH) {
        nextStep = true;
    }
}

/* timer callback to update temperature on LCD. */
void updateTemperature() {
    double currentTemperature = Thermistor(analogRead(0));

    if (temperature != currentTemperature) {
        temperature = currentTemperature;
        if (temperature >= 30.0) {
            digitalWrite(relayPin, HIGH);
            digitalWrite(ledOnPin, HIGH);
            digitalWrite(ledOffPin, LOW);
            lcd.setCursor(0, 0);
            lcd.print("ON ");
        } else if (temperature < 20.0) {
            digitalWrite(relayPin, LOW);
            digitalWrite(ledOnPin, LOW);
            digitalWrite(ledOffPin, HIGH);
            lcd.setCursor(0, 0);
            lcd.print("OFF");
        }

        lcd.setCursor(7, 0);
        lcd.print(String(temperature, 1));
        lcd.write((char) 223);
        lcd.write('C');
    }
}

/* timer callback to update process timespan on LCD. */
void updateProcessTime() {
    unsigned long span = (processTime - processStartTime) / 1000;
    int hours =  numberOfHours(span);
    int minutes = numberOfMinutes(span);
    int seconds = numberOfSeconds(span);

    lcd.setCursor(7, 1);
    if (hours < 10) {
        lcd.write('0');
    }
    lcd.print(hours);
    lcd.write(':');
    if (minutes < 10) {
        lcd.write('0');
    }
    lcd.print(minutes);
    lcd.write(':');
    if (seconds < 10) {
        lcd.write('0');
    }
    lcd.print(seconds);
}

void updateStepTime(unsigned int id) {
    unsigned long delta = (stepTime - stepStartTime) / 1000;
    int minutes = numberOfMinutes(delta);
    int seconds = numberOfSeconds(delta);

    lcd.setCursor(((id - 2) % 3) * 7, id > 4 ? 3 : 2);
    if (minutes < 10) {
        lcd.write('0');
    }
    lcd.print(minutes);
    lcd.write(':');
    if (seconds < 10) {
        lcd.write('0');
    }
    lcd.print(seconds);
}

/* Manage Start/Stop of the process and its intermediate steps. */
void manageProcess(unsigned long t) {
    if (nextStep) {
        nextStep = false;
        tone(buzzerPin, BUZZER_FREQUENCY, BUZZER_PERIOD);

        switch (stepCounter = (stepCounter + 1) % 8) {
            case 0:
                clearLcdLine(1);
                clearLcdLine(2);
                clearLcdLine(3);

                isProcessOn = false;
                stepStartTime = 0;
                processStartTime = 0;
                break;
            case 1:
                lcd.setCursor(7, 1);
                lcd.print("00:00:00");

                isProcessOn = true;
                isStepOn = true;
                processStartTime = stepStartTime =  t;
                break;
            case 7:
                isStepOn = false;
            default:
                updateStepTime(stepCounter);
                stepStartTime = stepTime;
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

    debounceTime = temperatureTime = processTime = stepTime = 0;
    processStartTime = stepStartTime = 0;
    isProcessOn = isStepOn = false;

    lcd.init();
    lcd.backlight();

    temperature = -273.15;

    lcd.setCursor(0, 0);
    lcd.print("OFF");

    digitalWrite(ledOnPin, LOW);
    digitalWrite(ledOffPin, HIGH);
}

void loop () {
    unsigned long currentTime = millis();

    if (currentTime > debounceTime + DEBOUNCE_DELAY) {
        debounceTime = currentTime;
        debounceButton();
    }
    if (currentTime > temperatureTime + TEMPERATURE_INTERVAL) {
        temperatureTime = currentTime;
        updateTemperature();
    }
    if (isStepOn && (currentTime > stepTime + STEP_INTERVAL)) {
        stepTime = currentTime;
        updateStepTime(stepCounter + 1);
    }
    if (isProcessOn && (currentTime > processTime + PROCESS_INTERVAL)) {
        processTime = currentTime;
        updateProcessTime();
    }
    manageProcess(currentTime);
}