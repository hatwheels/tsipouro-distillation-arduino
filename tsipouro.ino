#include <math.h>
// LiquidCrystal I2C - Version: Latest
#include <LiquidCrystal_I2C.h>
// arduino-timer - Version: Latest
#include <timer.h>

// Useful Constants
#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24L)

// Useful Macros for getting elapsed time
#define numberOfSeconds(_time_) (_time_ % SECS_PER_MIN)
#define numberOfMinutes(_time_) ((_time_ / SECS_PER_MIN) % SECS_PER_MIN)
#define numberOfHours(_time_) (( _time_% SECS_PER_DAY) / SECS_PER_HOUR)

const int buttonPin = 2;
const int buzzerPin = 9;
const int pinOut = 10;

const char emptyLine[21] = {
  (char) 254, (char) 254, (char) 254, (char) 254, (char) 254, (char) 254, (char) 254, (char) 254, (char) 254, (char) 254,
  (char) 254, (char) 254, (char) 254, (char) 254, (char) 254, (char) 254, (char) 254, (char) 254, (char) 254, (char) 254,
  '\0'
};

int buttonState;
int lastButtonState = LOW;

unsigned int pressCount = 0;
unsigned long firstPressTime = 0;
unsigned long currentPressTime = 0;
unsigned long pressStartTime = 0;
unsigned long pressStopTime = 0;

unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

double Thermistor(int rawADC) {
  double temp = log(10000.0 * ((1024.0 / rawADC - 1)));

  temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * temp * temp)) * temp);
  temp -= 273.15;
  temp = (temp * 9.0) / 5.0 + 32.0;
  return ((temp - 32) * 5 / 9);
}

Timer<1> timer;

LiquidCrystal_I2C lcd(0x27, 20, 4);

bool checkTemperature(void *) {
  double temp = round(Thermistor(analogRead(0)) * 10.0) / 10.0;

  if (temp >= 30.0) {
    digitalWrite(pinOut, HIGH);
  } else if (temp < 20.0) {
    digitalWrite(pinOut, LOW);
  }

  lcd.setCursor(7, 0);
  lcd.print(String(temp, 1));
  lcd.write((char) 223);
  lcd.write('C');

  return true; // continue timer
}

void setup() {
  pinMode(buttonPin, INPUT);
  pinMode(pinOut, OUTPUT);
  pinMode(buzzerPin, OUTPUT);

  lcd.init();
  lcd.backlight();

  buttonState = digitalRead(buttonPin);

  timer.every(1000, checkTemperature);

  Serial.begin(9600);
}

void loop() {
  timer.tick(); // tick the timer

  int reading = digitalRead(buttonPin);

  if (reading != lastButtonState) {
    lastButtonState = reading;
    lastDebounceTime = millis();
  }

  if (millis() - lastDebounceTime > debounceDelay && reading != buttonState) {
    buttonState = reading;

    if (buttonState == HIGH) { // pressed
      tone(buzzerPin, 700, 500);

      switch (++pressCount) {
        case 1:
          firstPressTime = currentPressTime = pressStartTime = millis();
          lcd.setCursor(7, 1);
          lcd.print("0:00:00");
          break;

        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7: {
            pressStopTime = millis();

            lcd.setCursor(7, 1);
            lcd.print("H:MM:SS");

            unsigned long delta = pressStopTime - pressStartTime;
            char deltaStr[6] = { '\0' };

            sprintf(deltaStr, "%02d:%02d", numberOfMinutes(delta), numberOfSeconds(delta));
            lcd.setCursor(((pressCount - 2) % 3) * 7, pressCount > 4 ? 3 : 2);
            lcd.print(deltaStr);

            pressStartTime = pressStopTime;
            break;
          }

        default:
          pressCount = 0;
          firstPressTime = currentPressTime = pressStartTime = pressStopTime = 0;
          lcd.setCursor(0, 1);
          lcd.print(emptyLine);
          lcd.setCursor(0, 2);
          lcd.print(emptyLine);
          lcd.setCursor(0, 3);
          lcd.print(emptyLine);
      }
    }
  }

  if (pressCount && millis() - currentPressTime > 999) {
    char pressStr[8] = { '\0' };
    unsigned long period = firstPressTime - currentPressTime;

    sprintf(pressStr, "%02d:%02d:%02d", numberOfHours(period), numberOfMinutes(period), numberOfSeconds(period));
    lcd.setCursor(7, 1);
    lcd.print(pressStr);
  }
}