#include <Wire.h>
#include <I2C_RTC.h>
#include <Adafruit_AHTX0.h>
#include <LiquidCrystal_I2C.h>
#include <KY040.h>

// ================= ROTARY ENCODER (KY-040) =================
// Pins: CLK on A0, DT on A1, SW (push) on A2
const uint8_t ENC_CLK = A0;
const uint8_t ENC_DT  = A1;
const uint8_t ENC_SW  = A2;

KY040 enc(ENC_CLK, ENC_DT);

// menu / button state
bool settingMode = false;       // true when in time-set menu
uint8_t setStage = 0;           // 0=hours,1=minutes
int setHour = 0;
int setMin = 0;
unsigned long buttonDownAt = 0;
bool buttonWasDown = false;
unsigned long lastDisplay = 0;
const unsigned long DISPLAY_INTERVAL = 1000;
unsigned long lastEncTime = 0;
const unsigned long ENC_DEBOUNCE_MS = 12;
// last rotation value polled from KY040
int lastRot = KY040::IDLE;


// ================= DEVICES =================
static DS3231 rtc;
Adafruit_AHTX0 aht;
LiquidCrystal_I2C lcd(0x27, 16, 2);


// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  // I2C
  Wire.begin();

  // encoder pins
  pinMode(ENC_SW, INPUT_PULLUP);
  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);

  // AHT sensor
  if (!aht.begin()) {
    Serial.println("AHT sensor not found on hardware I2C (Wire)");
  } else {
    Serial.println("AHT initialized on hardware I2C");
  }

  rtc.begin();

  lcd.init();
  lcd.backlight();
  lcd.clear();

}

// helper: convert decimal to BCD for DS3231
uint8_t decToBcd(uint8_t val) {
  return ((val / 10) << 4) | (val % 10);
}

// write time directly to DS3231 registers (address 0x68)
void writeRTCTime(uint8_t hour, uint8_t minute, uint8_t second) {
  Wire.beginTransmission(0x68);
  Wire.write((uint8_t)0x00); // start at seconds register
  Wire.write(decToBcd(second));
  Wire.write(decToBcd(minute));
  Wire.write(decToBcd(hour));
  Wire.endTransmission();
  Serial.print("RTC time set to ");
  if (hour < 10) Serial.print('0');
  Serial.print(hour);
  Serial.print(":");
  if (minute < 10) Serial.print('0');
  Serial.println(minute);
}

// ================= LOOP =================
void loop() {
  // Poll encoder every loop to avoid missing steps
  static int prevClk = HIGH, prevDt = HIGH;
  int clkVal = digitalRead(ENC_CLK);
  int dtVal = digitalRead(ENC_DT);
  if (clkVal != prevClk || dtVal != prevDt) {
    Serial.print("RAW CLK="); Serial.print(clkVal);
    Serial.print(" DT="); Serial.println(dtVal);
    prevClk = clkVal; prevDt = dtVal;
  }

  int rot = enc.getRotation();
  if (rot != KY040::IDLE && rot != KY040::ACTIVE) {
    lastRot = rot;
    Serial.print("enc.getRotation(): "); Serial.println(rot);
  } else {
    lastRot = rot;
  }

  int last = enc.getAndResetLastRotation();
  if (last != KY040::IDLE) {
    Serial.print("enc.getAndResetLastRotation(): "); Serial.println(last);
  }

  // ---------- encoder button handling (long press to enter setting mode) ----------
  bool btn = digitalRead(ENC_SW) == LOW; // active low
  if (btn && !buttonWasDown) {
    buttonDownAt = millis();
    buttonWasDown = true;
  } else if (!btn && buttonWasDown) {
    unsigned long held = millis() - buttonDownAt;
    buttonWasDown = false;
    if (!settingMode && held >= 5000) {
      // long press -> enter setting mode
      settingMode = true;
      setStage = 0;
      setHour = rtc.getHours();
      setMin = rtc.getMinutes();
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("SET TIME");
      delay(400);
    } else if (settingMode) {
      // short press while in menu -> advance stage / confirm
      if (held < 3000) {
        if (setStage == 0) {
          setStage = 1; // move to minutes
        } else {
          // finished — write new time to RTC (seconds = 0)
          writeRTCTime(setHour, setMin, 0);
          settingMode = false;
          lcd.clear();
          delay(300);
        }
      }
    }
  }

  // If in setting mode, handle encoder rotation to change values
  if (settingMode) {
    // Simple rising-edge on CLK increments (user request)
    static int prevClkEdge = HIGH;
    int clk = digitalRead(ENC_CLK);
    unsigned long now = millis();
    if (prevClkEdge == LOW && clk == HIGH && (now - lastEncTime) > ENC_DEBOUNCE_MS) {
      lastEncTime = now;
      int delta = 1; // increment on each CLK rising edge
      if (setStage == 0) {
        setHour += delta;
        while (setHour < 0) setHour += 24;
        while (setHour > 23) setHour -= 24;
      } else {
        setMin += delta;
        while (setMin < 0) setMin += 60;
        while (setMin > 59) setMin -= 60;
      }
      Serial.print("CLK-rise applied +1 -> "); Serial.print(setHour); Serial.print(":"); Serial.println(setMin);
    }
    prevClkEdge = clk;

    // render menu
    char buf[17];
    lcd.setCursor(0,0);
    snprintf(buf, sizeof(buf), "Set %s:          ", (setStage==0)?"Hour":"Min ");
    lcd.print(buf);
    lcd.setCursor(0,1);
    if (setStage == 0) {
      if (setHour < 10) lcd.print('0');
      lcd.print(setHour);
      lcd.print(":");
      if (setMin < 10) lcd.print('0');
      lcd.print(setMin);
      lcd.print("  (press to next)");
    } else {
      if (setHour < 10) lcd.print('0');
      lcd.print(setHour);
      lcd.print(":");
      if (setMin < 10) lcd.print('0');
      lcd.print(setMin);
      lcd.print("  (press to save)");
    }

    delay(40);
    return; // skip normal display while setting
  }

  // update display only at intervals to keep loop responsive for encoder
  unsigned long now = millis();
  if (now - lastDisplay >= DISPLAY_INTERVAL) {
    lastDisplay = now;

    // ---- RTC ----
    uint8_t hour   = rtc.getHours();
    uint8_t minute = rtc.getMinutes();
    uint8_t second = rtc.getSeconds();

    // ---- AHT10 ----
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);

    // ---- LCD update (write fixed-width fields to avoid flicker) ----
    // Line 0: HH:MM:SS (centered)
    char timestr[9];
    snprintf(timestr, sizeof(timestr), "%02u:%02u:%02u", hour, minute, second);
    lcd.setCursor(4, 0); // center on 16-char display
    lcd.print(timestr);

    // Line 1: temperature and humidity
    float t = temp.temperature;
    float h = humidity.relative_humidity;
    char deg = (char)223;
    float f = t * 9.0 / 5.0 + 32.0;
    char fbuf[8];
    char hbuf[8];
    dtostrf(f, 4, 1, fbuf);
    dtostrf(h, 3, 0, hbuf);

    char tstr[12];
    char hstr[8];
    snprintf(tstr, sizeof(tstr), "%s%cF", fbuf, deg);
    snprintf(hstr, sizeof(hstr), "%s%%", hbuf);

    char linebuf[17];
    snprintf(linebuf, sizeof(linebuf), "%s  %s", tstr, hstr); // two spaces
    int len = strlen(linebuf);
    int start = (16 - len) / 2;
    if (start < 0) start = 0;
    lcd.setCursor(0, 1);
    for (int i = 0; i < 16; i++) lcd.print(' ');
    lcd.setCursor(start, 1);
    lcd.print(linebuf);
  }
}
