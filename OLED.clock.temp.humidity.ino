#include <Wire.h>
#include <I2C_RTC.h>
#include <Adafruit_AHTX0.h>
#include <KY040.h>
#include <Adafruit_SH110X.h>

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

// OLED 128x64 (I2C). We'll use U8g2; many GME12864 modules use SH1106 controller.
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// U8g2 hardware I2C instances for SH1106 and SSD1306 as fallback
// Use page-buffer variants (smaller RAM footprint) for AVR/UNO
U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2_sh1106(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2_ssd1306(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
U8G2 *u8g2 = &u8g2_sh1106; // pointer to active instance

// detected I2C address (filled by scanner)
int detectedI2CAddr = -1;

int scanI2C() {
  byte error, address;
  int firstFound = -1;
  Serial.println("Scanning I2C bus...");
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at 0x");
      if (address < 16) Serial.print('0');
      Serial.print(address, HEX);
      Serial.println(" !");
      if (firstFound == -1) firstFound = address;
    } else if (error == 4) {
      Serial.print("Unknown error at 0x");
      if (address < 16) Serial.print('0');
      Serial.println(address, HEX);
    }
  }
  if (firstFound == -1) Serial.println("No I2C devices found");
  return firstFound;
}


// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  // I2C
  Wire.begin();

  // scan I2C bus and report addresses
  detectedI2CAddr = scanI2C();
  if (detectedI2CAddr != -1) {
    Serial.print("Using I2C address 0x"); if (detectedI2CAddr < 16) Serial.print('0'); Serial.println(detectedI2CAddr, HEX);
  } else {
    Serial.println("No devices found; will try default 0x3C for SSD1306");
  }

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
  
  // Initialize u8g2 (try SH1106 controller first since GME12864-13 often uses it)

    // Initialize u8g2 with SH1106 driver (GME12864-13 is SH1106-compatible)
    u8g2 = &u8g2_sh1106;
    u8g2->begin();
    u8g2->clearBuffer();
    u8g2->setFont(u8g2_font_ncenB14_tr);
    const char *msg = "SH1106 init";
    int x = (SCREEN_WIDTH - u8g2->getStrWidth(msg)) / 2; if (x < 0) x = 0;
    u8g2->drawStr(x, 30, msg);
    u8g2->sendBuffer();

    Serial.println("Using SH1106 driver. If the OLED stays blank, check power (3.3V/5V), SDA/SCL wiring,");
    Serial.println("and try the I2C Scanner output above to confirm the display address.");
  Serial.println("Startup complete. If the OLED stays blank, check power (3.3V/5V), SDA/SCL wiring,");
  Serial.println("and try the I2C Scanner output above to confirm the display address.");

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
      u8g2->clearBuffer();
      u8g2->setFont(u8g2_font_ncenB14_tr);
      const char *s = "SET TIME";
      int _x = (SCREEN_WIDTH - u8g2->getStrWidth(s)) / 2; if (_x < 0) _x = 0;
      u8g2->drawStr(_x, 20, s);
      u8g2->sendBuffer();
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
          u8g2->clearBuffer();
          u8g2->sendBuffer();
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

    // render menu on OLED via u8g2
    u8g2->clearBuffer();
    const char *title = (setStage==0)?"Set Hour":"Set Min";
    u8g2->setFont(u8g2_font_ncenB08_tr);
    int tx = (SCREEN_WIDTH - u8g2->getStrWidth(title)) / 2; if (tx < 0) tx = 0; u8g2->drawStr(tx, 12, title);
    // time value
    char timebuf[10];
    snprintf(timebuf, sizeof(timebuf), "%02d:%02d", setHour, setMin);
    u8g2->setFont(u8g2_font_ncenB14_tr);
    int tx2 = (SCREEN_WIDTH - u8g2->getStrWidth(timebuf)) / 2; if (tx2 < 0) tx2 = 0; u8g2->drawStr(tx2, 38, timebuf);
    // instruction
    u8g2->setFont(u8g2_font_ncenB08_tr);
    const char *instr = (setStage==0) ? "Rotate to set hour" : "Rotate to set minute";
    int tx3 = (SCREEN_WIDTH - u8g2->getStrWidth(instr)) / 2; if (tx3 < 0) tx3 = 0; u8g2->drawStr(tx3, 60, instr);
    u8g2->sendBuffer();

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

    // ---- OLED update via u8g2 ----
    // Time string HH:MM:SS centered
    char timestr[9];
    snprintf(timestr, sizeof(timestr), "%02u:%02u:%02u", hour, minute, second);
    u8g2->clearBuffer();
    u8g2->setFont(u8g2_font_ncenB14_tr);
    int tx = (SCREEN_WIDTH - u8g2->getStrWidth(timestr)) / 2; if (tx < 0) tx = 0;
    u8g2->drawStr(tx, 20, timestr);

    // Temperature and humidity line
    float t = temp.temperature;
    float h = humidity.relative_humidity;
    char deg = (char)223;
    float f = t * 9.0 / 5.0 + 32.0;
    char fbuf[8];
    char hbuf[8];
    dtostrf(f, 4, 1, fbuf);
    dtostrf(h, 3, 0, hbuf);

    char tstr[16];
    char hstr[8];
    snprintf(tstr, sizeof(tstr), "%s%cF", fbuf, deg);
    snprintf(hstr, sizeof(hstr), "%s%%", hbuf);

    char linebuf[32];
    snprintf(linebuf, sizeof(linebuf), "%s  %s", tstr, hstr);
    u8g2->setFont(u8g2_font_ncenB08_tr);
    int lx = (SCREEN_WIDTH - u8g2->getStrWidth(linebuf)) / 2; if (lx < 0) lx = 0;
    u8g2->drawStr(lx, 56, linebuf);

    u8g2->sendBuffer();
  }
}
