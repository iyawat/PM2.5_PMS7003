#include <Arduino.h>
#include <M5Stack.h>
#include <HardwareSerial.h>
#include <SimpleDHT.h>

// for DHT22,
//      VCC: 5V or 3V
//      GND: GND
//      DATA: 2
int pinDHT22 = 2;
SimpleDHT22 dht22(pinDHT22);

//const uint8_t PMS_RX=16, PMS_TX=17;
HardwareSerial pmsSerial(2); // UART2 on GPIO16(RX),GPIO17(TX)

// Stock font and GFXFF reference handle
#define GFXFF 1
#define FF18 &FreeSans12pt7b
#define CF_OL24 &Orbitron_Light_24
#define CF_OL32 &Orbitron_Light_32
#define CF_RT24 &Roboto_Thin_24
#define CF_S24  &Satisfy_24
#define CF_Y32  &Yellowtail_32

// Define meter size as 1 for M5.Lcd.rotation(0) or 1.3333 for M5.Lcd.rotation(1)
#define M_SIZE 1.3333

#define TFT_GREY 0x5AEB

uint32_t targetTime = 0;  // for next 1 second timeout

static uint8_t conv2d(const char* p); // Forward declaration needed for IDE 1.6.x

uint8_t hh = conv2d(__TIME__), mm = conv2d(__TIME__ + 3), ss = conv2d(__TIME__ + 6); // Get H, M, S from compile time

byte omm = 99, oss = 99;
byte xcolon = 0, xsecs = 0;
unsigned int colour = 0;

float ltx = 0;    // Saved x coord of bottom of needle
uint16_t osx = M_SIZE * 120, osy = M_SIZE * 120; // Saved x & y coords
uint32_t updateTime = 0;       // time for next update

int old_analog =  -999; // Value last displayed

int value[6] = {0, 0, 0, 0, 0, 0};
int old_value[6] = { -1, -1, -1, -1, -1, -1};
int d = 0;

int PM25AQI;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(100);
  }
  // sensor baud rate is 9600
  pmsSerial.begin(9600);

  M5.begin();
  M5.Lcd.fillScreen(TFT_BLACK);

  analogMeter(); // Draw analogue meter

  updateTime = millis(); // Next update time
  targetTime = millis() + 1000;
}

struct pms7003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};

struct pms7003data data;

void loop() {

  while (1) {
    delay(1000);
    
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
    if (targetTime < millis()) {
      // Set next update for 1 second later
      targetTime = millis() + 1000;

      // Adjust the time values by adding 1 second
      ss++;              // Advance second
      if (ss == 60) {    // Check for roll-over
        ss = 0;          // Reset seconds to zero
        omm = mm;        // Save last minute time for display update
        mm++;            // Advance minute
        if (mm > 59) {   // Check for roll-over
          mm = 0;
          hh++;          // Advance hour
          if (hh > 23) { // Check for 24hr roll-over (could roll-over on 13)
            hh = 0;      // 0 for 24 hour clock, set to 1 for 12 hour clock
          }
        }
      }

      // Update digital time
      int xpos = 160;
      int ypos = 210; // Top left corner ot clock text, about half way down
      int ysecs = ypos;

      if (omm != mm) { // Redraw hours and minutes time every minute
        omm = mm;
        // Draw hours and minutes
        if (hh < 10) xpos += M5.Lcd.drawChar('0', xpos, ypos, 1); // Add hours leading zero for 24 hr clock
        xpos += M5.Lcd.drawNumber(hh, xpos, ypos, 1);             // Draw hours
        xcolon = xpos; // Save colon coord for later to flash on/off later
        xpos += M5.Lcd.drawChar(':', xpos, ypos, 1);
        if (mm < 10) xpos += M5.Lcd.drawChar('0', xpos, ypos, 1); // Add minutes leading zero
        xpos += M5.Lcd.drawNumber(mm, xpos, ypos, 1);             // Draw minutes
        xsecs = xpos; // Sae seconds 'x' position for later display updates
      }
      if (oss != ss) { // Redraw seconds time every second
        oss = ss;
        xpos = xsecs;

        if (ss % 2) { // Flash the colons on/off
          M5.Lcd.setTextColor(0x39C4, TFT_BLACK);        // Set colour to grey to dim colon
          M5.Lcd.drawChar(':', xcolon, ypos, 1);     // Hour:minute colon
          xpos += M5.Lcd.drawChar(':', xsecs, ysecs, 1); // Seconds colon
          M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);    // Set colour back to yellow
        }
        else {
          M5.Lcd.drawChar(':', xcolon, ypos, 1);     // Hour:minute colon
          xpos += M5.Lcd.drawChar(':', xsecs, ysecs, 1); // Seconds colon
        }

        //Draw seconds
        if (ss < 10) xpos += M5.Lcd.drawChar('0', xpos, ysecs, 1); // Add leading zero
        M5.Lcd.drawNumber(ss, xpos, ysecs, 1);                     // Draw seconds
      }
    }

    if (!readPMSdata(&pmsSerial)) {
      Serial.write("read measurement failed\n");
      M5.Lcd.drawString("Read measurement failed", 1, 60, GFXFF);

    } else {
      Serial.write("MC 1.0: ");  Serial.println(data.pm10_env, DEC);
      Serial.write("MC 2.5: ");  Serial.println(data.pm25_env, DEC);
      Serial.write("MC 10.0: ");  Serial.println(data.pm100_env, DEC);
      Serial.write("MC 1.0: ");  Serial.println(data.pm10_standard, DEC);
      Serial.write("MC 2.5: ");  Serial.println(data.pm25_standard, DEC);
      Serial.write("MC 10.0: ");  Serial.println(data.pm100_standard, DEC);

      // read without samples.
      byte temperature = 0;
      byte humidity = 0;
      int err = SimpleDHTErrSuccess;
      if ((err = dht22.read(&temperature, &humidity, NULL)) != SimpleDHTErrSuccess) {
        Serial.print("Read DHT22 failed, err="); Serial.println(err); delay(2000);
        return;
      }

      Serial.print("Sample OK: ");
      Serial.print((int)temperature); Serial.print(" *C, ");
      Serial.print((int)humidity); Serial.println(" RH%");
      M5.Lcd.drawString("Temp:  ", 250, 170, 1);
      M5.Lcd.drawNumber((int)temperature, 300, 170, 1);
      M5.Lcd.drawString("Humid:  ", 250, 190, 1);
      M5.Lcd.drawNumber((int)humidity, 300, 190, 1);
 
      /*
        int pm25;
        if (measurement.mc_2p5 < 10) {
        pm25 = measurement.mc_2p5*2;
        } else {
        pm25 = measurement.mc_2p5*(1+(1/log10 (measurement.mc_2p5)));
        }
      */
      int pm25;
      pm25 = data.pm25_env;
      plotNeedle(pm25, 0);

      M5.Lcd.drawString("PM1:   ", 160, 170, 1);
      M5.Lcd.drawString("PM2.5: ", 160, 180, 1);
      M5.Lcd.drawString("PM10:  ", 160, 190, 1);

      //M5.Lcd.setTextSize(2);
      M5.Lcd.drawNumber(data.pm10_env, 200, 170, 1);
      M5.Lcd.drawNumber(data.pm25_env, 200, 180, 1);
      M5.Lcd.drawNumber(data.pm100_env, 200, 190, 1);
      //M5.Lcd.setTextSize(1);
    }
  }
}

// #########################################################################
//  Draw the analogue meter on the screen
// #########################################################################
void analogMeter() {
  // Meter outline
  // M5.Lcd.fillRect(0, 0, M_SIZE*239, M_SIZE*126, TFT_GREY);
  M5.Lcd.fillRect(5, 3, M_SIZE * 230, M_SIZE * 119, TFT_WHITE);

  M5.Lcd.setTextColor(TFT_BLACK);  // Text colour

  // CN AQI
  // Draw ticks every 5 degrees from -50 to +50 degrees (100 deg. FSD swing)
  for (int i = -50; i < 51; i += 5) {
    // Long scale tick length
    int tl = 45;

    // Coodinates of tick to draw
    float sx = cos((i - 90) * 0.0174532925);
    float sy = sin((i - 90) * 0.0174532925);
    uint16_t x0 = sx * (M_SIZE * 100 + tl) + M_SIZE * 120;
    uint16_t y0 = sy * (M_SIZE * 100 + tl) + M_SIZE * 160;
    uint16_t x1 = sx * M_SIZE * 100 + M_SIZE * 120;
    uint16_t y1 = sy * M_SIZE * 100 + M_SIZE * 160;

    // Coordinates of next tick for zone fill
    float sx2 = cos((i + 5 - 90) * 0.0174532925);
    float sy2 = sin((i + 5 - 90) * 0.0174532925);
    int x2 = sx2 * (M_SIZE * 100 + tl) + M_SIZE * 120;
    int y2 = sy2 * (M_SIZE * 100 + tl) + M_SIZE * 160;
    int x3 = sx2 * M_SIZE * 100 + M_SIZE * 120;
    int y3 = sy2 * M_SIZE * 100 + M_SIZE * 160;

    // Green zone limits
    if (i >= -50 && i < -35) {
      M5.Lcd.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_GREEN);
      M5.Lcd.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_GREEN);
    }

    // Yellow zone limits
    if (i >= -35 && i < -20) {
      M5.Lcd.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_YELLOW);
      M5.Lcd.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_YELLOW);
    }

    // Orange zone limits
    if (i >= -20 && i < -5) {
      M5.Lcd.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_ORANGE);
      M5.Lcd.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_ORANGE);
    }

    // Red zone limits
    if (i >= -5 && i < 10) {
      M5.Lcd.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_RED);
      M5.Lcd.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_RED);
    }

    // Magenta zone limits
    if (i >= 10 && i < 50) {
      M5.Lcd.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_MAGENTA);
      M5.Lcd.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_MAGENTA);
    }

    // Short scale tick length
    // if (i % 10 != 0) tl = 8;

    // Check if labels should be drawn, with position tweaks
    if (i % 10 == 0) {
      // Calculate label positions
      x0 = sx * (M_SIZE * 100 + tl + 5) + M_SIZE * 120;
      y0 = sy * (M_SIZE * 100 + tl + 5) + M_SIZE * 160;
      switch (i / 10) {
        case -5: M5.Lcd.drawCentreString("0", x0, y0 - 14, 1); break;
        case -4: M5.Lcd.drawCentreString("25", x0, y0 - 14, 1); break;
        case -3: M5.Lcd.drawCentreString("50", x0, y0 - 12, 1); break;
        case -2: M5.Lcd.drawCentreString("75", x0, y0 - 12, 1); break;
        case -1: M5.Lcd.drawCentreString("100", x0, y0 - 10, 1); break;
        case 0: M5.Lcd.drawCentreString("125", x0, y0 - 10, 1); break;
        case 1: M5.Lcd.drawCentreString("150", x0, y0 - 10, 1); break;
        case 2: M5.Lcd.drawCentreString("175", x0, y0 - 12, 1); break;
        case 3: M5.Lcd.drawCentreString("200", x0, y0 - 12, 1); break;
        case 4: M5.Lcd.drawCentreString("225", x0, y0 - 14, 1); break;
        case 5: M5.Lcd.drawCentreString("250", x0, y0 - 14, 1); break;
      }
    }

    // Recalculate coords incase tick lenght changed
    x0 = sx * (M_SIZE * 100 + tl) + M_SIZE * 120;
    y0 = sy * (M_SIZE * 100 + tl) + M_SIZE * 160;
    x1 = sx * M_SIZE * 100 + M_SIZE * 120;
    y1 = sy * M_SIZE * 100 + M_SIZE * 160;

    // Draw tick
    M5.Lcd.drawLine(x0, y0, x1, y1, TFT_BLACK);

    // Now draw the arc of the scale
    sx = cos((i + 5 - 90) * 0.0174532925);
    sy = sin((i + 5 - 90) * 0.0174532925);
    x0 = sx * M_SIZE * 100 + M_SIZE * 120;
    y0 = sy * M_SIZE * 100 + M_SIZE * 160;
    // Draw scale arc, don't draw the last part
    if (i < 50) M5.Lcd.drawLine(x0, y0, x1, y1, TFT_BLACK);
  }

  // US AQI
  // Draw ticks every 5 degrees from -50 to +50 degrees (100 deg. FSD swing)
  for (int i = -50; i < 51; i += 5) {
    // Long scale tick length
    int tl = 30;

    // Coodinates of tick to draw
    float sx = cos((i - 90) * 0.0174532925);
    float sy = sin((i - 90) * 0.0174532925);
    uint16_t x0 = sx * (M_SIZE * 100 + tl) + M_SIZE * 120;
    uint16_t y0 = sy * (M_SIZE * 100 + tl) + M_SIZE * 160;
    uint16_t x1 = sx * M_SIZE * 100 + M_SIZE * 120;
    uint16_t y1 = sy * M_SIZE * 100 + M_SIZE * 160;

    // Coordinates of next tick for zone fill
    float sx2 = cos((i + 5 - 90) * 0.0174532925);
    float sy2 = sin((i + 5 - 90) * 0.0174532925);
    int x2 = sx2 * (M_SIZE * 100 + tl) + M_SIZE * 120;
    int y2 = sy2 * (M_SIZE * 100 + tl) + M_SIZE * 160;
    int x3 = sx2 * M_SIZE * 100 + M_SIZE * 120;
    int y3 = sy2 * M_SIZE * 100 + M_SIZE * 160;

    // Green zone limits
    if (i >= -50 && i < -45) {
      M5.Lcd.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_GREEN);
      M5.Lcd.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_GREEN);
    }

    // Yellow zone limits
    if (i >= -45 && i < -35) {
      M5.Lcd.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_YELLOW);
      M5.Lcd.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_YELLOW);
    }

    // Orange zone limits
    if (i >= -35 && i < -25) {
      M5.Lcd.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_ORANGE);
      M5.Lcd.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_ORANGE);
    }

    // Red zone limits
    if (i >= -25 && i < 10) {
      M5.Lcd.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_RED);
      M5.Lcd.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_RED);
    }

    // Magenta zone limits
    if (i >= 10 && i < 50) {
      M5.Lcd.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_MAGENTA);
      M5.Lcd.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_MAGENTA);
    }

    // Short scale tick length
    // if (i % 10 != 0) tl = 8;

    // Recalculate coords incase tick lenght changed
    x0 = sx * (M_SIZE * 100 + tl) + M_SIZE * 120;
    y0 = sy * (M_SIZE * 100 + tl) + M_SIZE * 160;
    x1 = sx * M_SIZE * 100 + M_SIZE * 120;
    y1 = sy * M_SIZE * 100 + M_SIZE * 160;

    // Draw tick
    M5.Lcd.drawLine(x0, y0, x1, y1, TFT_BLACK);

    // Now draw the arc of the scale
    sx = cos((i + 5 - 90) * 0.0174532925);
    sy = sin((i + 5 - 90) * 0.0174532925);
    x0 = sx * M_SIZE * 100 + M_SIZE * 120;
    y0 = sy * M_SIZE * 100 + M_SIZE * 160;
    // Draw scale arc, don't draw the last part
    if (i < 50) M5.Lcd.drawLine(x0, y0, x1, y1, TFT_BLACK);
  }

  // TH AQI
  // Draw ticks every 5 degrees from -50 to +50 degrees (100 deg. FSD swing)
  for (int i = -50; i < 51; i += 5) {
    // Long scale tick length
    int tl = 15;

    // Coodinates of tick to draw
    float sx = cos((i - 90) * 0.0174532925);
    float sy = sin((i - 90) * 0.0174532925);
    uint16_t x0 = sx * (M_SIZE * 100 + tl) + M_SIZE * 120;
    uint16_t y0 = sy * (M_SIZE * 100 + tl) + M_SIZE * 160;
    uint16_t x1 = sx * M_SIZE * 100 + M_SIZE * 120;
    uint16_t y1 = sy * M_SIZE * 100 + M_SIZE * 160;

    // Coordinates of next tick for zone fill
    float sx2 = cos((i + 5 - 90) * 0.0174532925);
    float sy2 = sin((i + 5 - 90) * 0.0174532925);
    int x2 = sx2 * (M_SIZE * 100 + tl) + M_SIZE * 120;
    int y2 = sy2 * (M_SIZE * 100 + tl) + M_SIZE * 160;
    int x3 = sx2 * M_SIZE * 100 + M_SIZE * 120;
    int y3 = sy2 * M_SIZE * 100 + M_SIZE * 160;

    // Blue zone limits
    if (i >= -50 && i < -40) {
      M5.Lcd.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_BLUE);
      M5.Lcd.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_BLUE);
    }

    // Green zone limits
    if (i >= -40 && i < -35) {
      M5.Lcd.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_GREEN);
      M5.Lcd.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_GREEN);
    }

    // Yellow zone limits
    if (i >= -35 && i < -30) {
      M5.Lcd.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_YELLOW);
      M5.Lcd.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_YELLOW);
    }

    // Orange zone limits
    if (i >= -30 && i < -15) {
      M5.Lcd.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_ORANGE);
      M5.Lcd.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_ORANGE);
    }

    // Red zone limits
    if (i >= -15 && i < 50) {
      M5.Lcd.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_RED);
      M5.Lcd.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_RED);
    }
    // Short scale tick length
    // if (i % 10 != 0) tl = 8;

    // Recalculate coords incase tick lenght changed
    x0 = sx * (M_SIZE * 100 + tl) + M_SIZE * 120;
    y0 = sy * (M_SIZE * 100 + tl) + M_SIZE * 160;
    x1 = sx * M_SIZE * 100 + M_SIZE * 120;
    y1 = sy * M_SIZE * 100 + M_SIZE * 160;

    // Draw tick
    M5.Lcd.drawLine(x0, y0, x1, y1, TFT_BLACK);

    // Now draw the arc of the scale
    sx = cos((i + 5 - 90) * 0.0174532925);
    sy = sin((i + 5 - 90) * 0.0174532925);
    x0 = sx * M_SIZE * 100 + M_SIZE * 120;
    y0 = sy * M_SIZE * 100 + M_SIZE * 160;
    // Draw scale arc, don't draw the last part
    if (i < 50) M5.Lcd.drawLine(x0, y0, x1, y1, TFT_BLACK);
  }

  M5.Lcd.drawString("TH", M_SIZE * (30), M_SIZE * (119 - 20), 2);
  M5.Lcd.drawString("US", M_SIZE * (20), M_SIZE * (119 - 30), 2);
  M5.Lcd.drawString("CN", M_SIZE * (10), M_SIZE * (119 - 40), 2);
  M5.Lcd.drawString("ug/m3", M_SIZE * (5 + 230 - 30), M_SIZE * (139 - 30), 2); // Units at bottom right
  M5.Lcd.drawCentreString("PM2.5", M_SIZE * 120, M_SIZE * 90, 4);
  M5.Lcd.drawRect(5, 3, M_SIZE * 230, M_SIZE * 119, TFT_BLACK); // Draw bezel line

  M5.Lcd.setTextColor(TFT_YELLOW);  // Text colour
  M5.Lcd.drawString("FB:PM2.5daily", 160, 220, 2);

  plotNeedle(0, 0); // Put meter needle at 0
}

// #########################################################################
// Update needle position
// This function is blocking while needle moves, time depends on ms_delay
// 10ms minimises needle flicker if text is drawn within needle sweep area
// Smaller values OK if text not in sweep area, zero for instant movement but
// does not look realistic... (note: 100 increments for full scale deflection)
// #########################################################################
void plotNeedle(int value, byte ms_delay)
{
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  char buf[8]; dtostrf(value, 4, 0, buf);
  M5.Lcd.drawString(buf, M_SIZE * 10, M_SIZE * (119 + 15), 7);

  value = value / 2.5;
  if (value < -10) value = -10; // Limit value to emulate needle end stops
  if (value > 110) value = 110;

  // Move the needle until new value reached
  while (!(value == old_analog)) {
    if (old_analog < value) old_analog++;
    else old_analog--;

    if (ms_delay == 0) old_analog = value; // Update immediately if delay is 0

    float sdeg = map(old_analog, -10, 110, -150, -30); // Map value to angle
    // Calcualte tip of needle coords
    float sx = cos(sdeg * 0.0174532925);
    float sy = sin(sdeg * 0.0174532925);

    // Calculate x delta of needle start (does not start at pivot point)
    float tx = tan((sdeg + 90) * 0.0174532925);

    // Erase old needle image
    M5.Lcd.drawLine(M_SIZE * (120 + 20 * ltx - 1), M_SIZE * (140 - 20), osx - 1, osy, TFT_WHITE);
    M5.Lcd.drawLine(M_SIZE * (120 + 20 * ltx), M_SIZE * (140 - 20), osx, osy, TFT_WHITE);
    M5.Lcd.drawLine(M_SIZE * (120 + 20 * ltx + 1), M_SIZE * (140 - 20), osx + 1, osy, TFT_WHITE);

    // Re-plot text under needle
    M5.Lcd.setTextColor(TFT_BLACK);
    M5.Lcd.drawCentreString("PM2.5", M_SIZE * 120, M_SIZE * 90, 4); // // Comment out to avoid font 4

    // Store new needle end coords for next erase
    ltx = tx;
    osx = M_SIZE * (sx * 98 + 120);
    osy = M_SIZE * (sy * 98 + 160);

    // Draw the needle in the new postion, magenta makes needle a bit bolder
    // draws 3 lines to thicken needle
    M5.Lcd.drawLine(M_SIZE * (120 + 20 * ltx - 1), M_SIZE * (140 - 20), osx - 1, osy, TFT_RED);
    M5.Lcd.drawLine(M_SIZE * (120 + 20 * ltx), M_SIZE * (140 - 20), osx, osy, TFT_MAGENTA);
    M5.Lcd.drawLine(M_SIZE * (120 + 20 * ltx + 1), M_SIZE * (140 - 20), osx + 1, osy, TFT_RED);

    // Slow needle down slightly as it approaches new postion
    if (abs(old_analog - value) < 10) ms_delay += ms_delay / 5;

    // Wait before next update
    delay(ms_delay);
  }
}

// Function to extract numbers from compile time string
static uint8_t conv2d(const char* p) {
  uint8_t v = 0;
  if ('0' <= *p && *p <= '9')
    v = *p - '0';
  return 10 * v + *++p - '0';
}

void printTest() {

  // reading data was successful!
  Serial.println();
  Serial.println("---------------------------------------");
  Serial.println("Concentration Units (standard)");
  Serial.print("PM 1.0: "); Serial.print(data.pm10_standard);
  Serial.print("\t\tPM 2.5: "); Serial.print(data.pm25_standard);
  Serial.print("\t\tPM 10: "); Serial.println(data.pm100_standard);
  Serial.println("---------------------------------------");
  Serial.println("Concentration Units (environmental)");
  Serial.print("PM 1.0: "); Serial.print(data.pm10_env);
  Serial.print("\t\tPM 2.5: "); Serial.print(data.pm25_env);
  Serial.print("\t\tPM 10: "); Serial.println(data.pm100_env);
  Serial.println("---------------------------------------");
  Serial.print("Particles > 0.3um / 0.1L air:"); Serial.println(data.particles_03um);
  Serial.print("Particles > 0.5um / 0.1L air:"); Serial.println(data.particles_05um);
  Serial.print("Particles > 1.0um / 0.1L air:"); Serial.println(data.particles_10um);
  Serial.print("Particles > 2.5um / 0.1L air:"); Serial.println(data.particles_25um);
  Serial.print("Particles > 5.0um / 0.1L air:"); Serial.println(data.particles_50um);
  Serial.print("Particles > 10.0 um / 0.1L air:"); Serial.println(data.particles_100um);
  Serial.println("---------------------------------------");

}

boolean readPMSdata(Stream *s) {
  if (! s->available()) {
    return false;
  }

  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }

  // Now read all 32 bytes
  if (s->available() < 32) {
    return false;
  }

  uint8_t buffer[32];
  uint16_t sum = 0;
  s->readBytes(buffer, 32);

  // get checksum ready
  for (uint8_t i = 0; i < 30; i++) {
    sum += buffer[i];
  }

  /* debugging
    for (uint8_t i=2; i<32; i++) {
    Serial.print("0x"); Serial.print(buffer[i], HEX); Serial.print(", ");
    }
    Serial.println();
  */

  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i = 0; i < 15; i++) {
    buffer_u16[i] = buffer[2 + i * 2 + 1];
    buffer_u16[i] += (buffer[2 + i * 2] << 8);
  }

  // put it into a nice struct 🙂
  memcpy((void *)&data, (void *)buffer_u16, 30);

  if (sum != data.checksum) {
    Serial.println("Checksum failure");
    return false;
  }
  // success!
  return true;
}
