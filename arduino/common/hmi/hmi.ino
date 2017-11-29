/*
Using the Arduino IDE, install the library "ESP8266 SSD1306 OLED" Display lib.
This code was developed with a Wemos D1 Mini (ESP8266).
*/

#include <Wire.h>    // Only needed for Arduino 1.6.5 and earlier
#include "SSD1306.h" // alias for `#include "SSD1306Wire.h"`

// Create UI and Display.
#include "OLEDDisplayUi.h"
SSD1306  display(0x3c, D2, D1);
OLEDDisplayUi ui(&display);


const char activeSymbol[] PROGMEM = {
    B00000000,
    B00000000,
    B00011000,
    B00100100,
    B01000010,
    B01000010,
    B00100100,
    B00011000
};

const char inactiveSymbol[] PROGMEM = {
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00011000,
    B00011000,
    B00000000,
    B00000000
};

void msOverlay(OLEDDisplay *display, OLEDDisplayUiState* state) {
    // display->setTextAlignment(TEXT_ALIGN_RIGHT);
    // display->setFont(ArialMT_Plain_10);
    // display->drawString(128, 0, String(millis()));
}

void drawFrameTitleComponent(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y, String text) {
    display->setFont(ArialMT_Plain_16);
    display->setTextAlignment(TEXT_ALIGN_CENTER);
    display->drawString(64 + x, 0 + y, text);
}

void drawBigCentralMessageComponent(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y, String text) {
    display->setFont(ArialMT_Plain_24);
    display->setTextAlignment(TEXT_ALIGN_CENTER);
    display->drawString(64 + x, 22 + y, text);
}

void drawMCQComponent(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y, String[] options, int num_options) {
    int y_offset
    for(i = 0; i < num_options; i++) {

    }
}

void drawHelloFrame(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
    drawFrameTitleComponent(display, state, x, y, "RPi init...");
    drawBigCentralMessageComponent(display, state, x, y, "WAIT");
}

void drawTeamFrame(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
    drawFrameTitleComponent(display, state, x, y, "Team select");
}

void drawStrategyFrame(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
    drawFrameTitleComponent(display, state, x, y, "Strat select");
}

void drawArmFrame(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
    drawFrameTitleComponent(display, state, x, y, "Calibration");
    drawBigCentralMessageComponent(display, state, x, y, "ARM?");
}

void drawJackFrame(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
    drawFrameTitleComponent(display, state, x, y, "Jack");
    drawBigCentralMessageComponent(display, state, x, y, "JACK?");
}

void drawInGameFrame(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
    drawFrameTitleComponent(display, state, x, y, "In Game");
    display->setTextAlignment(TEXT_ALIGN_LEFT);
    display->setFont(ArialMT_Plain_10);
    display->drawStringMaxWidth(0 + x, 15 + y, 128, "Lorem ipsum\n dolor sit amet, consetetur sadipscing elitr.");

    display.drawProgressBar(0, 52, 120, 10, 67);
}

// frames are the single views that slide in
FrameCallback frames[] = {drawHelloFrame, drawTeamFrame, drawStrategyFrame, drawArmFrame, drawJackFrame, drawInGameFrame};
int frameCount = 6;

// Overlays are statically drawn on top of a frame
OverlayCallback overlays[] = { msOverlay };
int overlaysCount = 1;

void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println();

    ui.setTargetFPS(20);

    ui.setInactiveSymbol(inactiveSymbol);
    ui.setFrameAnimation(SLIDE_LEFT);

    ui.setFrames(frames, frameCount);
    ui.setOverlays(overlays, overlaysCount);

    ui.disableAutoTransition();
    ui.init();

    display.flipScreenVertically();
    ui.nextFrame();
}


void loop() {
    int remainingTimeBudget = ui.update();
    if(Serial.available() > 0) {
          char c = Serial.read();
          if(c == 'n') ui.nextFrame();
          if(c == 'p') ui.previousFrame();
        }
    if (remainingTimeBudget > 0) {
        delay(remainingTimeBudget);
    }
}

