/*
Using the Arduino IDE, install the library "ESP8266 SSD1306 OLED" Display lib.
This code was developed with a Wemos D1 Mini (ESP8266).
Author: Pierre LACLAU, December 2017.
*/
// Including rosserial
#include <ros.h>
#include <std_msgs/Empty.h>
#include <drivers_ard_hmi/SetStrategies.h>

//Creating OLED display instances
#include <Wire.h>    // Only needed for Arduino 1.6.5 and earlier
#include "SSD1306.h" // alias for `#include "SSD1306Wire.h"`
#include "OLEDDisplayUi.h"
SSD1306  display(0x3c, D2, D1);
OLEDDisplayUi ui(&display);

//HMI variables
String options[10];
uint8_t options_count = 8;
uint8_t cursor_pos;

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

void drawMCQComponent(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
    if(options_count > 0) {
        int y_offset = 12;
        display->setFont(ArialMT_Plain_10);
        display->setTextAlignment(TEXT_ALIGN_LEFT);
    
        int start = cursor_pos - 1;
        if(cursor_pos == 0) start = 0;
        if(cursor_pos == options_count - 1) {
          start = options_count - 3;
          if(options_count == 2) start = options_count - 2;
          if(options_count == 1) start = options_count - 1;
        }
    
        for(int i = 0; i < 3; i++) {
            display->drawString(11 + x, 16 + y + y_offset * i, options[start + i]);
            if(cursor_pos == start + i) {
                display->drawString(4 + x, 16 + y + y_offset * i, ">");
            }
        }

        char b[5];
        sprintf(b, "%d/%d",cursor_pos + 1,options_count);
        display->drawString(4 + x, 52 + y, b);
    }
    else {
        display->setFont(ArialMT_Plain_10);
        display->setTextAlignment(TEXT_ALIGN_CENTER);
        display->drawString(64 + x, 28 + y, "No options.");
    }
}

void drawHelloFrame(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
    drawFrameTitleComponent(display, state, x, y, "RPi init...");
    drawBigCentralMessageComponent(display, state, x, y, "WAIT");
}

void drawTeamFrame(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
    drawFrameTitleComponent(display, state, x, y, "Team select");
    drawMCQComponent(display, state, x, y);
}

void drawStrategyFrame(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
    drawFrameTitleComponent(display, state, x, y, "Strat select");
    drawMCQComponent(display, state, x, y);
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
    ui.disableIndicator();
    drawFrameTitleComponent(display, state, x, y, "In Game...");
    
    display->setTextAlignment(TEXT_ALIGN_CENTER);
    display->setFont(ArialMT_Plain_24);
    display->drawString(64 + x, 22 + y, "178pts");

    display->drawProgressBar(x + 0, y + 54, 120, 8, 67); // shows time left
}

// frames are the single views that slide in
FrameCallback frames[] = {drawHelloFrame, drawTeamFrame, drawStrategyFrame, drawArmFrame, drawJackFrame, drawInGameFrame};
int frameCount = 6;

// Overlays are statically drawn on top of a frame
//OverlayCallback overlays[] = { msOverlay };
//int overlaysCount = 1;

void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println("Hello");

    ui.setTargetFPS(20);

    ui.setActiveSymbol(activeSymbol);
    ui.setInactiveSymbol(inactiveSymbol);
    ui.setFrameAnimation(SLIDE_LEFT);

    ui.setFrames(frames, frameCount);
    //ui.setOverlays(overlays, overlaysCount);

    ui.disableAutoTransition();
    ui.init();

    display.flipScreenVertically();
    ui.switchToFrame(5); //tmp for tests

    options[0] = "strat1";
    options[1] = "strategy_ftw";
    options[2] = "rcva_going_to_death";
    options[3] = "insa_is_trash";
    options[4] = "hi";
    options[5] = "MajusCules";
    options[6] = "bonjour";
    options[7] = "la derniere.";
}


void loop() {
    int remainingTimeBudget = ui.update();
    if(Serial.available() > 0) { //tmp
          char c = Serial.read();
          if(c == 'l') ui.nextFrame();
          if(c == 'j') ui.previousFrame();
          if(c == 'k' && options_count) {
            cursor_pos += 1;
            if(cursor_pos >= options_count) cursor_pos = options_count - 1;
          }
          if(c == 'i' && options_count) {
            if(cursor_pos > 0) cursor_pos -= 1;
          }
        }
    Serial.println(cursor_pos);
    if (remainingTimeBudget > 0) {
        delay(remainingTimeBudget);
    }
}
