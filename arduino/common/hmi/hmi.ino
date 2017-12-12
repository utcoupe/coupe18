/*
Using the Arduino IDE, install the library "ESP8266 SSD1306 OLED" Display lib.
This code was developed with a Wemos D1 Mini (ESP8266) and a 128x64 i2c OLED.
Author: Pierre LACLAU, December 2017, UTCoupe.
*/

// Including rosserial
#include <ros.h>
#include <drivers_ard_hmi/SetStrategies.h>  // ROS sets strategy names for displaying on hmi.
#include <drivers_ard_hmi/SetTeams.h>       // ROS sets teams names for displaying on hmi.
#include <drivers_ard_hmi/SelectedConfig.h> // HMI sends selected team and strategy (id, not string).
#include <drivers_ard_hmi/HMIEvent.h>       // HMI sends events : JACK, GAME_STOP
#include <drivers_ard_hmi/ROSEvent.h>       // ROS sends events : ASK_JACK, GAME_STOP
//#include <ai_game_status/ROSEvent.h>       // ROS sends game and init status (to  determine when RPi ready).
#include <drivers_ard_hmi/ArrowClick.h> //tmp while no physical buttons
ros::NodeHandle nh;

//Creating OLED display instances
#include <Wire.h>    // Only needed for Arduino 1.6.5 and earlier
#include "SSD1306.h" // alias for `#include "SSD1306Wire.h"`
#include "OLEDDisplayUi.h"
SSD1306  display(0x3c, D2, D1);
OLEDDisplayUi ui(&display);

//Component variables
uint8_t current_frame;
uint8_t cursor_pos;

//HMI variables
String strats[12];
uint8_t strats_count = 8;

String teams[12];
uint8_t teams_count = 8;

uint8_t chosen_strat = -1;
uint8_t chosen_team = -1;

uint8_t game_status;
uint8_t init_status;
bool is_arming = false;

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

//Input methods
bool up_state    = false;
bool down_state  = false;
bool left_state  = false;
bool right_state = false;

bool upPressed() {
    bool backup = up_state;
    up_state = false;
    if(backup) return true;
    return false;
}

bool downPressed() {
    bool backup = down_state;
    down_state = false;
    if(backup) return true;
    return false;
}

bool leftPressed() {
    bool backup = left_state;
    left_state = false;
    if(backup) return true;
    return false;
}

bool rightPressed() {
    bool backup = right_state;
    right_state = false;
    if(backup) return true;
    return false;
}


//ROS methods
void on_set_strategies(const drivers_ard_hmi::SetStrategies& msg){
  strats_count = msg.strategies_names_length;
  for(int i=0; i < msg.strategies_names_length; i++) {
    strats[i] = msg.strategies_names[i];
  }
}

void on_set_teams(const drivers_ard_hmi::SetTeams& msg){
  teams_count = msg.teams_names_length;
  for(int i=0; i < msg.teams_names_length; i++) {
    teams[i] = msg.teams_names[i];
  }
}

//void on_game_status(const ai_game_status::GameStatus& msg){
//  game_status = msg.game_status;
//  init_status = msg.init_status;
//}

void on_click(const drivers_ard_hmi::ArrowClick& msg){
    if(msg.arrow == 0) up_state = true;
    if(msg.arrow == 1) down_state = true;
    if(msg.arrow == 2) left_state = true;
    if(msg.arrow == 3) right_state = true;
}

void on_ros_event(const drivers_ard_hmi::ROSEvent& msg){
}

ros::Subscriber<drivers_ard_hmi::SetStrategies> sub_strats("/feedback/ard_hmi/set_strategies", &on_set_strategies);
ros::Subscriber<drivers_ard_hmi::SetTeams> sub_teams("/feedback/ard_hmi/set_teams", &on_set_teams);
ros::Subscriber<drivers_ard_hmi::ArrowClick> sub_click("/feedback/ard_hmi/click", &on_click); //tmp
ros::Subscriber<drivers_ard_hmi::ROSEvent> sub_ros_events("/feedback/ard_hmi/ros_event", &on_ros_event);
//ros::Subscriber<ai_game_status::GameStatus> sub_game_status( "/ai/game_status/status", &on_game_status);

drivers_ard_hmi::SelectedConfig config_msg;
ros::Publisher config_pub("/feedback/ard_hmi/arm_config", &config_msg);
drivers_ard_hmi::HMIEvent hmi_event_msg;
ros::Publisher hmi_events_pub("/feedback/ard_hmi/hmi_event", &hmi_event_msg);




//Components methods
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

void drawMCQComponent(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y, String options[], uint8_t options_count) {
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

//Frames rendering methods
void drawHelloFrame(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
    current_frame = 0;
    drawFrameTitleComponent(display, state, x, y, "RPi init...");
    drawBigCentralMessageComponent(display, state, x, y, "WAIT");

    if(game_status == 0 && init_status != 0 && strats_count && teams_count)
        ui.nextFrame();
}

void drawTeamFrame(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
    current_frame = 1;
    drawFrameTitleComponent(display, state, x, y, "Team select");
    drawMCQComponent(display, state, x, y, teams, teams_count);

    if(rightPressed()) {
        chosen_team = cursor_pos;
        cursor_pos = 0;
        ui.nextFrame();
    }
    if(leftPressed()) {
        cursor_pos = 0;
        ui.previousFrame();
    }
    if(downPressed() && teams_count) {
        cursor_pos += 1;
        if(cursor_pos >= teams_count) cursor_pos = teams_count - 1;
    }
    if(upPressed() && teams_count) {
        if(cursor_pos > 0) cursor_pos -= 1;
    }
}

void drawStrategyFrame(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
    current_frame = 2;
    drawFrameTitleComponent(display, state, x, y, "Strat select");
    drawMCQComponent(display, state, x, y, strats, strats_count);

    if(rightPressed()) {
        chosen_strat = cursor_pos;
        cursor_pos = 0;
        ui.nextFrame();
    }
    if(leftPressed()) {
        cursor_pos = 0;
        ui.previousFrame();
    }
    if(downPressed() && strats_count) {
        cursor_pos += 1;
        if(cursor_pos >= strats_count) cursor_pos = strats_count - 1;
    }
    if(upPressed() && strats_count) {
        if(cursor_pos > 0) cursor_pos -= 1;
    }
}

void drawArmFrame(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
    current_frame = 3;
    drawFrameTitleComponent(display, state, x, y, "Calibration");
    if(is_arming)  drawBigCentralMessageComponent(display, state, x, y, "ARM?");
    if(!is_arming) drawBigCentralMessageComponent(display, state, x, y, "ARMING...");

    if(leftPressed()) ui.previousFrame();
    if(rightPressed()) {
      config_msg.team_id = chosen_team;
      config_msg.strategy_id = chosen_strat;
      config_pub.publish(&config_msg); // send config
      is_arming = true;
    }
}

void drawJackFrame(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
    current_frame = 4;
    drawFrameTitleComponent(display, state, x, y, "Armed");
    drawBigCentralMessageComponent(display, state, x, y, "JACK?");

    if(leftPressed()) ui.previousFrame();
    if(rightPressed()) ui.nextFrame();
}

void drawInGameFrame(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
    current_frame = 5;
    ui.disableIndicator();
    drawFrameTitleComponent(display, state, x, y, "In Game...");
    
    display->setTextAlignment(TEXT_ALIGN_CENTER);
    display->setFont(ArialMT_Plain_24);
    display->drawString(64 + x, 22 + y, "178pts");
    
    display->drawProgressBar(x + 0, y + 54, 120, 8, 67); // shows time left

    if(leftPressed()) ui.previousFrame();
    if(rightPressed()) ui.nextFrame();
}

// frames are the single views that slide in
FrameCallback frames[] = {drawHelloFrame, drawTeamFrame, drawStrategyFrame, drawArmFrame, drawJackFrame, drawInGameFrame};
int frameCount = 6;

// Overlays are statically drawn on top of a frame
//OverlayCallback overlays[] = { msOverlay };
//int overlaysCount = 1;

void setup() {
    nh.initNode();
    nh.subscribe(sub_strats);
    nh.subscribe(sub_teams);
    //nh.subscribe(sub_game_status);
    nh.subscribe(sub_click);
    nh.subscribe(sub_ros_events);
    nh.advertise(config_pub);
    nh.advertise(hmi_events_pub);
    
    ui.setTargetFPS(30);
    
    ui.setActiveSymbol(activeSymbol);
    ui.setInactiveSymbol(inactiveSymbol);
    ui.setFrameAnimation(SLIDE_LEFT);
    
    ui.setFrames(frames, frameCount);
    //ui.setOverlays(overlays, overlaysCount);
    
    ui.disableAutoTransition();
    ui.init();
    
    display.flipScreenVertically();
}


void loop() {
    ui.update();

    if(current_frame != 5 && game_status == 1 /*INGAME*/)
        ui.transitionToFrame(5); //if game starts, goto game screen no matter what.

    nh.spinOnce();
}
