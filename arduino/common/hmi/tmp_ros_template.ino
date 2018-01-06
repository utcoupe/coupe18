/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Empty.h>
#include <drivers_ard_hmi/SetStrategies.h>
#include <drivers_ard_hmi/SetTeams.h>
#include <ai_game_status/GameStatus.h>

ros::NodeHandle nh;


uint8_t game_status;
uint8_t init_status;

String strats[12];
String teams[5];

void on_set_strategies(const drivers_ard_hmi::SetStrategies& msg){
  for(int i=0; i < msg.strategies_names_length; i++) {
    strats[i] = msg.strategies_names[i];
  }
}

void on_set_teams(const drivers_ard_hmi::SetTeams& msg){
  for(int i=0; i < msg.teams_length; i++) {
    strats[i] = msg.teams[i];
  }
}

void on_game_status(const ai_game_status::GameStatus& msg){
  game_status = msg.game_status;
  init_status = msg.init_status;
}

ros::Subscriber<drivers_ard_hmi::SetStrategies> sub_strats("/feedback/ard_hmi/set_strategies", &on_set_strategies);
ros::Subscriber<drivers_ard_hmi::SetTeams> sub_teams("/feedback/ard_hmi/set_teams", &on_set_teams);
ros::Subscriber<ai_game_status::GameStatus> sub_game_status( "/ai/game_status/status", &on_game_status);


void setup()
{
  pinMode(D4, OUTPUT);
  nh.initNode();
  nh.subscribe(sub_strats);
  nh.subscribe(sub_teams);
  nh.subscribe(sub_game_status);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}