#ifndef AI_GAME_STATUS_INIT_SERVICE_H
#define AI_GAME_STATUS_INIT_SERVICE_H

#include <ros/ros.h>

#include "ai_game_status/GameStatus.h"
#include "ai_game_status/ArmRequest.h"
#include "ai_game_status/NodeReady.h"

#include <functional>

class StatusServices
{
public:
    typedef std::function<bool()> ArmCallback_t;
    typedef std::function<void(const ai_game_status::GameStatus::ConstPtr&)> StatusCallback_t;
    
    StatusServices(const std::string& namespaceName, const std::string& packageName, ArmCallback_t armCallback = nullptr, StatusCallback_t statusCallback = nullptr);
    
    void setReady(bool success);

private:
    enum class Errors {SERVICE_TIMEOUT, SERVICE_NOT_RESPONDING};
    
    std::string _nodeName;
    ArmCallback_t _armCallback;
    StatusCallback_t _statusCallback;
    ros::NodeHandle _nodeHandle;
    ros::ServiceServer _armServer;
    ros::Subscriber _gameStatusSubscriber;

    bool _on_arm(ai_game_status::ArmRequest::Request &req, ai_game_status::ArmRequest::Response &rep);
    void _on_gameStatus(const ai_game_status::GameStatus::ConstPtr& msg);
};

#endif // AI_GAME_STATUS_INIT_SERVICE_H
