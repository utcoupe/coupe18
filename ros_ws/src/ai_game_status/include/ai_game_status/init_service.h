#ifndef AI_GAME_STATUS_INIT_SERVICE_H
#define AI_GAME_STATUS_INIT_SERVICE_H

#include "ai_game_status/GameStatus.h"
#include "ai_game_status/ArmRequest.h"
#include "ai_game_status/NodeReady.h"

class StatusServices
{
public:
    StatusServices(const std::string& namespaceName, const std::string& packageName);
    
    void setReady(bool success);

private:
    std::string _nodeName;

    bool _on_arm(ai_game_status::ArmRequest::Request &req, ai_game_status::ArmRequest::Response &rep);
};

#endif // AI_GAME_STATUS_INIT_SERVICE_H