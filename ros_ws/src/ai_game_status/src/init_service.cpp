#include "ai_game_status/init_service.h"

using namespace std;

const string READY_SRV = "/ai/game_status/node_ready";
const string ARM_SRV = "/arm";
const string HALT_SRV = "/ai/game_status/status";

StatusServices::StatusServices(const string& namespaceName, const string& packageName)
{
    _nodeName = "/" + namespaceName + "/" + packageName;
}