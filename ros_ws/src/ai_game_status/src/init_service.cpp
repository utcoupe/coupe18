#include "ai_game_status/init_service.h"

using namespace std;

const string READY_SRV = "/ai/game_status/node_ready";
const string ARM_SRV = "/arm";
const string HALT_SRV = "/ai/game_status/status";
const auto TIMEOUT_READY_SRV = ros::Duration(15.0);

StatusServices::StatusServices(const string& namespaceName, const string& packageName, ArmCallback_t armCallback, StatusCallback_t statusCallback) :
    _armCallback(armCallback),
    _statusCallback(statusCallback)
{
    _nodeName = "/" + namespaceName + "/" + packageName;
    if (_armCallback)
        _armServer = _nodeHandle.advertiseService(ARM_SRV, &StatusServices::_on_arm, this);
    if (_statusCallback)
        _gameStatusSubscriber = _nodeHandle.subscribe(HALT_SRV, 10, &StatusServices::_on_gameStatus, this);
}

void StatusServices::setReady(bool success)
{
    try
    {
        if (!ros::service::waitForService(READY_SRV, TIMEOUT_READY_SRV))
            throw Errors::SERVICE_TIMEOUT;
        ros::ServiceClient readyPub = _nodeHandle.serviceClient<ai_game_status::NodeReady>(READY_SRV);
        ai_game_status::NodeReady msg;
        msg.request.success = success;
        msg.request.node_name = _nodeName;
        if (!readyPub.call(msg))
            throw Errors::SERVICE_NOT_RESPONDING;
        if (success)
            ROS_INFO_STREAM("Node " << _nodeName << " initialized successfully.");
        else
            ROS_ERROR_STREAM("Node " << _nodeName << " didn't initialize correctly.");
    }
    catch(...)
    {
        ROS_ERROR("status_services couldn't contact ai/game_status to send init notification.");
    }
}


bool StatusServices::_on_arm(ai_game_status::ArmRequest::Request &req, ai_game_status::ArmRequest::Response &rep)
{
    rep.success = _armCallback();
    return true;
}

void StatusServices::_on_gameStatus(const ai_game_status::GameStatus::ConstPtr& msg)
{
    _statusCallback(msg);
}
