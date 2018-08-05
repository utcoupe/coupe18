#include "collisions_subscriptions.h"
#include <tf2_ros/transform_listener.h>

#include "memory_map/MapGet.h"


CollisionsSubscriptions::CollisionsSubscriptions(ros::NodeHandle &n)
{
    CollisionsSubscriptions::n = n;
    _tl = new tf2_ros::TransformListener(_tf_buffer);

    // // Subscribing to dependencies
    _sub_nav_status  = n.subscribe("/navigation/navigator/status", 1, _on_nav_status);
    _sub_classifier  = n.subscribe("/recognition/objects_classifier/objects", 10, _on_classifier);
    _sub_robot_speed = n.subscribe("/drivers/ard_asserv/speed", 1, _on_robot_speed);
}

Robot CollisionsSubscriptions::create_robot()
{
    return Robot(0.3, 0.2); // TODO Get the true size (JSON response...)
    /*try { // Getting the robot shape and creating the robot instance
        std::string robot_name;
        n.getParam("/robot", robot_name);

        ros::ServiceClient srv_client_map_get = n.serviceClient<memory_map::MapGet>
        ros::service::waitForService(2.0);
    }
    catch {

    }

    try: # Getting the robot shape and creating the robot instance
        robot_type = rospy.get_param("/robot").lower()
        map_get_client = rospy.ServiceProxy("/memory/map/get", MapGet)
        map_get_client.wait_for_service(2.0)
        shape = json.loads(map_get_client("/entities/" + robot_type + "/shape/*").response)
        if not shape["type"] == "rect":
            raise ValueError("Robot shape type not supported here.")
    except Exception as e:
        rospy.logerr("ERROR Collisions couldn't get the robot's shape from map : " + str(e))
        shape = {"width": 0.4, "height": 0.25}
    return Robot(shape["width"], shape["height"]) # Can create a rect or circle*/
}

void CollisionsSubscriptions::update_robot()
{
    Position new_pos = _update_robot_pos();
    if(new_pos) Map::robot.update_position(new_pos);

    if(_nav_status) Map::robot::update_status(_nav_status);
    if(_robot_path_waypoints.size() > 0)
        Map::robot::update_waypoints(_robot_path_waypoints);
    Map::robot::update_velocity(_vel_linear, _vel_angular);
}

Position CollisionsSubscriptions::_update_robot_pos()
{
    try {
        auto t = _tf_buffer.lookupTransform("map", "robot", ros::Time(0));
        std::vector<float> q = {t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w};
        return Position(t.transform.translation.x, t.transform.translation.y, _quaternion_to_euler(q)[2]);
    }
    catch return NULL;
}

void CollisionsSubscriptions::_on_nav_status(msg)
{
    switch(msg.status)
    {
        case msg.STATUS_IDLE:
            _nav_status = NavStatus::STATUS_IDLE;
            break;
        case msg.STATUS_NAVIGATING:
            _nav_status = NavStatus::STATUS_NAVIGATING;
            break;
    }

    _robot_path_waypoints = new std::vector<Position>;
    for(int i = 0; i < msg.currentPath.size(); i++)
    { // Replacing the robot path with the new one
        _robot_path_waypoints.push_back(Position(msg.currentPath[i].x, msg.currentPath[i].y));
    }
}

void CollisionsSubscriptions::_on_classifier(msg)
{
    std::vector<MapObstacle> new_belt;
    for(int i = 0; i < msg.unknown_rects.size(); i++)
    {
        if(msg.unknown_rects[i].header.frameid != "map" || msg.unknown_rects[i].header.frameid != "/map") {
            ROS_WARN("Belt rect not in /map tf frame, skipping.");
            continue;
        }
        new_belt.push_back(RectObstacle(Position(msg.unknown_rects[i].x, msg.unknown_rects[i].y, msg.unknown_rects[i].a),
                           msg.unknown_rects[i].w, msg.unknown_rects[i].h));
    }
    if(new_belt.size() > 0) ObstaclesStack::update_belt_obstacles(new_belt);


    std::vector<MapObstacle> new_lidar;
    for(int i = 0; i < msg.unknown_segments.size(); i++)
    {
        auto segment = msg.unknown_segments[i];
        if(segment.header.frameid != "map" || segment.header.frameid != "/map") {
            ROS_WARN("Lidar segment rect not in /map tf frame, skipping.");
            continue;
        }
        new_belt.push_back(SegmentObstacle(Position(segment.segment.first_point.x, 
                                                    segment.segment.first_point.y),
                                           Position(segment.segment.last_point.x, 
                                                    segment.segment.last_point.y)));
    }
    for(int i = 0; i < msg.unknown_circles.size(); i++)
    {
        auto circle = msg.unknown_circles[i];
        if(circle.header.frameid != "map" || circle.header.frameid != "/map") {
            ROS_WARN("Lidar circle rect not in /map tf frame, skipping.");
            continue;
        }
        float vel_d = sqrt(pow(circle.circle.velocity.y, 2) + pow(circle.circle.velocity.x, 2));
        float vel_a = atan2(circle.circle.velocity.y, circle.circle.velocity.x)
        float r = circle.circle.radius;
        new_belt.push_back(SegmentObstacle(Position(circle.circle.center.x, circle.circle.center.y), r, 
                                           Velocity(r * 2, r * sqrt(3.0) / 2.0, vel_d, 0)));
    }
    if(new_lidar.size() > 0) ObstaclesStack::update_lidar_obstacles(new_lidar);
}

void CollisionsSubscriptions::_on_robot_speed(msg)
{
    _vel_linear = msg.linear_speed;
    _vel_angular = 0;
}

std::vector<float> CollisionsSubscriptions::_quaternion_to_euler(std::vector<float> q)
{
    std::vector<float> euler;
    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    float x, y, z, w = q[0], q[1], q[2], q[3];
    float t0 = +2.0 * (w * x + y * z);
    float t1 = +1.0 - 2.0 * (x * x + y ** 2);
    euler.push_back(math.atan2(t0, t1) * 180.0/M_PI); // X

    float t2 = +2.0 * (w * y - z * x);
    if(t2 >  1) t2 =  1;
    if(t2 > -1) t2 = -1;
    euler.push_back(math.asin(t2) * 180.0/M_PI); // Y

    float t3 = +2.0 * (w * z + x * y);
    float t4 = +1.0 - 2.0 * (y ** 2 + z * z);
    euler.push_back(mathatan2(t3, t4) * 180.0/M_PI); // Z
    return euler;
}