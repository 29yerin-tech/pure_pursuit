#include "pure_pursuit/pure_pursuit_function.hpp"
#include <fstream>

double ld_straight;
double ld_curve;
double target_velocity;
double goal_tolerance;
std::string path_file_name;
std::string ref_file_name;

void gpsCallback(const morai_msgs::GPSMessage::ConstPtr& msg) {
    double x, y, z;
    wgs84ToENU(msg->latitude, msg->longitude, msg->altitude,
               lat0, lon0, h0, x, y, z);
    ego_x = x;
    ego_y = y;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    ego_yaw = quaternionToYaw(msg->orientation.x,
                              msg->orientation.y,
                              msg->orientation.z,
                              msg->orientation.w);

    if (!reached_first_wp) {
        if (!approachFirstWaypoint()) return;
    }

    if (checkGoalReached()) return;

    double yaw_rate = msg->angular_velocity.z;
    double ld = adjustLookaheadDistance(yaw_rate, ld_straight, ld_curve);

    auto target = findLookAheadPoint(ld);
    double alpha = computeAlpha(target);
    double steering_angle = computeSteering(alpha, ld);

    morai_msgs::CtrlCmd cmd;
    cmd.longlCmdType = 2;
    cmd.velocity = target_velocity;
    cmd.steering = steering_angle;
    cmd_pub.publish(cmd);

    ROS_INFO("Published CtrlCmd: v=%.2f, steer=%.2f, ld=%.2f", 
              target_velocity, steering_angle, ld);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pure_pursuit_node");
    ros::NodeHandle nh;

    nh.param("ld_straight", ld_straight, 6.0);
    nh.param("ld_curve", ld_curve, 3.0);
    nh.param("target_velocity", target_velocity, 10.0);
    nh.param("goal_tolerance", goal_tolerance, 1.0);
    nh.getParam("path_file_name", path_file_name);
    nh.getParam("ref_file_name", ref_file_name);

    // 🔹 어떤 파일을 열고 있는지 출력
    ROS_INFO("Opening ref file: %s", ref_file_name.c_str());
    std::ifstream ref_file(ref_file_name);
    if (!ref_file.is_open()) {
        ROS_ERROR("Failed to open ref file: %s", ref_file_name.c_str());
        return -1;
    }
    ref_file >> lat0 >> lon0 >> h0;
    ref_file.close();
    wgs84ToECEF(lat0, lon0, h0, x0_ecef, y0_ecef, z0_ecef);

    ROS_INFO("Opening path file: %s", path_file_name.c_str());
    std::ifstream path_file(path_file_name);
    if (!path_file.is_open()) {
        ROS_ERROR("Failed to open path file: %s", path_file_name.c_str());
        return -1;
    }

    double px, py, pz;
    while (path_file >> px >> py >> pz) {
        waypoints.push_back({px, py});
    }
    path_file.close();
    ROS_INFO("Waypoints loaded: %zu", waypoints.size());

    cmd_pub = nh.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd_0", 10);
    ros::Subscriber gps_sub = nh.subscribe("/gps", 50, gpsCallback);
    ros::Subscriber imu_sub = nh.subscribe("/imu", 50, imuCallback);

    ros::spin();
    return 0;
}

