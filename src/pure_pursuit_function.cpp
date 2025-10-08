#include "pure_pursuit/pure_pursuit_function.hpp"
#include <morai_msgs/CtrlCmd.h>
#include <cmath>
#include <ros/ros.h>

// 전역 변수 정의
static int last_search_idx = 0;
double lat0, lon0, h0;
double x0_ecef, y0_ecef, z0_ecef;
double ego_x, ego_y, ego_yaw, ego_vel;
bool reached_first_wp = false;
ros::Publisher cmd_pub;
std::vector<std::pair<double, double>> waypoints;

// ====================== 좌표 변환 함수 ======================
void wgs84ToECEF(double lat, double lon, double h,
                 double& x, double& y, double& z) {
    double a = 6378137.0;
    double e2 = 6.69437999014e-3;

    double rad_lat = lat * M_PI / 180.0;
    double rad_lon = lon * M_PI / 180.0;
    double N = a / sqrt(1 - e2 * sin(rad_lat) * sin(rad_lat));

    x = (N + h) * cos(rad_lat) * cos(rad_lon);
    y = (N + h) * cos(rad_lat) * sin(rad_lon);
    z = (N * (1 - e2) + h) * sin(rad_lat);
}

void wgs84ToENU(double lat, double lon, double h,
                double lat_ref, double lon_ref, double h_ref,
                double& x, double& y, double& z) {
    double x_ecef, y_ecef, z_ecef;
    wgs84ToECEF(lat, lon, h, x_ecef, y_ecef, z_ecef);

    double dx = x_ecef - x0_ecef;
    double dy = y_ecef - y0_ecef;
    double dz = z_ecef - z0_ecef;

    double rad_lat = lat_ref * M_PI / 180.0;
    double rad_lon = lon_ref * M_PI / 180.0;

    double t[3][3] = {
        {-sin(rad_lon), cos(rad_lon), 0},
        {-sin(rad_lat) * cos(rad_lon), -sin(rad_lat) * sin(rad_lon), cos(rad_lat)},
        {cos(rad_lat) * cos(rad_lon), cos(rad_lat) * sin(rad_lon), sin(rad_lat)}
    };

    x = t[0][0]*dx + t[0][1]*dy + t[0][2]*dz;
    y = t[1][0]*dx + t[1][1]*dy + t[1][2]*dz;
    z = t[2][0]*dx + t[2][1]*dy + t[2][2]*dz;
}

double quaternionToYaw(double x, double y, double z, double w) {
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return atan2(siny_cosp, cosy_cosp);
}

// ====================== Pure Pursuit ======================
bool approachFirstWaypoint() {
    if (waypoints.empty()) {
        ROS_WARN("No waypoints loaded!");
        return false;
    }
    double dx = waypoints[0].first - ego_x;
    double dy = waypoints[0].second - ego_y;
    double dist = sqrt(dx*dx + dy*dy);
    ROS_INFO("Distance to first waypoint = %.2f", dist);

    if (dist < 7.0) {
        reached_first_wp = true;
        ROS_INFO("Reached first waypoint.");
        return true;
    }
    return false;
}

bool checkGoalReached() {
    if (waypoints.empty()) return false;
    double dx = waypoints.back().first - ego_x;
    double dy = waypoints.back().second - ego_y;
    double dist = sqrt(dx*dx + dy*dy);
    ROS_INFO("Distance to goal = %.2f", dist);

    if (dist < 1.0) {
        morai_msgs::CtrlCmd stop_cmd;
        stop_cmd.longlCmdType = 2;
        stop_cmd.velocity = 0.0;
        stop_cmd.steering = 0.0;
        stop_cmd.accel = 0.0;
        stop_cmd.brake = 1.0;
        cmd_pub.publish(stop_cmd);
        ROS_INFO("Reached goal. Vehicle stopped.");
        ros::shutdown();
        return true;
    }
    return false;
}

std::pair<double, double> findLookAheadPoint(double ld) {
    for (int i = last_search_idx; i < waypoints.size(); ++i) {
        double dx = waypoints[i].first - ego_x;
        double dy = waypoints[i].second - ego_y;
        double dist_from_ego = std::hypot(dx, dy);
        if (dist_from_ego > ld) {
            last_search_idx = i;
            return waypoints[i];
        }
    }
    return waypoints.back();
}

double computeAlpha(const std::pair<double, double>& target) {
    double dx = target.first - ego_x;
    double dy = target.second - ego_y;
    double heading_to_target = atan2(dy, dx);
    return heading_to_target - ego_yaw;
}

double computeSteering(double alpha, double ld) {
    double wheelbase = 2.7;
    return atan2(2.0 * wheelbase * sin(alpha), ld);
}

double adjustLookaheadDistance(double yaw_rate,
                               double ld_straight,
                               double ld_curve) {
    double threshold = 0.1;
    return (fabs(yaw_rate) < threshold) ? ld_straight : ld_curve;
}
