#include "pure_pursuit_function.hpp"
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
    if (waypoints.empty()) return false;
    double dx = waypoints[0].first - ego_x;
    double dy = waypoints[0].second - ego_y;
    double dist = sqrt(dx*dx + dy*dy);
    if (dist < 1.0) {
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
    if (dist < 1.0) {
        morai_msgs::LongitudinalCmd stop_cmd;
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

    // for 반복문을 사용해 경로점(waypoints)을 확인할 겁니다.
    // 가장 중요한 부분! -> int i = 0; 이 아니라, int i = last_search_idx; 부터 탐색을 시작합니다.
    // 즉, 이전에 찾아냈던 목표점 위치(책갈피가 꽂힌 곳)부터 탐색을 이어 나갑니다.
    for (int i = last_search_idx; i < waypoints.size(); ++i) {
        
        // i번째 경로점의 x좌표와 내 차의 x좌표(ego_x) 사이의 거리를 계산합니다.
        double dx = waypoints[i].first - ego_x;
        // i번째 경로점의 y좌표와 내 차의 y좌표(ego_y) 사이의 거리를 계산합니다.
        double dy = waypoints[i].second - ego_y;
        
        // 피타고라스 정리를 이용해 내 차와 i번째 경로점 사이의 실제 직선 거리를 계산합니다.
        double dist_from_ego = std::hypot(dx, dy);

        // 만약 계산된 거리가, 외부에서 전달받은 목표 시야거리(ld)보다
        // 마침내 더 멀어졌다면, (이게 우리가 찾던 바로 그 점입니다!)
        if (dist_from_ego > ld) {
            
            // "목표를 찾았다!" 다음 탐색을 위해, 지금 찾은 위치(i)를 '책갈피' 변수에 저장(갱신)합니다.
            last_search_idx = i;
            
            // 지금 찾은 i번째 경로점의 좌표를 반환하고, 함수를 즉시 종료합니다.
            return waypoints[i];
        }
    } // 만약 if 조건이 맞지 않으면, 다음 경로점(i+1)으로 넘어가서 위 과정을 반복합니다.

    // 위 반복문이 경로 끝까지 다 돌았는데도 목표점을 못 찾았다면,
    // (보통 차량이 경로의 거의 끝에 도달했을 때 이런 상황이 발생합니다)
    // 안전을 위해 경로의 가장 마지막 점을 목표점으로 반환합니다.
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
    // 고정된 6.0 대신, 외부에서 전달받은 ld 값을 사용합니다.
    return atan2(2.0 * wheelbase * sin(alpha), ld); //atan2(x, y)=arctan(x/y)
}

// ====================== Ld 조정 함수 ======================
double adjustLookaheadDistance(double yaw_rate,
                               double ld_straight,
                               double ld_curve) {
    double threshold = 0.1; // yaw_rate 기준
    if (fabs(yaw_rate) < threshold) {
        return ld_straight; // 직선
    } else {
        return ld_curve;    // 곡선
    }
}