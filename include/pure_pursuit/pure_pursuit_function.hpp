#pragma once
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <morai_msgs/LongitudinalCmd.h>
#include <vector>

// ENU 기준점
extern double lat0, lon0, h0;
extern double x0_ecef, y0_ecef, z0_ecef;

// 차량 상태
extern double ego_x, ego_y, ego_yaw, ego_vel;
extern bool reached_first_wp;

// 제어 명령 발행
extern ros::Publisher cmd_pub;

// 경로
extern std::vector<std::pair<double, double>> waypoints;

// 변환 함수
void wgs84ToECEF(double lat, double lon, double h,
                 double& x, double& y, double& z);
void wgs84ToENU(double lat, double lon, double h,
                double lat_ref, double lon_ref, double h_ref,
                double& x, double& y, double& z);
double quaternionToYaw(double x, double y, double z, double w);

// Pure Pursuit 핵심 함수
bool approachFirstWaypoint();
bool checkGoalReached();
std::pair<double, double> findLookAheadPoint(double ld); // <- double 타입의 재료를 받는다고 명시!
double computeAlpha(const std::pair<double, double>& target);
double computeSteering(double alpha, double ld); // Ld 값을 추가로 받도록 수정


// Ld 조정 함수 (직선/곡선)
double adjustLookaheadDistance(double yaw_rate,
                               double ld_straight,
                               double ld_curve);

