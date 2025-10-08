#include "pure_pursuit_function.hpp"
#include <fstream>

// === ROS 파라미터 (YAML에서 불러옴) ===
double ld_straight;   // 직선 구간 Ld
double ld_curve;     // 곡선 구간 Ld
double target_velocity;      // 목표 속도
double goal_tolerance;       // 목표점 도착 판정 반경
std::string path_file_name;  // 경로 파일(Path.txt)
std::string ref_file_name;   // 기준점 파일(ref.txt)

// === GPS 콜백: /gps 토픽을 받아 WGS84 → ENU로 변환해 ego_x, ego_y 업데이트 ===
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    double x, y, z;
    // WGS84 좌표계 → ENU 좌표계 변환
    wgs84ToENU(msg->latitude, msg->longitude, msg->altitude,
               lat0, lon0, h0, x, y, z);
    ego_x = x;
    ego_y = y;
}

// === IMU 콜백: /imu 토픽을 받아 yaw 추출 후 Pure Pursuit 알고리즘 수행 ===
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    // 쿼터니안 → yaw(heading)
    ego_yaw = quaternionToYaw(msg->orientation.x,
                              msg->orientation.y,
                              msg->orientation.z,
                              msg->orientation.w);

    // 1. 첫 웨이포인트로 접근 (스폰된 위치와 경로 시작점이 다를 수 있음)
    if (!reached_first_wp) {
        if (!approachFirstWaypoint()) return; 
    }

    // 2. 마지막 웨이포인트 도착 여부 확인 → 도착 시 차량 정지 후 종료
    if (checkGoalReached()) return;

    // 3. 직선/곡선 판단 후 Lookahead Distance(Ld) 결정
    double yaw_rate = msg->angular_velocity.z;
    double Ld = adjustLookaheadDistance(yaw_rate, ld_straight, ld_curve);

    // 4. Ld 반경 안에서 목표 웨이포인트 찾기
    auto target = findLookAheadPoint(ld);

    // 5. 목표점과 현재 heading 차이 → α 계산
    double alpha = computeAlpha(target);

    // 6. 조향각 계산
    double steering_angle = computeSteering(alpha,ld);

    // 7. 제어 명령 발행 (MORAI LongitudinalCmd 사용)
    morai_msgs::LongitudinalCmd cmd;
    cmd.longlCmdType = 2;                // 2: 속도 제어 모드
    cmd.velocity = target_velocity;      // 목표 속도 유지
    cmd.steering = steering_angle;       // 계산된 조향각
    // cmd.accel = (target_velocity - ego_vel) * 0.0; // 속도 보정용 가속도
    // cmd.brake = (ego_vel > target_velocity) ? 0.0 : 0.0; // 과속 시 감속
    cmd_pub.publish(cmd);
}

// === 메인 함수 ===
int main(int argc, char** argv) {
    ros::init(argc, argv, "pure_pursuit_node");
    ros::NodeHandle nh;

    // 1. YAML에서 파라미터 불러오기
    nh.param("ld_straight", ld_straight, 6.0);
    nh.param("ld_curve", ld_curve, 3.0);
    nh.param("target_velocity", target_velocity, 10.0);
    nh.param("goal_tolerance", goal_tolerance, 1.0);
    nh.getParam("path_file_name", path_file_name);
    nh.getParam("ref_file_name", ref_file_name);

    // 2. 기준점(ref.txt) 로드 (ENU 변환 기준점)
    std::ifstream ref_file(ref_file_name);
    ref_file >> lat0 >> lon0 >> h0;
    ref_file.close();
    wgs84ToECEF(lat0, lon0, h0, x0_ecef, y0_ecef, z0_ecef);

    // 3. 경로(Path.txt) 로드 (ENU 좌표의 웨이포인트들)
    std::ifstream path_file(path_file_name);
    double px, py, pz;
    while (path_file >> px >> py >> pz) {
        waypoints.push_back({px, py}); // ENU 좌표만 사용
    }
    path_file.close();

    // 4. ROS pub/sub 설정
    cmd_pub = nh.advertise<morai_msgs::LongitudinalCmd>("/Longitudinal_cmd", 10);
    ros::Subscriber gps_sub = nh.subscribe("/gps", 50, gpsCallback);
    ros::Subscriber imu_sub = nh.subscribe("/imu", 50, imuCallback);

    // 5. ROS 스핀 → 콜백 기반 실행
    ros::spin();
    return 0;
}


