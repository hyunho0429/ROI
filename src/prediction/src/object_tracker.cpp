#include "object_tracker.hpp"
#include <iostream>
#include <stdexcept>

// DynamicObstacleTracker Class Implementation
DynamicObstacleTracker::DynamicObstacleTracker(double dt, double T)
    : dt(dt),
      T(T),
      mat_trans(Eigen::MatrixXd(2, 2)),
      mu(Eigen::RowVectorXd(2)),
      Q_list({Eigen::VectorXd(5), Eigen::VectorXd(6)}),
      R_list({Eigen::VectorXd(4), Eigen::VectorXd(4)}) {

        auto ca_model = std::make_shared<CA>(dt);
        auto ctra_model = std::make_shared<CTRA>(dt);
        this->models = {ca_model, ctra_model};

        auto ekf_ca = std::make_shared<ExtendedKalmanFilter>(5, 4, this->models[0]);
        auto ekf_ctra = std::make_shared<ExtendedKalmanFilter>(6, 4, this->models[1]);
        this->filters = {ekf_ca, ekf_ctra};

        this->mat_trans << 0.85, 0.15,
                           0.15, 0.85;

        this->mu << 0.8, 0.2;

        this->Q_list[0] << 0.1, 0.1, 0.1, 0.1, 0.1;
        this->Q_list[1] << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;

        this->R_list[0] << 0.05, 0.05, 0.05, 0.05;
        this->R_list[1] << 0.05, 0.05, 0.05, 0.05;

        for (int i = 0; i < this->filters.size(); ++i) {
            this->filters[i]->Q = this->Q_list[i].asDiagonal();
            this->filters[i]->R = this->R_list[i].asDiagonal();
        }

        this->IMM = std::make_unique<IMMFilter>(this->filters, this->mu, this->mat_trans);

        this->X.clear();
        this->MM.clear();
        this->MM.push_back(this->mu);
}

void DynamicObstacleTracker::initialize(const Eigen::RowVectorXd& data) {
    // data = [x, y, h, v]
    // x = [x, y, v, a, theta], [x, y, v, a, theta, theta_rate]

    double x = data(0);
    double y = data(1);
    double h = data(2);
    double v = data(3);

    Eigen::RowVectorXd x1(5), x2(6);
    x1 << x, y, v, 0, h;
    x2 << x, y, v, 0, h, 0;

    this->filters[0]->x = x1;
    this->filters[1]->x = x2;

    this->X.push_back(x2);
}

void DynamicObstacleTracker::update(const Eigen::RowVectorXd& data) {
    // data = [x, y, h, v]
    // z = [x, y, v, theta]

    double x = data(0);
    double y = data(1);
    double h = data(2);
    double v = data(3);

    Eigen::RowVectorXd z(4);
    z << x, y, v, h;

    this->IMM->prediction();
    this->IMM->merging(z);

    while (this->MM.size() > 10) {
        this->MM.pop_front();
    }
    while (this->X.size() > 10) {
        this->X.pop_front();
    }

    this->MM.push_back(this->IMM->mu);
    this->X.push_back(this->IMM->x);
}

Eigen::MatrixXd DynamicObstacleTracker::pred() {
    Eigen::MatrixXd traj = this->IMM->pred(this->T);
    return traj;
}


// TrackedObject Class Implementation
TrackedObject::TrackedObject(int obj_id, std::shared_ptr<DynamicObstacleTracker> tracker, ros::Time last_update_time)
    : obj_id(obj_id), tracker(tracker), last_update_time(last_update_time) {}


// MultiDynamicObstacleTracker Class Implementation
MultiDynamicObstacleTracker::MultiDynamicObstacleTracker(double dt, double T, double timeout)
    : dt(dt), T(T), timeout(timeout) {}

void MultiDynamicObstacleTracker::initialize(int obj_id, const Eigen::RowVectorXd& data) {
    this->addTracker(obj_id);
    int idx = this->obj_id_map[obj_id];
    this->objects[idx]->tracker->initialize(data);
    this->objects[idx]->last_update_time = ros::Time::now();
}

void MultiDynamicObstacleTracker::addTracker(int obj_id) {
    if (this->obj_id_map.count(obj_id) == 0) {
        auto tracker = std::make_shared<DynamicObstacleTracker>(this->dt, this->T);
        ros::Time last_update_time = ros::Time::now();
        auto obj = std::make_shared<TrackedObject>(obj_id, tracker, last_update_time);
        this->objects.push_back(obj);
        this->obj_id_map[obj_id] = this->objects.size() - 1;
    }
}

void MultiDynamicObstacleTracker::update(int obj_id, const Eigen::RowVectorXd& data) {
    if (this->obj_id_map.count(obj_id) != 0) {
        int idx = this->obj_id_map[obj_id];
        this->objects[idx]->tracker->update(data);
        this->objects[idx]->last_update_time = ros::Time::now();
    } else {
        this->initialize(obj_id, data);
    }
}

void MultiDynamicObstacleTracker::clean() {
    ros::Time current_time = ros::Time::now();
    std::vector<int> to_delete;

    for (const auto& it : this->obj_id_map){
        int obj_id = it.first;
        int idx = it.second;

        if (current_time - this->objects[idx]->last_update_time > ros::Duration(this->timeout)) {
            to_delete.push_back(idx);
        }
    }

    for (int idx : to_delete) {
        this->deleteObject(this->objects[idx]->obj_id);
    }
}

void MultiDynamicObstacleTracker::deleteObject(int obj_id) {
    if (this->obj_id_map.count(obj_id) != 0) {
        int idx = this->obj_id_map[obj_id];
        this->deleted_ids.insert(obj_id);

        if (idx != this->objects.size() - 1) {
            this->obj_id_map[this->objects.back()->obj_id] = idx;
            std::swap(this->objects[idx], this->objects.back());
        }

        this->objects.pop_back();
        this->obj_id_map.erase(obj_id);
    }
}

std::vector<int> MultiDynamicObstacleTracker::getDeletedIds() {
    std::vector<int> deleted_ids_vec(this->deleted_ids.begin(), this->deleted_ids.end());
    this->deleted_ids.clear();
    return deleted_ids_vec;
}

std::unordered_map<int, Eigen::MatrixXd> MultiDynamicObstacleTracker::pred() {
    std::unordered_map<int, Eigen::MatrixXd> trajs;
    for (const auto& obj : this->objects) {
        trajs[obj->obj_id] = obj->tracker->pred();
    }

    if (trajs.empty()) {
        return std::unordered_map<int, Eigen::MatrixXd>();
    } else {
        return trajs;
    }
}


// DynamicObstacleTrackerNode Class Implementation
DynamicObstacleTrackerNode::DynamicObstacleTrackerNode()
    : nh(), is_odom(false), is_object(false), rate(30) {
    // ROS 토픽을 구독
    this->object_info_sub = nh.subscribe("/Object_topic", 10, &DynamicObstacleTrackerNode::objectInfoCallback, this);
    this->odom_sub = nh.subscribe("odom", 10, &DynamicObstacleTrackerNode::odomCallback, this);

    // 퍼블리셔 초기화
    this->object_pose_pub = nh.advertise<prediction::TrackedObjectPoseList>("/Object_topic/tracked_object_pose_topic", 10);
    this->object_path_pub = nh.advertise<prediction::PredictedObjectPathList>("/Object_topic/tracked_object_path_topic", 10);
    this->deleted_id_pub = nh.advertise<std_msgs::Int32>("/Object_topic/deleted_object_id", 10);

    // MultiDynamicObstacleTracker 객체를 초기화합니다.
    this->multi_tracker = MultiDynamicObstacleTracker(0.05, 0.8, 0.1);
}

void DynamicObstacleTrackerNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    this->is_odom = true;
    this->odom_data = *msg;
}

void DynamicObstacleTrackerNode::objectInfoCallback(const morai_msgs::ObjectStatusList::ConstPtr& msg) {
    this->is_object = true;
    this->object_data = *msg;
}

Eigen::RowVectorXd DynamicObstacleTrackerNode::dataPreprocessing(const morai_msgs::ObjectStatus& obstacle) {
    // 평면 속도를 계산합니다.
    double v = std::sqrt(std::pow(obstacle.velocity.x, 2) + std::pow(obstacle.velocity.y, 2));
    Eigen::RowVectorXd data(4);
    data << obstacle.position.x, obstacle.position.y, std::fmod(obstacle.heading, 360.0) * M_PI / 180.0, v;
    return data;
}

void DynamicObstacleTrackerNode::publishObjectPose() {
    prediction::TrackedObjectPoseList pose_list;
    pose_list.header.stamp = ros::Time::now();

    for (const auto& obj : this->multi_tracker.objects) {
        const auto& x = obj->tracker->X.back();
        prediction::TrackedObjectPose pose;
        prediction::TrackedPoint point;
        pose.unique_id = obj->obj_id;
        point.x = x(0);
        point.y = x(1);
        point.v = x(2);
        point.a = x(3);
        point.theta = x(4);
        point.theta_rate = x(5);
        pose.pose = point;
        pose_list.pose_list.push_back(pose);
    }

    this->object_pose_pub.publish(pose_list);
}

void DynamicObstacleTrackerNode::publishObjectPath() {
    std::unordered_map<int, Eigen::MatrixXd> trajs = this->multi_tracker.pred();

    prediction::PredictedObjectPathList path_list;
    path_list.header.stamp = ros::Time::now();

    if(!trajs.empty()){
        for (auto it = trajs.begin(); it != trajs.end(); ++it) {
            int obj_id = it->first;
            const auto& traj = it->second;
            prediction::PredictedObjectPath path;
            path.unique_id = obj_id;

            for (int i = 0; i < traj.rows(); ++i) {
                prediction::TrackedPoint point;
                point.x = traj(i, 0);
                point.y = traj(i, 1);
                point.v = traj(i, 2);
                point.a = traj(i, 3);
                point.theta = traj(i, 4);
                point.theta_rate = traj(i, 5);

                path.path.push_back(point);
            }
            path_list.path_list.push_back(path);
        }
    }

    this->object_path_pub.publish(path_list);
}

void DynamicObstacleTrackerNode::publishDeletedIds() {
    std::vector<int> deleted_ids = this->multi_tracker.getDeletedIds();
    std_msgs::Int32 msg;
    for (int deleted_id : deleted_ids) {
        msg.data = deleted_id;
        this->deleted_id_pub.publish(msg);
    }
}

void DynamicObstacleTrackerNode::run() {
    while (ros::ok()) {
        ros::spinOnce(); // 콜백 함수를 실행합니다.

        if (is_object && is_odom) {
            ros::Time current_time = ros::Time::now();

            // 현재 차량의 위치를 가져옵니다.
            double x = this->odom_data.pose.pose.position.x;
            double y = this->odom_data.pose.pose.position.y;

            // 객체 정보를 처리합니다.
            for (const auto& obstacle : this->object_data.npc_list) {
                int obj_id = obstacle.unique_id;

                double obstacle_x = obstacle.position.x;
                double obstacle_y = obstacle.position.y;
                double distance = std::sqrt(std::pow(x - obstacle_x, 2) + std::pow(y - obstacle_y, 2));

                if (distance > 30) {
                    continue;
                }

                Eigen::MatrixXd data = this->dataPreprocessing(obstacle);

                this->multi_tracker.update(obj_id, data);
            }

            // 타임아웃된 객체를 삭제합니다.
            this->multi_tracker.clean();
            this->publishDeletedIds();

            // 객체 위치 및 경로를 발행합니다.
            this->publishObjectPose();
            this->publishObjectPath();
        }

        this->rate.sleep();   // 루프 주기를 유지합니다.
    }
}


#ifndef OBJECT_TRACKER_LIBRARY_ONLY
int main(int argc, char **argv) {
    ros::init(argc, argv, "dynamic_obstacle_tracker_node");

    DynamicObstacleTrackerNode tracker;
    tracker.run();

    return 0;
}
#endif
