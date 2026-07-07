#include "object_tracker.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "dynamic_obstacle_tracker_node");

    DynamicObstacleTrackerNode tracker;
    tracker.run();

    return 0;
}
