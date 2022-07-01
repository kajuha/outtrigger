#include <ros/ros.h>

#include <outtrigger/State.h>

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "outtrigger");
    ros::NodeHandle nh("~");

    ros::Time ts_now, ts_pre;
    
    ts_pre = ts_now = ros::Time::now();

    while (ros::ok()) {
        ts_now = ros::Time::now();

#define DUMMY_CYCLE (1.0)
        if ((ts_now.toSec() - ts_pre.toSec()) > DUMMY_CYCLE) {
            ts_pre = ts_now;

            printf("[%lf] outtrigger package\n", ts_now.toSec());
        }

        ros::spinOnce();
    }

    return 0;
}