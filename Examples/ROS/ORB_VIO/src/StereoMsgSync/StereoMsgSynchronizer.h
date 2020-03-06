#ifndef STEREOMSGSYNCHRONIZER_H
#define STEREOMSGSYNCHRONIZER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <queue>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <mutex>

using namespace std;

namespace ORBVIO
{
class StereoMsgSynchronizer
{
public:
    enum Status{
        NOTINIT = 0,
        INIT,
        NORMAL
    };

    StereoMsgSynchronizer(const double& imagedelay = 0.);
    ~StereoMsgSynchronizer();

    // add messages in callbacks
    void addImageMsg(const sensor_msgs::ImageConstPtr &imgmsg, std::queue<sensor_msgs::ImageConstPtr>* queue);
    void addImuMsg(const sensor_msgs::ImuConstPtr &imumsg);

    // loop in main function to handle all messages
    bool getRecentMsgs(sensor_msgs::ImageConstPtr &leftimgmsg, sensor_msgs::ImageConstPtr &rightimgmsg, std::vector<sensor_msgs::ImuConstPtr> &vimumsgs);

    void clearMsgs(void);

    // for message callback if needed
    void leftImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void rightImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void imuCallback(const sensor_msgs::ImuConstPtr& msg);

    //
    inline Status getStatus(void) {return _status;}

    double getImageDelaySec(void) const {return _imageMsgDelaySec;}

private:
    double _imageMsgDelaySec;  // image message delay to imu message, in seconds
    std::mutex _mutexImageQueue;
    std::queue<sensor_msgs::ImageConstPtr> _leftImageMsgQueue;
    std::queue<sensor_msgs::ImageConstPtr> _rightImageMsgQueue;
    std::mutex _mutexIMUQueue;
    std::queue<sensor_msgs::ImuConstPtr> _imuMsgQueue;
    ros::Time _imuMsgTimeStart;
    Status _status;
    int _dataUnsyncCnt;
};

}

#endif // STEREOMSGSYNCHRONIZER_H
