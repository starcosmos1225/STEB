#include <iostream>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
using namespace std;
class TestClass
{
private:
    ros::Subscriber sub;
    ros::NodeHandle n;
public:
    TestClass(ros::NodeHandle n_)
    {
        n = n_;
        sub = n.subscribe<nav_msgs::Odometry>("/h1/odometry/filtered",100,&TestClass::callback,this);
    }
    void callback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        cout<<"get data"<<endl;
        while (true)
        {
            //ROS_INFO("getdata");
        }

    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "testnode");
    ros::NodeHandle n("~");
    TestClass t(n);
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("testnode/twist",50);
    ros::AsyncSpinner spinner(4);
    while(ros::ok())
    {
        geometry_msgs::Twist t;
        pub.publish(t);
        ROS_INFO("publish");
        ros::Duration(1).sleep();
        spinner.start();
    }
    return 0;
}
