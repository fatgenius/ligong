#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <pthread.h>
#include <std_msgs/Float32.h>

#include "../lib/inc/MyDataProto.hpp"
#include "../lib/inc/MySerial.hpp"

// #define SEND_SPEED_TEST  

Serial serial;
TranData serial_data;
size_t DataSize = sizeof(serial_data);
uint8_t *serial_data_buf = (uint8_t*)&serial_data;

double linear_s = 0.0, angle_s = 0.0;
double linear_r = 0.0, angle_r = 0.0;

uint8_t sumData(uint8_t *data, int datasize) { 

    uint8_t sum = 0;
    for (int i = 0; i < datasize - 3 -1; i++) {
        sum += data[i];
    }
    return sum;
}

void revCallback(const geometry_msgs::Twist::ConstPtr &Twist) {

    float lx = Twist->linear.x;
    float ly = Twist->linear.y;
    float lz = Twist->linear.z;

    float wx = Twist->angular.x;
    float wy = Twist->angular.y;
    float wz = Twist->angular.z;

    linear_s = lx;
    angle_s = -wz;
    serial_data.Communicate[0] = linear_s;
    serial_data.Communicate[1] = angle_s;

    serial_data.flag = 0xff;
}


int main(int argc, char **argv) {
    std::string wheel_port;

    ros::init(argc, argv, "wheel_rev");
    ros::NodeHandle n;

    n.param<std::string>("wheel_port", wheel_port, "/dev/ttyUSB0");
    serial.SerialOpen((char *) wheel_port.c_str(), 230400);

    ros::Subscriber sub = n.subscribe("/cmd_vel", 1, revCallback);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 100);
    tf::TransformBroadcaster odom_broadcaster;

    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    ros::Rate loop_rate(100);

    for (int p = 0; p < 15; p++) {
        serial_data.Communicate[p] = 0;
    }
    serial_data.start = 0x55;
    serial_data.start1 = 0xfa;
    serial_data.start2 = 0x56;
    serial_data.start3 = 0xfb;
    serial_data.flag = 0x0;
    serial_data.stop_1 = 0x0d;
    serial_data.stop_2 = 0x0a;

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    nav_msgs::Odometry odom;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    odom.pose.covariance = {1e-3, 0, 0, 0, 0, 0, 
                            0, 1e-3, 0, 0, 0, 0, 
                            0, 0, 1e-3, 0, 0, 0, 
                            0, 0, 0, 1e-3, 0, 0, 
                            0, 0, 0, 0, 1e-3, 0, 
                            0, 0, 0, 0, 0, 1e-3};
    
#ifdef SEND_SPEED_TEST
    serial_data.Communicate[0] = 0.0;
    serial_data.Communicate[1] = 0.07;
    serial_data.flag = 0xff;
#endif

    double dAvge_linear = -1, dAvge_angle = -1;
    while (ros::ok()) {
        ros::spinOnce();

        serial_data.sum = sumData(serial_data_buf, DataSize);   
        serial.SendData(serial_data_buf, DataSize);
        serial.SerialRecv(serial_data_buf, DataSize);
        if (serial_data.sum != sumData(serial_data_buf, DataSize)) {
            printf("skip %x %x \n",serial_data.sum,sumData(serial_data_buf, DataSize));
            continue;
        }

        //后置归零，勿污染接收buf
        serial_data.flag = 0;
        linear_r = serial_data.Communicate[2];
        angle_r = -serial_data.Communicate[3];
        double atRatio_linear, atRatio_angle;
        if(linear_s != 0) {
            atRatio_linear = linear_r / linear_s;
        }
        else {
            atRatio_linear = 1;
        }
        if(angle_s != 0) {
            atRatio_angle = angle_r / angle_s;
        }
        else{
            atRatio_angle = 1;
        }

        if(dAvge_angle == -1){
            dAvge_angle = atRatio_angle;
        }
        else{
            dAvge_angle = (dAvge_angle + atRatio_angle) / 2;
        }
        printf("ls=%.3f lr=%.3f lr/ls=%.3f as=%.3f ar=%.3f ar/as=%.3f avge=%.3f\r",
               linear_s,
               linear_r,
               atRatio_linear,
               angle_s,
               angle_r,
               atRatio_angle,
               dAvge_angle
        );

        current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();
        x += linear_r * cos(th) * dt;
        y += linear_r * sin(th) * dt;
        th += angle_r * dt / 1.95;

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
        odom_trans.header.stamp = current_time; 
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;


        odom.header.stamp = current_time;
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        odom.twist.twist.linear.x = linear_r;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = angle_r;

        odom_broadcaster.sendTransform(odom_trans);
        odom_pub.publish(odom);
        
        last_time = current_time;
        loop_rate.sleep();
    }
    return 0;
}