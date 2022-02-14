/**
 *  MIT License
 *  Copyright (c) 2021 Christian Castaneda <github.com/christianjc>
 * 
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 * 
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 * 
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 * 
*/

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <ros_imu_pkg/Fusion_imu.h>
#include <ros_imu_pkg/Raw_imu.h>
#include <bno055_driver/bno055_driver.h>

int main(int argc, char **argv) {

    /** This initializes the ros client library and gives the node a default
        name: imu_bno055_node **/
    ros::init(argc, argv, "imu_bno055_node");

    /** Making the nodehandle object registers this node with the master node **/
    ros::NodeHandle nh_fusion;
    // ros::NodeHandle nh_raw;

    /* Set topic name and queue size */
    ros::Publisher pub_bno_fusion = nh_fusion.advertise<ros_imu_pkg::Fusion_imu>("imu_fusion", 10, true);
    // ros::Publisher pub_bno_raw = nh_raw.advertise<ros_imu_pkg::Raw_imu>("imu_raw", 10, true);

    /* Declare imu msg type */
    ros_imu_pkg::Fusion_imu imu_fusion;
    // ros_imu_pkg::Raw_imu imu_raw;

    /* Set the loop rate */
    ros::Rate rate(3);

    /** Prints a string to the INFO log **/
    ROS_INFO_STREAM("Starting Node...");

    bno055_imu::BNO055Driver node_fusion("/dev/i2c-1", 0x28, bno055_imu::OPERATION_MODE_IMUPLUS);
    // bno055_imu::BNO055Driver node_raw("/dev/i2c-1", 0x28, bno055_imu::OPERATION_MODE_ACCGYRO);
    node_fusion.init();
    // node_raw.init();

    while(ros::ok()) {
        node_fusion.read_imu_data(imu_fusion);
        // node_raw.read_imu_data_raw(imu_raw);
        pub_bno_fusion.publish(imu_fusion);
        // pub_bno_raw.publish(imu_raw);
        rate.sleep();
    }
}
