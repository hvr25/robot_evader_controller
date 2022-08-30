#include <ros/ros.h>

// Publish to a topic with this message type
#include <ackermann_msgs/AckermannDriveStamped.h>
// AckermannDriveStamped messages include this message type
#include <ackermann_msgs/AckermannDrive.h>

// Subscribe to a topic with this message type
#include <nav_msgs/Odometry.h>

#include <sensor_msgs/LaserScan.h>

// for printing
#include <iostream>

// for RAND_MAX
#include <cstdlib>

class RandomWalker {
private:
    // A ROS node
    ros::NodeHandle n;

    // car parameters
    double max_speed, max_steering_angle;

    // Listen for odom messages
    ros::Subscriber odom_sub;

    ros::Subscriber laser_sub;


    // Publish drive data
    ros::Publisher drive_pub;

    // previous desired steering angle
    double prev_angle=0.0;


public:
    RandomWalker() {
        // Initialize the node handle
        n = ros::NodeHandle("~");

        // get topic names
        std::string drive_topic, odom_topic, scan_topic;
        n.getParam("rand_drive_topic", drive_topic);
        n.getParam("odom_topic", odom_topic);
	n.getParam("scan_topic", scan_topic);

        // get car parameters
        n.getParam("max_speed", max_speed);
        n.getParam("max_steering_angle", max_steering_angle);

        // Make a publisher for drive messages
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);

        // Start a subscriber to listen to odom messages
        //odom_sub = n.subscribe(odom_topic, 1, &RandomWalker::odom_callback, this);
	laser_sub = n.subscribe(scan_topic, 1, &RandomWalker::laser_callback, this);

    }


    void laser_callback(const sensor_msgs::LaserScan & msg)  {
        // publishing is done in odom callback just so it's at the same rate as the sim

        // initialize message to be published
        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;

        /// SPEED CALCULATION:
        // set constant speed to be half of max speed
        drive_msg.speed =  2.0;

	//
        int precision = 40;
        int slot = msg.ranges.size()/precision;
        float range_new[precision];
        int cnt = 0;
        for (size_t i = 0; i < msg.ranges.size(); i = i + slot) {
            float sum = 0.0;
            
            for (size_t j = i; j < i+slot; j++) {
                sum = sum + msg.ranges[j];
            }
            range_new[cnt] = sum/slot;
            cnt++;
        }

        float range_front[precision/2];
        for (int i = 0; i < precision/2; i++) {
            range_front[i] = range_new[i + precision/4];
        }

	/*
	   ROS_INFO("range_0: [%f]", msg.ranges[0]);
        ROS_INFO("range_269: [%f]", msg.ranges[269]);
        ROS_INFO("range_539: [%f]", msg.ranges[539]);
        ROS_INFO("range_809: [%f]", msg.ranges[809]);

        
        for (int i= 0; i < precision; i++) {
            ROS_INFO("ranges([%d]): ",i);
             ROS_INFO( "%f", range_new[i]);
        }
        
        for (int i= 0; i < precision/2; i++) {
            ROS_INFO("range_front([%d]): ",i);
             ROS_INFO( "%f", range_front[i]);
        }    
*/
        // set angle (add random change to previous angle)
       // drive_msg.steering_angle = std::min(std::max(prev_angle + rand_ang, -max_steering_angle), max_steering_angle);

        // reset previous desired angle
        prev_angle = drive_msg.steering_angle;

        int dir_num = std::distance(range_front,std::max_element(range_front, range_front + precision/2));
        int mid_num = (sizeof(range_front)/sizeof(range_front[0]))/2;
        int ang_factor = dir_num - mid_num;    

        if (msg.ranges[539] < 1.5) {   
            drive_msg.steering_angle = ang_factor * 0.2;                 
        } 

        if (range_front[precision/4] < 0.7 or range_front[precision/4 - 1] < 0.7) {
            drive_msg.steering_angle = ang_factor * 0.2;
           
        }

        if (range_front[precision/2-1] < 0.5 or range_front[0] < 0.5) {
            drive_msg.steering_angle = ang_factor * 0.1 ;
           
        }

        if (range_front[precision/2-1] < 1.0 and range_front[0] < 1.0 and range_front[precision/4] < 0.7) {
            drive_msg.steering_angle = ang_factor ;
           drive_msg.speed =  0.0;
        }


        // set drive message in drive stamped message
        drive_st_msg.drive = drive_msg;

        // publish AckermannDriveStamped message to drive topic
        drive_pub.publish(drive_st_msg);
	/*

        /// STEERING ANGLE CALCULATION
        // random number between 0 and 1
        double random = ((double) rand() / RAND_MAX);
        // good range to cause lots of turning
        double range = max_steering_angle / 2.0;
        // compute random amount to change desired angle by (between -range and range)
        double rand_ang = range * random - range / 2.0;

        // sometimes change sign so it turns more (basically add bias to continue turning in current direction)
        random = ((double) rand() / RAND_MAX);
        if ((random > .8) && (prev_angle != 0)) {
            double sign_rand = rand_ang / std::abs(rand_ang);
            double sign_prev = prev_angle / std::abs(prev_angle);
            rand_ang *= sign_rand * sign_prev;
        }

        // set angle (add random change to previous angle)
        drive_msg.steering_angle = std::min(std::max(prev_angle + rand_ang, -max_steering_angle), max_steering_angle);

	

        // reset previous desired angle
        prev_angle = drive_msg.steering_angle;

        // set drive message in drive stamped message
        drive_st_msg.drive = drive_msg;

        // publish AckermannDriveStamped message to drive topic
        drive_pub.publish(drive_st_msg);
	*/

    }

}; // end of class definition


int main(int argc, char ** argv) {
    ros::init(argc, argv, "random_walker");
    RandomWalker rw;
    ros::spin();
    return 0;
}
