#include <iostream>
#include <math.h>
//#include "comm/Comm.h"

#include <ros/ros.h>
// #include <std_msgs/String.h>
#include <cdf_msgs/Pic_Action.h>
#include <laser_geometry/laser_geometry.h>

#include <pigpiod_if2.h>
/*
#include <nav_msgs/Odometry.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
*/

#define X_SAFE  0.20 // cm, demi-largeur du robot
#define Y_SAFE  0.26 // cm, demi-longueur du robot + distance de sécurité

#define FRONT_LIDAR     6      // != 0, =ID gpio detection avant (pin 31)
#define BACK_LIDAR      12      // != 0, =ID gpio detection arriere (pin 32)
#define NO_OBSTACLE     0

#define RAD_225DEG      3.92699082
#define RAD_180DEG      3.14159265
#define RAD_135DEG      2.35619449
#define RAD_90DEG       1.57079632
#define RAD_45DEG       0.78539816
//#define PI              3.14159265

//typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> sync_policy;

//ros::ServiceClient client;
//comm::Comm srv;

struct point {
    float x;
    float y;
};

// int piGpioId;
float ray, raydiag, dMaxInterest;
bool stateFront, stateBack;

class Obstacle_detector
{
    public:
        Obstacle_detector()
        {
            // piGpioId = pigpio_start(NULL, NULL);
            // set_mode(piGpioId, FRONT_LIDAR, PI_OUTPUT);
            // set_mode(piGpioId, BACK_LIDAR, PI_OUTPUT);
            // gpio_write(piGpioId, FRONT_LIDAR, 0);
            // gpio_write(piGpioId, BACK_LIDAR, 0);

            ray = 0.15;
            raydiag = 0.106; // ray*sqrt(2)/2. En cm ?
            // distance max à laquelle il est possible qu'une détection soit dans la SafeZone
            dMaxInterest = ray + sqrt(X_SAFE * X_SAFE + Y_SAFE * Y_SAFE);
            std::cout << "DmaxInterest is " << dMaxInterest << std::endl;
            stateFront = 0;
	        stateBack = 0;   // no obstacle

            laser_sub_ = n.subscribe("scan", 1, &Obstacle_detector::callback, this);
            avoidOrder_p = n.advertise<sender::Pic_Action>("/action_orders", 1);
        }

    private:
        //message_filters::Synchronizer<sync_policy> sync;
        ros::Publisher avoidOrder_p;
        //message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
        ros::Subscriber laser_sub_;
        ros::NodeHandle n;
        // std_msgs::String msg; // Message to send : TRUE = STOP ROBOT.
        sender::Pic_Action msg;

    /* Description repère par rapport au placement du LIDAR
          [Avant du Robot]
                    y
                 /|\
      135°       |90°         45°
          \_      |
             \_   |
                \_|
    ----------------------------- 0�------> x Côté droit ( angle mort par capot)
               _/ |\
            _/    | \ r (m), t (rad)
          /       |/ \
     -135�        |            -45°
                 -90°
    */

    int checkHazardZone(point p, float th /*radians !*/)
    {
        int isIn = 0;
        point pts[3]; // Points projection horizontale, diagonale et verticale

        if (th > RAD_45DEG)
        { //Looking to the FRONT
            isIn = FRONT_LIDAR;
            pts[0].y = p.y;             // points --+--
            pts[1].y = p.y - raydiag;   // point '\. ou ./'
            pts[2].y = p.y - ray;       // point '|  ou  |.
            if(th > RAD_90DEG) // jusqu'à 270° logiquement
            { //Looking to the FRONT - LEFT
                pts[0].x = p.x + ray;      // point --+
                pts[1].x = p.x + raydiag;  // point '\.
                pts[2].x = p.x;            // point proj verticale
            }
            else
            { //Looking FRONT RIGHT
                pts[0].x = p.x - ray;      // point proj horizontal
                pts[1].x = p.x - raydiag;  // point proj diagonale
                pts[2].x = p.x;            // point proj verticale
            }
        }
        else if (th < -RAD_45DEG)
        { // Looking BACK
            isIn = BACK_LIDAR;
            pts[0].y = p.y;             // point proj horizontal
            pts[1].y = p.y + raydiag;   // point proj diagonale
            pts[2].y = p.y + ray;       // point proj verticale
            if(th < -RAD_90DEG) // jusqu'a� -90° logiquement
            { // Looking BACK - RIGHT
                pts[0].x = p.x + ray;      // point proj horizontal
                pts[1].x = p.x + raydiag;  // point proj diagonale
                pts[2].x = p.x;            // point proj verticale
            }
            else
            { // Looking BACK - RIGHT
                pts[0].x = p.x - ray;      // point proj horizontal
                pts[1].x = p.x - raydiag;  // point proj diagonale
                pts[2].x = p.x;            // point proj verticale
            }
        }

        for (int i = 0; i < 3; i++)
        {
            if (abs(pts[i].x) < X_SAFE && abs(pts[i].y) < Y_SAFE )
            {
                return isIn;
            }
        }
        return NO_OBSTACLE;
    }
//int printed = 0;
    void callback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {
        float angle = scan_in->angle_min;// + theta;
        float increment = scan_in->angle_increment;
//        std::cout << "checking obstacle" << std::endl;

        point p;
        float distance;
        static int jumpNextPoint = 2;
        int detectedFront = NO_OBSTACLE;
	    int detectedBack = NO_OBSTACLE;
        int cptF = 0, cptB = 0;

        for (int i = 0; angle <= scan_in->angle_max; i += jumpNextPoint)
        {
           /* if (!printed)
	    {
		std::cout << "Point : " << angle << " d=" <<  scan_in->ranges[i] << std::endl;
            }*/
            distance = scan_in->ranges[i];
            if (distance > 0.05 && distance < dMaxInterest)
            {
                p.x =  distance * cos(angle); // (attention placement des axes, c.f. schema !)
                p.y = distance * cos(angle); // (attention placement des axes, c.f. schema !)
                int obstacle = checkHazardZone(p, angle);
                if (obstacle)
                {
                    if (obstacle == FRONT_LIDAR)
                    {
                        cptF++;
                        if (cptF >= 6) detectedFront = true;
                    }
                    else if (obstacle == BACK_LIDAR)
                    {
                        cptB++;
			if (cptB >= 6) detectedBack = true;
		    }
                    std::cout << "obstacle at " << distance << " cm, " << angle << " rad." << std::endl;
                }
            }
            angle += scan_in->angle_increment * jumpNextPoint;
        }
//	std::cout << "Angle min : " << scan_in->angle_min << " angle max : " << scan_in->angle_max << std::endl;
//	printed = 1;
        if (!detectedFront && !detectedBack)
        {
            msg.action_destination = "motor";
            msg.action_msg = "LIDAR 0 0\n";
            avoidOrder_p.publish(msg);
        }
        if (detectedFront && !detectedBack)
        {
            msg.action_destination = "motor";
            msg.action_msg = "LIDAR 1 0\n";
            avoidOrder_p.publish(msg);
        }
        if (!detectedFront && detectedBack)
        {
            msg.action_destination = "motor";
            msg.action_msg = "LIDAR 0 1\n";
            avoidOrder_p.publish(msg);
        }
        if (detectedFront && detectedBack)
        {
            msg.action_destination = "motor";
            msg.action_msg = "LIDAR 1 1\n";
            avoidOrder_p.publish(msg);
        }
        // if (detectedFront && !stateFront)
        // {
        //     //srv.request.command = "STOP\n";
        //     //client.call(srv);
        //     //msg.data = true;
        //     //avoidOrder_p.publish(msg);
        //     gpio_write(piGpioId, FRONT_LIDAR , 1);
        //     stateFront = true;
        // }
        // else if (!detectedFront && stateFront)
        // {
        //     //msg.data = NO_OBSTACLE;
        //     //avoidOrder_p.publish(msg);
        //     gpio_write(piGpioId, FRONT_LIDAR, 0);
        //     stateFront = NO_OBSTACLE;
        //     std::cout << "No more obstacles in front." << std::endl;
        // }

        // if (detectedBack && !stateBack)
        // {
        //     //msg.data = true;
        //     //avoidOrder_p.publish(msg);
        //     gpio_write(piGpioId, BACK_LIDAR , 1);
        //     stateBack = true;
        // }
        // else if (!detectedBack && stateBack)
        // {
        //     //srv.request.command = "RESTART\n";
        //     //client.call(srv);
        //     //msg.data = NO_OBSTACLE;
        //     //avoidOrder_p.publish(msg);
        //     gpio_write(piGpioId, BACK_LIDAR, 0);
        //     stateBack = NO_OBSTACLE;
        //     std::cout << "No more obstacles in back." << std::endl;
        // }

    }
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "avoidance_system");
   //client = n.serviceClient<comm::Comm>("pic_pi_comm");

    Obstacle_detector detector;

    ros::spin();

    return 0;
}
