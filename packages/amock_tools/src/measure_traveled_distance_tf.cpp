#include <ros/ros.h>


#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>

#include <rosgraph_msgs/Clock.h>



int main(int argc, char** argv)
{
    ros::init(argc, argv, "measure_traveled_distance_tf");

    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    std::string reference_frame;
    std::string robot_frame;

    nh_p.param<std::string>("reference_frame", reference_frame, "map");
    nh_p.param<std::string>("robot_frame", robot_frame, "base_footprint");

   
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    ros::Rate r(100);

    geometry_msgs::Vector3 p_old;
    bool first_point = true;
    double traveled_distance = 0.0;
    double avg_speed = 0.0;
    size_t n_measurements = 0;

    ros::Time last_stamp = ros::Time(0);

    double min_stamp_dist = 0.001;

    while(ros::ok())
    {
        try
        {
            geometry_msgs::TransformStamped T;
            T = tf_buffer.lookupTransform(
                reference_frame, 
                robot_frame, 
                ros::Time(0));

            if(first_point)
            {
                p_old = T.transform.translation;
                first_point = false;
                last_stamp = T.header.stamp;
                n_measurements++;
            } else {

                ros::Duration dt = T.header.stamp - last_stamp;

                if(dt.toSec() > min_stamp_dist)
                {
                    geometry_msgs::Vector3 dp;
                    dp.x = T.transform.translation.x - p_old.x;
                    dp.y = T.transform.translation.y - p_old.y;
                    dp.z = T.transform.translation.z - p_old.z;
                    double ddist = sqrt(dp.x * dp.x + dp.y * dp.y + dp.z * dp.z);
                    double current_speed = ddist / dt.toSec();
                    
                    traveled_distance += ddist;
                    
                    double nd = static_cast<double>(n_measurements);
                    avg_speed = nd/(nd+1.0) * avg_speed + 1.0/(nd+1.0) * current_speed;
                    
                    n_measurements++;
                    last_stamp = T.header.stamp;
                    p_old = T.transform.translation;

                    // print
                    std::cout << "-------" << std::endl;
                    std::cout << "- current speed: " << current_speed << " m/s" << std::endl;
                    std::cout << "- travelled distance: " << traveled_distance << " m" << std::endl;
                    std::cout << "- avg speed: " << avg_speed << " m/s" << std::endl;
                    std::cout << "- number of measurements: " << n_measurements << std::endl;
                }
            }

        } catch(tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }

       
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}