#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <rmagine/math/types.h>

#include <random>

namespace rm = rmagine;

// params
std::string collector_frame = "";
int max_points = 0;
double max_time_diff = 0;
double range_min = 0.2;
double range_max = 50.0;
int insertions_per_scan = 50;

// global variables
// std::vector<rm::Vector> points;
bool first_pcl = true;
sensor_msgs::PointCloud2 pcl_collection;
std::shared_ptr<tf2_ros::Buffer> tf_buffer;
std::shared_ptr<tf2_ros::TransformListener> tf_listener;

ros::Publisher pub_pcl;


void pclCB(const sensor_msgs::PointCloud2::ConstPtr& pcl)
{
    // Transform from sensor to collector frame
    rm::Transform Tsc;
    rm::Transform_<double> Tscd;

    double dt_pcl_transform;
    
    try
    {
        geometry_msgs::TransformStamped Tros;
        Tros = tf_buffer->lookupTransform(
            collector_frame, 
            pcl->header.frame_id, 
            pcl->header.stamp);

        dt_pcl_transform = (pcl->header.stamp - Tros.header.stamp).toSec();

        Tscd.R = {Tros.transform.rotation.x, Tros.transform.rotation.y, Tros.transform.rotation.z, Tros.transform.rotation.w};
        Tscd.t = {Tros.transform.translation.x, Tros.transform.translation.y, Tros.transform.translation.z};
        Tscd.stamp = dt_pcl_transform;

    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        return;
    }

    ROS_INFO_STREAM("PCL COLLECT! time: pcl - transform = " << dt_pcl_transform << "s");

    Tsc = Tscd.cast<float>();

    sensor_msgs::PointField field_x;
    sensor_msgs::PointField field_y;
    sensor_msgs::PointField field_z;

    for(size_t i=0; i<pcl->fields.size(); i++)
    {
        if(pcl->fields[i].name == "x")
        {
            field_x = pcl->fields[i];
        }
        if(pcl->fields[i].name == "y")
        {
            field_y = pcl->fields[i];
        }
        if(pcl->fields[i].name == "z")
        {
            field_z = pcl->fields[i];
        }
    }

    if(field_x.datatype != sensor_msgs::PointField::FLOAT32 
        && field_x.datatype != sensor_msgs::PointField::FLOAT64)
    {
        throw std::runtime_error("Field X has unknown DataType. Check Topic of pcl");
    }



    pcl_collection.header.stamp = pcl->header.stamp;
    
    if(first_pcl)
    {
        pcl_collection.height = 1;
        pcl_collection.point_step = pcl->point_step;
        pcl_collection.fields = pcl->fields;
        pcl_collection.is_dense = true;

        first_pcl = false;
    }

    size_t n_source = pcl->width * pcl->height;

    // TODO how to set this. param?
    size_t n_transfer = insertions_per_scan;

    std::random_device rd;  // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()

    std::uniform_int_distribution<size_t> source_dist(0, n_source-1);
    std::uniform_real_distribution<float> rand_zero_to_one(0.0, 1.0);


    for(size_t i = 0; i<n_transfer; i++)
    {
        // copy from
        size_t source_id = source_dist(gen);
        const uint8_t* src_data_ptr = &pcl->data[source_id * pcl->point_step];

        // check if source is ok
        if(field_x.datatype == sensor_msgs::PointField::FLOAT32)
        {
            // Float
            const float* x = reinterpret_cast<const float*>(src_data_ptr + field_x.offset);
            const float* y = reinterpret_cast<const float*>(src_data_ptr + field_y.offset);
            const float* z = reinterpret_cast<const float*>(src_data_ptr + field_z.offset);

            // read and check
            rm::Vector3f vec{*x, *y, *z};

            // assuming input sensor frame is the frame where the cloud was initially measured.
            float dist = vec.l2norm();

            if(dist < range_min || dist > range_max)
            {
                continue;
            }
        } else if(field_x.datatype == sensor_msgs::PointField::FLOAT64) {
            // Double
            const double* x = reinterpret_cast<const double*>(src_data_ptr + field_x.offset);
            const double* y = reinterpret_cast<const double*>(src_data_ptr + field_y.offset);
            const double* z = reinterpret_cast<const double*>(src_data_ptr + field_z.offset);

            // read and check
            rm::Vector3d vec{*x, *y, *z};

            // assuming input sensor frame is the frame where the cloud was initially measured.
            float dist = vec.l2norm();

            if(dist < range_min || dist > range_max)
            {
                continue;
            }
        }


        size_t n_target = pcl_collection.width * pcl_collection.height;
        float p_insert = static_cast<float>(n_target) / static_cast<float>(max_points);


        // if collected pointcloud is extended
        // - target_id points to new element at back
        // else
        // - target id points to random element
        size_t target_id;

        // decide to extend
        if(p_insert < rand_zero_to_one(gen))
        {
            // extend collection and put to end
            for(size_t i=0; i<pcl->point_step; i++)
            {
                pcl_collection.data.push_back(0);
            }

            // pcl_collection.data.resize(pcl_collection.data.size() + pcl->point_step);
            pcl_collection.width += 1;
            pcl_collection.row_step = pcl_collection.point_step * pcl_collection.width;

            target_id = pcl_collection.width * pcl_collection.height - 1;
        } else {
            // insert into existing collection
            std::uniform_int_distribution<size_t> target_dist(0, pcl_collection.width * pcl_collection.height - 1);
            target_id = target_dist(gen);
        }

        // copy data
        uint8_t* tgt_data_ptr = &pcl_collection.data[target_id * pcl->point_step];
        std::memcpy(tgt_data_ptr, src_data_ptr, pcl->point_step);

        // transform data points

        // copy point
        if(field_x.datatype == sensor_msgs::PointField::FLOAT32)
        {
            // Float
            float* x = reinterpret_cast<float*>(tgt_data_ptr + field_x.offset);
            float* y = reinterpret_cast<float*>(tgt_data_ptr + field_y.offset);
            float* z = reinterpret_cast<float*>(tgt_data_ptr + field_z.offset);

            // read and transform
            rm::Vector3f vec = Tsc * rm::Vector3f{*x, *y, *z};

            // write back
            *x = vec.x;
            *y = vec.y;
            *z = vec.z;
        } else if(field_x.datatype == sensor_msgs::PointField::FLOAT64) {
            // Double
            double* x = reinterpret_cast<double*>(tgt_data_ptr + field_x.offset);
            double* y = reinterpret_cast<double*>(tgt_data_ptr + field_y.offset);
            double* z = reinterpret_cast<double*>(tgt_data_ptr + field_z.offset);

            // read and transform
            rm::Vector3d vec = Tscd * rm::Vector3d{*x, *y, *z};

            // write back
            *x = vec.x;
            *y = vec.y;
            *z = vec.z;
        }

        // normals?
    }

    ROS_INFO_STREAM("Publish cloud with size " << pcl_collection.width);
    pub_pcl.publish(pcl_collection);
}

int main(int argc, char** argv)
{
    // TODO
    ros::init(argc, argv, "pointcloud_collector_random");

    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");


    tf_buffer = std::make_shared<tf2_ros::Buffer>();
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    nh_p.param<std::string>("collector_frame", collector_frame, "odom");
    nh_p.param<int>("max_points", max_points, 100000);
    nh_p.param<double>("max_time_diff", max_time_diff, 0.1);
    nh_p.param<int>("insertions_per_scan", insertions_per_scan, 50);
    nh_p.param<double>("range_min", range_min, 0.2);
    nh_p.param<double>("range_max", range_max, 50.0);


    pcl_collection.header.frame_id = collector_frame;

    pub_pcl = nh.advertise<sensor_msgs::PointCloud2>("collected_cloud", 1);
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("input_cloud", 1, pclCB);

    
    ros::spin();

    tf_buffer.reset();
    tf_listener.reset();

    return 0;
}