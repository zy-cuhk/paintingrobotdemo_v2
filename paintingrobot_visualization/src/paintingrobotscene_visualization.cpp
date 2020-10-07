#include <ros/ros.h>
#include <iostream> 
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>      
#include <pcl/point_types.h>   
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

bool loadCloud (const string &filename, PointCloud<PointXYZRGB> &cloud)
{
    ifstream fs;
    fs.open (filename.c_str (), ios::binary);
    if (!fs.is_open () || fs.fail ())
    {
        PCL_ERROR ("Could not open file '%s'! Error : %s\n", filename.c_str (), strerror (errno)); 
        fs.close ();
        return (false);
    }
  
    string line;
    std::vector<string> st;

    while (!fs.eof ())
    {
        getline (fs, line);
        if (line.empty())
        continue;

        boost::trim (line);
        boost::split (st, line, boost::is_any_of ("\t\r "), boost::token_compress_on);

        if (st.size () != 6)
        continue;
        
        pcl::PointXYZRGB point;
        point.x = float (atof (st[0].c_str ())); 
        point.y = float (atof (st[1].c_str ())); 
        point.z = float (atof (st[2].c_str ()))-5.80+30.0+6.3;
        point.r = uint8_t (atof (st[3].c_str ()));
        point.g = uint8_t (atof (st[4].c_str ()));
        point.b = uint8_t (atof (st[5].c_str ()));

        // std::cout << "xyz:" << point.x << ", "<< point.y << ", "<< point.z << ", "<< int(point.r) << ", "<< int(point.g) << ", "<< int(point.b) << std::endl;

        cloud.push_back(point);
    }
    fs.close ();
    cloud.width = std::uint32_t (cloud.size ()); cloud.height = 1; cloud.is_dense = true;
    return (true);
}

int main (int argc, char** argv)
{
    // step 1: set up ros node and topic 
    ros::init (argc, argv, "pub_pcl");
    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);
    sensor_msgs::PointCloud2 output;
    ros::Rate loop_rate(1);


    // step 2: input xyz format file and output point cloud 
    string xyz_file;
    ros::param::get("/xyz_flie_path", xyz_file);
    PointCloud<PointXYZRGB> cloud;
    if (!loadCloud (xyz_file, cloud)) 
        return (-1);

    // step 3: convert the cloud to ROS message
    std::cout << "Loaded "<< cloud.width * cloud.height<< " data points from pcd file with the following fields: "<< std::endl;
    pcl::toROSMsg(cloud, output);
    output.header.stamp=ros::Time::now();
    output.header.frame_id ="map";

    while (ros::ok())
    {
        // cout<<"publish cloud points"<<endl;
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::shutdown();
}