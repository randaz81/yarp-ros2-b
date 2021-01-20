/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "localization2D_nws_ros2.h"

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>

using namespace std::chrono_literals;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::dev::Nav2D;


YARP_LOG_COMPONENT(LOCALIZATION2D_NWS_ROS2, "yarp.ros2.localization2D_nws_ros2", yarp::os::Log::TraceType);


Ros2Init::Ros2Init()
{
    rclcpp::init(/*argc*/ 0, /*argv*/ nullptr);
    node = std::make_shared<rclcpp::Node>("yarprobotinterface_node");
}

Ros2Init& Ros2Init::get()
{
    static Ros2Init instance;
    return instance;
}


Localization2D_nws_ros2::Localization2D_nws_ros2() :
        yarp::os::PeriodicThread(0.01)
{
}

bool Localization2D_nws_ros2::attachAll(const PolyDriverList &device2attach)
{
    if (device2attach.size() != 1)
    {
        yCError(LOCALIZATION2D_NWS_ROS2, "Cannot attach more than one device");
        return false;
    }

    yarp::dev::PolyDriver * Idevice2attach = device2attach[0]->poly;
    if (Idevice2attach->isValid())
    {
        Idevice2attach->view(m_iLoc);
    }

    //attach the hardware device
    if (nullptr == m_iLoc)
    {
        yCError(LOCALIZATION2D_NWS_ROS2, "Subdevice passed to attach method is invalid");
        return false;
    }
    attach(m_iLoc);
    
   return true;
}

bool Localization2D_nws_ros2::detachAll()
{
    if (PeriodicThread::isRunning())
    {
        PeriodicThread::stop();
    }
    m_iLoc = nullptr;
    return true;
}

void Localization2D_nws_ros2::attach(yarp::dev::Nav2D::ILocalization2D *s)
{
    m_iLoc = s;
}

void Localization2D_nws_ros2::detach()
{
    if (PeriodicThread::isRunning())
    {
        PeriodicThread::stop();
    }
    m_iLoc = nullptr;
}

void Localization2D_nws_ros2::run()
{
	    double m_stats_time_curr = yarp::os::Time::now();
    if (m_stats_time_curr - m_stats_time_last > 5.0)
    {
        yCInfo(LOCALIZATION2D_NWS_ROS2) << "Running";
        m_stats_time_last = yarp::os::Time::now();
    }

    if (m_getdata_using_periodic_thread)
    {
        bool ret = m_iLoc->getLocalizationStatus(m_current_status);
        if (ret == false)
        {
            yCError(LOCALIZATION2D_NWS_ROS2) << "getLocalizationStatus() failed";
        }

        if (m_current_status == LocalizationStatusEnum::localization_status_localized_ok)
        {
            //update the stamp


            bool ret2 = m_iLoc->getCurrentPosition(m_current_position);
            if (ret2 == false)
            {
                yCError(LOCALIZATION2D_NWS_ROS2) << "getCurrentPosition() failed";
            }
            else
            {
                m_loc_stamp.update();
            }
            bool ret3 = m_iLoc->getEstimatedOdometry(m_current_odometry);
            if (ret3 == false)
            {
                //yCError(LOCALIZATION2D_NWS_ROS2) << "getEstimatedOdometry() failed";
            }
            else
            {
                m_odom_stamp.update();
            }
        }
        else
        {
            yCWarning(LOCALIZATION2D_NWS_ROS2, "The system is not properly localized!");
        }
    }

    if (1) publish_odometry_on_ROS_topic();
    if (1) publish_odometry_on_TF_topic();
	
	/*
    yCTrace(LOCALIZATION2D_NWS_ROS2);
    auto message = std_msgs::msg::String();
    
    if (m_iDevice!=nullptr)
    {
        bool ret = true;
        IRangefinder2D::Device_status status;
        yarp::sig::Vector ranges;
        ret &= m_iDevice->getRawData(ranges);
        ret &= m_iDevice->getDeviceStatus(status);
        
        if (ret)
        {
            int ranges_size = ranges.size();

            sensor_msgs::msg::LaserScan rosData;

            rosData.header.stamp = Ros2Init::get().node->get_clock()->now();    //@@@@@@@@@@@ CHECK HERE: simulation time?
            rosData.header.frame_id = m_frame_id;
            rosData.angle_min = m_minAngle * M_PI / 180.0;
            rosData.angle_max = m_maxAngle * M_PI / 180.0;
            rosData.angle_increment = m_resolution * M_PI / 180.0;
            rosData.time_increment = 0;             // all points in a single scan are considered took at the very same time
            rosData.scan_time = getPeriod();        // time elapsed between two successive readings
            rosData.range_min = m_minDistance;
            rosData.range_max = m_maxDistance;
            rosData.ranges.resize(ranges_size);
            rosData.intensities.resize(ranges_size);

            for (int i = 0; i < ranges_size; i++)
            {
                // in yarp, NaN is used when a scan value is missing. For example when the angular range of the rangefinder is smaller than 360.
                // is ros, NaN is not used. Hence this check replaces NaN with inf.
                if (std::isnan(ranges[i]))
                {
                   rosData.ranges[i] = std::numeric_limits<double>::infinity();
                   rosData.intensities[i] = 0.0;
                }
                else
                {
                   rosData.ranges[i] = ranges[i];
                   rosData.intensities[i] = 0.0;
                }
            }
            m_publisher->publish(rosData);
        }
        else
        {
            yCError(LOCALIZATION2D_NWS_ROS2, "Sensor returned error");
        }
    }*/
}

bool Localization2D_nws_ros2::open(yarp::os::Searchable &config)
{
    if(config.check("subdevice"))
    {
        Property       p;
        PolyDriverList driverlist;
        p.fromString(config.toString(), false);
        p.put("device", config.find("subdevice").asString());

        if(!m_driver.open(p) || !m_driver.isValid())
        {
            yCError(LOCALIZATION2D_NWS_ROS2) << "Failed to open subdevice.. check params";
            return false;
        }

        driverlist.push(&m_driver, "1");
        if(!attachAll(driverlist))
        {
            yCError(LOCALIZATION2D_NWS_ROS2) << "Failed to open subdevice.. check params";
            return false;
        }
        m_isDeviceOwned = true;
    }
 
    //wrapper params
    m_topic    = config.check("topic",  yarp::os::Value("laser_topic"), "Name of the ROS2 topic").asString();
    m_frame_id = config.check("frame",  yarp::os::Value("laser_frame"), "Name of the frameId").asString();
    m_period   = config.check("period", yarp::os::Value(0.010), "Period of the thread").asFloat64();
       
    //create the topic
    yCTrace(LOCALIZATION2D_NWS_ROS2);

    m_publisher_odom = Ros2Init::get().node->create_publisher<nav_msgs::msg::Odometry>(m_topic, 10);
    m_publisher_tf   = Ros2Init::get().node->create_publisher<tf2_msgs::msg::TFMessage>(m_topic, 10);
    yCInfo(LOCALIZATION2D_NWS_ROS2, "Opened topic: %s", m_topic.c_str());
        
    //start the publishig thread
    setPeriod(m_period);
    start();
    return true;
}

bool Localization2D_nws_ros2::close()
{
    return true;
}

void Localization2D_nws_ros2::publish_odometry_on_TF_topic()
{
    tf2_msgs::msg::TFMessage rosData;

    yarp::rosmsg::geometry_msgs::TransformStamped transform;
    transform.child_frame_id = m_child_frame_id;
    transform.header.frame_id = m_parent_frame_id;
    transform.header.seq = m_odom_stamp.getCount();
    transform.header.stamp = m_odom_stamp.getTime();
    double halfYaw = m_current_odometry.odom_theta / 180.0 * M_PI * 0.5;
    double cosYaw = cos(halfYaw);
    double sinYaw = sin(halfYaw);
    transform.transform.rotation.x = 0;
    transform.transform.rotation.y = 0;
    transform.transform.rotation.z = sinYaw;
    transform.transform.rotation.w = cosYaw;
    transform.transform.translation.x = m_current_odometry.odom_x;
    transform.transform.translation.y = m_current_odometry.odom_y;
    transform.transform.translation.z = 0;
    if (rosData.transforms.size() == 0)
    {
        rosData.transforms.push_back(transform);
    }
    else
    {
        rosData.transforms[0] = transform;
    }

    m_publisher_tf->publish(rosData);
}

void Localization2D_nws_ros2::publish_odometry_on_ROS_topic()
{
    nav_msgs::msg::Odometry rosData;

        odom.clear();
        odom.header.frame_id = m_fixed_frame;
        odom.header.seq = m_odom_stamp.getCount();
        odom.header.stamp = m_odom_stamp.getTime();
        odom.child_frame_id = m_robot_frame;

        odom.pose.pose.position.x = m_current_odometry.odom_x;
        odom.pose.pose.position.y = m_current_odometry.odom_y;
        odom.pose.pose.position.z = 0;
        yarp::sig::Vector vecrpy(3);
        vecrpy[0] = 0;
        vecrpy[1] = 0;
        vecrpy[2] = m_current_odometry.odom_theta;
        yarp::sig::Matrix matrix = yarp::math::rpy2dcm(vecrpy);
        yarp::math::Quaternion q; q.fromRotationMatrix(matrix);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        //odom.pose.covariance = 0;

        odom.twist.twist.linear.x = m_current_odometry.base_vel_x;
        odom.twist.twist.linear.y = m_current_odometry.base_vel_y;
        odom.twist.twist.linear.z = 0;
        odom.twist.twist.angular.x = 0;
        odom.twist.twist.angular.y = 0;
        odom.twist.twist.angular.z = m_current_odometry.base_vel_theta;
        //odom.twist.covariance = 0;

        m_publisher_odom->publish(rosData);
}
