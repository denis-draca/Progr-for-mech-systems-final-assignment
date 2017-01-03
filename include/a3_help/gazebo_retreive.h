#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/ModelStates.h"
#include "tf/transform_datatypes.h"

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sstream>
#include <iostream>
#include <string>

#include <thread>
#include <chrono>
#include <deque>
#include <mutex>
#include <random>
#include <queue>
#include <vector>


#include "raytrace.h"
#include "interpolation.h"

/*!
 *  \ingroup   GazeboRetrieve
 *  \brief     gazebo retrieve class, attains and uses laser and odometer readings
 *  \details   This class will take the readings from gazebo such as the laser readings and the robot position and yaw. This will be attained using\n
 *             the ros ecosystem for communication accross nodes.It is the main system for dealing with ros in this project. This data will be pushed to \n
 *             the interpolation class for later data calculation. It will deal with the global to image and image to global to image conversions as needed\n
 *             along with reading the laser data and converting it to global space, then image space and giving it to the raytracing class for mapping.\n
 *             It will also create an image itself that will be og map. This map will be pushed out at the end so the map can be visually seen.\n
 *             finally it will compute the furthest unknown point and push the point (in global space) through ros under the topic \exploration_goal\n
 *             The message in this topic will be wrapped up in a geometery_msg::Point format.
 *
 *             MAP KEY:\n
 *                  White = 255 = Free space (unoccupied)\n
 *                  Grey = 126 = Unexplored space (unknown)\n
 *                  Black = 0 = Wall (occupied)\n
 *
 *
 *  \author    Denis Draca
 *  \version   1.0
 *  \date      2016
 *  \pre       none
 *  \bug       none reported as of 2016-07-02
 *  \warning
 */

class GazeboRetrieve{

private:
    ///
    /// \brief The DataBuffer struct - Holds the robot position readings in order to be shared accros threads
    ///
    struct DataBuffer
    {
        std::deque<geometry_msgs::Pose> poseDeq;    //! Deque to store odometry message
        std::deque<ros::Time> timeStampDeq;         //! Reque to store time of message
        std::mutex buffer_mutex_;                   //! Mutex used to secure buffer
    };

    ///
    /// \brief The LaserTransfer struct - This struct is used to store the laser specific data that is attained in lasercallback. Allowing me to
    /// transfer it between threads
    ///
    struct LaserTransfer
    {
        std::mutex mxLaser;                                     //!< The mutex associated with the laser, it will be used to lock and unlock all the following data members
        std::vector<float> rangeData;                           //!< readings of the laser ranger
        double smallestAngle;                                   //!< smallest angle of the laser
        double angleIncrement;                                  //!< Angle step of the laser
        double maxRange;                                        //!< Maximum range of the laser
        double minRange;                                        //!< Minimum range of the laser

        double laserTime;                                       //!< The time that the reading was taken
    };


public:
    GazeboRetrieve(ros::NodeHandle nh);             //! Constructore

    ~GazeboRetrieve();                              //! Destructor
    ///
    /// \brief seperateThread - This method will constantly run (while ros is ok), it will get the laser and robot data and push them into the interpolation object
    /// it will then use the ray trace class to map by calculating the laser end points and sending them to the ray tracing class. This thread will deal with the space
    /// conversions
    ///
    void seperateThread();                          //! Thread of execution

private:
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);   //! Odometry Callback
    void laserCallback(const sensor_msgs::LaserScanConstPtr& msg);//! Laser Calback
    ///
    /// \brief exploration_goal - This method will take the robots current position on the image map, convert it to global position and calculate the furthest unexplored point from the robots position
    /// this position will be placed into a ros message and published via the topic /exploration_goal
    /// \param[in] (double) xpoint - robot x position in image space
    /// \param[in] (double) ypoint - robot y position in image space
    /// \param toPixels - the meters to pixels scale
    ///
    void exploration_goal(double xpoint, double ypoint, double toPixels);




private:
    ros::NodeHandle nh_;                    //! Node Handle
    image_transport::ImageTransport it_;    //! Image transport
    image_transport::Publisher image_pub_;  //! Publisher
    ros::Subscriber sub1;                   //! Subscriber 1
    ros::Subscriber sub2;                   //! Subscriber 2

    ros::Publisher goal_ad = nh_.advertise<geometry_msgs::Point>("/exploration_goal", 10);

    cv::Mat image_;                         //! Storage for OgMap in Mat
    int count_;                             //! A counter to allow executing items on N iterations
    int pixels_;                            //! size of OgMap in pixels

    DataBuffer buffer;                      //! And now we have our container


    LaserTransfer lasertransfer;            //!< Object of the lasertransfer struct

    Interpolation inter;                    //!< Interpolation object

    RayTrace ray;                           //!< Object of the raytracing class

    int publish_count;                      //! Counter to allow publish to occur ever X cycles

};


