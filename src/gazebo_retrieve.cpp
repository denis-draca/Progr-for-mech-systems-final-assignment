#include "a3_help/gazebo_retreive.h"

#define PI 3.14159265359

GazeboRetrieve::GazeboRetrieve(ros::NodeHandle nh)
: nh_(nh), it_(nh)
{
    sub1 = nh_.subscribe("odom", 1000, &GazeboRetrieve::odomCallback,this);
    sub2 = nh_.subscribe("scan", 10, &GazeboRetrieve::laserCallback,this);
    image_pub_ = it_.advertise("ogmap/image", 1);

    ros::NodeHandle pn("~");
    double mapSize;
    double resolution;
    pn.param<double>("map_size", mapSize, 20.0);
    pn.param<double>("resolution", resolution, 1.0);

    pixels_ = (int) mapSize / resolution;
    std::cout << "size:" << mapSize << " resolution:" << resolution <<
                 " pixels:" << pixels_ << std::endl;

    //! Create an OgMap which is a grayscale (mono) image of size pixels
    image_ = cv::Mat::zeros(pixels_,pixels_, CV_8UC1);

    count_ =0;
    publish_count = 0;
    cv::waitKey(30);

}

GazeboRetrieve::~GazeboRetrieve()
{

}


void GazeboRetrieve::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    ros::Time timeOdom = ros::Time::now();;
    //Let's get the pose out from odometry message
    // REMEBER: on command line you can view entier msg as
    //rosmsg show nav_msgs/Odometry
    geometry_msgs::Pose pose=msg->pose.pose;



    //std::cout<<"TIME: "<<timebot<<std::endl;
    buffer.buffer_mutex_.lock();
//    timebot = timeOdom.toSec();
//    pushODtime(timebot);

    //std::unique_lock<std::mutex> locker(buffer.buffer_mutex_);
    buffer.poseDeq.push_back(pose);
    buffer.timeStampDeq.push_back(msg->header.stamp);
//    buffer.odNew = true;
//    //locker.unlock();
//    buffer.convar.notify_all();
    buffer.buffer_mutex_.unlock();

}



void GazeboRetrieve::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
    double xpoint;
    double ypoint;
    double toPixels = pixels_/20;
    double closest_point=msg->range_max;
    double angle=0;
    double x,y;
    for (unsigned int idx=0 ; idx < msg->ranges.size() ; idx++){
        if(msg->ranges.at(idx)<closest_point){
            closest_point=msg->ranges.at(idx);
            angle=msg->angle_min+(idx*msg->angle_increment);
        }
    }
    ros::Time timeLaser = msg->header.stamp;
    x = closest_point * cos(angle);
    y = closest_point * sin(angle);


    lasertransfer.mxLaser.lock();
    lasertransfer.rangeData.clear();
    lasertransfer.rangeData = msg->ranges;
    lasertransfer.angleIncrement = msg->angle_increment;
    lasertransfer.maxRange = msg->range_max;
    lasertransfer.minRange = msg->range_min;

    lasertransfer.laserTime = timeLaser.toSec();
//    buffer.laserNew = true;

//    locker.unlock();

//    buffer.convar.notify_all();
    lasertransfer.mxLaser.unlock();

    //laserMade = true;

    //locker.unlock();

    //convar.notify_all();



    std::cout << timeLaser << " L: [d,theta,x,y]=[" << closest_point << "," << angle << "," << x << "," << y << "]" << std::endl;

    //std::this_thread::sleep_for(std::chrono::milliseconds(10));
}


void GazeboRetrieve::exploration_goal(double xpoint, double ypoint, double toPixels)
{
    geometry_msgs::Point goal;
    double distance = 0;
    double distance_max = 0;

    double xMaxDis = 0;
    double yMaxDis = 0;

    for(int x = 0; x < image_.cols ; x++)
    {
        for(int y = 0; y < image_.rows; y++)
        {
            if (x < image_.rows && y < image_.rows && x > 0 && y > 0)
            {
                if(image_.at<uchar>(y,x) == 126)
                {
                    distance = sqrt(pow((xpoint - x),2) + pow((ypoint - y),2));

                    if(distance > distance_max)
                    {
                        distance_max = distance;
                        xMaxDis = x;
                        yMaxDis = y;
                    }
                }
            }
        }
    }
    goal.x = ((image_.cols/2) - yMaxDis)/toPixels;
    goal.y = ((image_.rows/2) - xMaxDis)/toPixels;
    goal.z = 0;
    goal_ad.publish(goal);

}



void GazeboRetrieve::seperateThread() {
    /**
     * The below loop runs until ros is shutdown, to ensure this thread does not remain
     * a zombie thread
     *
     * The loop locks the buffer, checks the size
     * And then pulls items: the pose and timer_t
     * You can think if these could have been combined into one ...
     */


     double yaw,x,y;
     double xpoint,ypoint;
     int ranOnce = 0;
     double angle;
     double x2point;
     double y2point;


     double xlaser;
     double ylaser;

     double laserMin;
     double laserMax;
     double angleIncrement;
     double laserTime;

     int deqSz =-1;
     int laserSz = -1;

     double toPixels = pixels_/20;

     std::vector<float> rangerFound;
     std::vector<float> ranger;

     /// The below gets the current Ros Time
     ros::Time timeOdom = ros::Time::now();;
     while (ros::ok()) {
         deqSz = -1;
         laserSz = -1;

//         std::unique_lock<std::mutex> locker(buffer.buffer_mutex_);
//         while(!buffer.laserNew && !buffer.odNew)
//         {
//             buffer.convar.wait(locker);
//         }
         lasertransfer.mxLaser.lock();
         laserSz = lasertransfer.rangeData.size();
         if(laserSz > 0)
         {
             laserMin = lasertransfer.minRange;
             laserMax = lasertransfer.maxRange;

             angleIncrement = lasertransfer.angleIncrement;
             laserTime = lasertransfer.laserTime;
             rangerFound = lasertransfer.rangeData;

         }

         lasertransfer.mxLaser.unlock();


         buffer.buffer_mutex_.lock();
         deqSz = buffer.poseDeq.size();
         if (deqSz > 0) {
             geometry_msgs::Pose pose=buffer.poseDeq.front();
             tf::Quaternion quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

             quat = quat.normalize();

             yaw = tf::getYaw(quat);
             //yaw = tf::getYaw(pose.orientation);
             x = pose.position.x;
             y = pose.position.y;
             timeOdom = buffer.timeStampDeq.front();
             buffer.poseDeq.pop_front();
             buffer.timeStampDeq.pop_front();
         }

//         buffer.laserNew = false;
//         buffer.odNew = false;
         //locker.unlock();
         buffer.buffer_mutex_.unlock();

         if(deqSz>0){

             if (yaw < 0) //convert yaw reading from (pi to -pi) to (0 to 2pi)
             {
                 yaw = 2*PI + yaw;
             }

             x += 0.499949*cos(yaw); //these two offset the robot position so that it sits on the laser position (essentially i wanted the laser position to be basis for everything)
             y += 0.499949*sin(yaw);

             if(!ranOnce)
             {
                 for (int z = 0; z<image_.cols; z++)
                 {
                     for(int ver = 0; ver<image_.rows; ver++)
                     {
                         image_.at<uchar>(ver,z) = 126;
                     }
                 }
                 ranOnce = 1;
             }


             inter.pushYAW(yaw);
             inter.pushY(y);
             inter.pushX(x);
             inter.pushODtime(timeOdom.toSec());

             inter.pushlaserTime(laserTime);
             inter.pushRanger(rangerFound);

             if (count_> 10){ //to make sure i have enough readings stored so i can interpolate (i look through them to find two points that a laser time sits in between)
                 count_=0;

                 inter.setup();

                 ranger = inter.returnFoundRanger();
                 laserTime = inter.returnLaserTimer();

                 xpoint = image_.rows/2 - y*toPixels;
                 ypoint = image_.cols/2 - x*toPixels;

                 if (xpoint < image_.rows && ypoint < image_.rows && xpoint > 0 && ypoint > 0)
                 {
                    image_.at<uchar>(ypoint,xpoint) = 255;
                 }


                 yaw = inter.interYaw(laserTime);
                 x = inter.interx(laserTime);
                 y = inter.interY(laserTime);

                 x2point = image_.rows/2 - y*toPixels;
                 y2point = image_.cols/2 - x*toPixels;


                for(unsigned int idx=0 ; idx <ranger.size() ; idx++)
                 {

                    if(ranger.at(idx) > (laserMin) && ranger.at(idx)< laserMax)
                    {

                        angle = PI - yaw - (angleIncrement*idx);

                        ylaser = (ranger.at(idx))*(cos(angle)) + y;
                        xlaser = (ranger.at(idx))*(sin(angle)) + x;


                        xpoint = image_.rows/2 - ylaser*toPixels;
                        ypoint = image_.rows/2 - xlaser*toPixels;



                        ray.map(xpoint,ypoint,x2point,y2point,0,image_);
                        //ray.map(-10,1000,250,250,255,image_);
                    }
                    else
                    {
                        angle = PI - yaw - (angleIncrement*idx);
                        ylaser = (ranger.at(idx))*(cos(angle)) + y;
                        xlaser = (ranger.at(idx))*(sin(angle)) + x;

                        xpoint = image_.rows/2 - ylaser*toPixels;
                        ypoint = image_.rows/2 - xlaser*toPixels;




                        ray.map(xpoint,ypoint,x2point,y2point,255,image_);
                        //ray.map(-10,1000,250,250,255,image_);
                    }
                 }

                ranger.clear();
                rangerFound.clear();

                 std::cout << timeOdom  << " O: [x,y,yaw]=[" <<
                              x << "," << y << "," << yaw << "]" << std::endl;

                 sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_).toImageMsg();
                 image_pub_.publish(msg);
             }
             else{
                 count_++;

                 if (publish_count > 5)
                 {
                     publish_count = 0;
                     xpoint = image_.rows/2 - y*toPixels;
                     ypoint = image_.cols/2 - x*toPixels;

                     exploration_goal(xpoint, ypoint, toPixels);
                 }
                 else
                 {
                     publish_count++;
                 }
             }
         }
         else{


         }
     }
}




