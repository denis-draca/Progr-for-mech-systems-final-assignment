#include <gtest/gtest.h>
#include <climits>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


//#include "rostalker.h"
// bad function:
// for example: how to deal with overflow?
int add(int a, int b){
    return a + b;
}

TEST(NumberCmpTest, ShouldPass){
    ASSERT_EQ(3, add(1,2));
}

double min(double x, double y)
{
    if(x<=0 || y<= 0) //bit of protection if the size is a bit off
    {
        return 0;
    }
    if (x < y)
    {
        return x;
    }
    else
    {
        return y;
    }
}

double max(double x, double y, cv::Mat &image_)
{
    if (x > image_.rows || y > image_.rows)//bit of protection if the size is a bit off
    {
        return image_.rows;
    }
    if (x > y)
    {
        return x;
    }
    else
    {
        return y;
    }
}
void map(double xpoint, double ypoint, double x, double y, int colour, cv::Mat &image_)
{
    int stepx = 1;//Since my code performs a 'point on the line' test to ray trace, this can result in alot of calculations at large resolutions
    // to decrease the load im going to change the for loop increment from every pixel, to every other pixel for resolutions > 500x500
    int stepy = 1;

    if(image_.rows > 500)
    {
        stepx = 2;
        stepy = 2;
    }

    if(image_.rows > 1000)
    {
        stepx = 3;
        stepy = 2;
    }

    if (xpoint < image_.rows && ypoint < image_.rows && xpoint > 0 && ypoint > 0)
    {
        if(image_.at<uchar>(ypoint,xpoint) != 0)
        {
            image_.at<uchar>(ypoint,xpoint) = colour;
        }
    }

    double m;


    double x1 = x;
    double y1 = y;

    if((xpoint - x1) > 0.1 ||(xpoint - x1) < -0.1 ) //safety precaution incase the two x values are very close to each other the the gradient will become very large and if exact then it will be a division by 0, not allowed
    {
        m = (y1 - ypoint)/(x1 - xpoint);
    }
    else
    {
        for (int i = max(y1,ypoint,image_); i > min(y1,ypoint); i--)
        {
            if (i < image_.rows && x1 < image_.rows && i > 0 && x1 > 0)
            {
                if(image_.at<uchar>(i,x1) == 126)
                {

                    image_.at<uchar>(i,x1) = 255;
                }
            }
        }
        return;
    }

    for (double i = (min(x1,xpoint) + 1); i < (max(x1, xpoint,image_) - 1);i+=stepx)
    {

        for(double j = (min(y1, ypoint) + 1); j<(max(y1, ypoint,image_)-1); j+=stepy)
        {

            if((j <= (m*(i-xpoint) + ypoint) + 1) &&(j >= (m*(i-xpoint) + ypoint) - 1))
            {
                if (i < image_.rows && j < image_.rows && i > 0 && j > 0)
                {
                   if(image_.at<uchar>(j,i) == 126)
                   {
                       image_.at<uchar>(j,i) = 255;
                   }
                }
            }
        }
    }
}


TEST(RayTrace, ShouldPass)
{
    double mapSize=50;
    double resolution=0.1;

    int pixels = (int) mapSize / resolution;

    cv::Mat image_ = cv::Mat::zeros(pixels,pixels, CV_8UC1);

    map(0,0,0,0,255,image_); //should draw a line from ogmap (0,0) to gazebo (0,0)

    ASSERT_EQ(0,image_.at<uchar>(0,0));

    //testing to see a diagonal line, from the top left to the centre has been drawn.
    //Essentially a test to see if the equation between 2 lines did its job

    for (double x = 1; x < 249 ;x++)
    {

        for(double y = 1; y<249; y++)
        {
            if((y == (1*(x-0) + 0)) == 255)
            {
                ASSERT_EQ(255, image_.at<uchar>(y,x));
            }
        }
    }

}


TEST(image2global, ShouldPass)
{
    //testing to see if the code gives me the same result as what i expected by hand calculation
    double mapSize=20.0;
    double resolution=0.1;

    int pixels = (int) mapSize / resolution;
    int toPixels = pixels/20;

    int ximage = 250;
    int yimage = 250;

    double xglobal;
    double yglobal;

    yglobal = ((pixels/2) - ximage)/pixels;
    xglobal = ((pixels/2) - yimage)/pixels;

    ASSERT_EQ(0, yglobal);
    ASSERT_EQ(0, xglobal);

}

TEST(global2image, ShouldPass)
{
    //testing to see if the code gives me the same result as what i expected by hand calculation
    double mapSize=50.0;
    double resolution=0.1;

    int pixels = (int) mapSize / resolution;
    int toPixels = pixels/20;

    double xglobal = 5.2; //global test values
    double yglobal = 3.8;

    int ximage;
    int yimage;

    ximage = pixels/2 - yglobal*toPixels;
    yimage = pixels/2 - xglobal*toPixels;

    ASSERT_EQ(120, yimage);
    ASSERT_EQ(155, ximage);
}

TEST(ImageSize,ShouldPass){

    //! I would STRONGLY suggest to write two functions
    //! image2global and global2image and Unit test them
    //! The functions would allow to go from image space
    //! To Global coordinate system and vice versa

    //! The below code does not do this, it merley sets up the framework
    //! For testing and allows accessing an OPENCV image

    double mapSize=20.0;
    double resolution=0.1;

    int pixels = (int) mapSize / resolution;

    // Create an OgMap which is a grayscale (mono) image of size pixels
    cv::Mat image = cv::Mat::zeros(pixels,pixels, CV_8UC1);

    // Let's check map size compared to allocation, just in case
    ASSERT_EQ(pixels, image.rows);
    ASSERT_EQ(pixels, image.cols);

    // Let's check the map is allocated all zeros
    ASSERT_EQ(0,image.at<uchar>(0,0));

    // Draw a link from top left to botom right corner
    cv::line(image,cv::Point(0,0),cv::Point(pixels,pixels),cv::Scalar(255,255,255),1);

    // Let's check the centre is now white (255)
    ASSERT_EQ(255,image.at<uchar>(pixels/2,pixels/2));
}


//TEST(RtTest, TalkerFunction){
//    RosTalker rt;
//    ASSERT_EQ(3, rt.add(1,2));
//}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    // do not forget to init ros because this is also a node
//    ros::init(argc, argv, "talker_tester");
    return RUN_ALL_TESTS();
}
