#include "a3_help/raytrace.h"
#include <stack>

RayTrace::RayTrace()
{
}

void RayTrace::map(double xpoint, double ypoint, double x, double y, int colour, cv::Mat &image_)
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

double RayTrace::min(double x, double y)
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

double RayTrace::max(double x, double y, cv::Mat &image_)
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

