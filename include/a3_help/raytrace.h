#ifndef RAYTRACE_H
#define RAYTRACE_H

#include <iostream>
#include <opencv2/highgui/highgui.hpp>

/*!
 *  \ingroup   RayTrace
 *  \brief     The ray trace class uses the entire laser line to determine unoccupied space in the image
 *  \details   The RayTrace class will take the calculated laser end point and robot position in image space (i.e the data sent in should be in an image reference)\n
 *             it will take those two points and draw a line between them in white (white is free space (255)). The logic used behind drawing the line is\n
 *             point on a line method. That is, this class will take the two points (end point and robot point) and calculate the equation of the line between\n
 *             the two points. It will then proceed to test the points inbetween and if the point fits on the line it will colour it in. As doubles are being compared\n
 *             the calculation will not look for exacts but rather ranges, so if the point fits on the line or slightly close to it, it will be accepted.
 *  \author    Denis Draca
 *  \version   1.0
 *  \date      2016
 *  \pre       none
 *  \bug       none reported as of 2016-07-02
 *  \warning
 */

class RayTrace
{
public:
    RayTrace();

    ///
    /// \brief map - The raytracing method. It will take 2 points, the end point colour and a reference to the image. It will colour the end point the given colour\n
    /// and compute the equation of the line between the two points and fill in any pixels that fit on that line. Data input should be in image space.\n
    /// When mapping, since a 'point on a line' method is used to calculate which pixels are unoccupied space, this can become computation heavy on large
    /// image sizes, in order to reduce the load, as the resolution reaches certain sizes the code will reduce the amount of calculations it performs
    /// in order to protect the integrity of the image.
    ///
    /// \param[in] (double) xpoint - x location of the end point of the laser line in image space
    /// \param[in] (double) ypoint - y location of the end point of the laser line in image space
    /// \param[in] (double) x - x point of the robot in image space
    /// \param[in] (double) y - y point of the robot in image space
    /// \param[in] (int) colour - greyscale colour number for the end point of the laser line (0-255)
    /// \param[in] (cv::Mat &) image_ - reference to the ogmap image. This will allow the method to modify the map to fit the newly calculated line
    ///

    void map(double xpoint, double ypoint, double x, double y, int colour, cv::Mat &image_);



private:
    ///
    /// \brief min - simple method that returns the minimum value between two points. This is used by map in order to constrain the location of the
    /// line equation to a smaller square instead of the whole image. Had it peformed the line equation on the whole map the map would quickly fill
    /// up with white.
    /// \param[in] (double) x - first value
    /// \param[in] (double) y - second value
    /// \return (double) - the smaller value of the two either x or y
    ///
    double min(double x, double y);
    ///
    /// \brief max - simple method that returns the maximum value between two points. This is used by map in order to constrain the location of the
    /// line equation to a smaller square instead of the whole image. Had it peformed the line equation on the whole map the map would quickly fill
    /// up with white.
    /// \param[in] (double) x - first value
    /// \param[in] (double) y - second value
    /// \return (double) - the larger value of the two either x or y
    ///
    double max(double x, double y, cv::Mat &image_);
};

#endif // RAYTRACE_H
