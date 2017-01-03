#ifndef INTERPOLATION_H
#define INTERPOLATION_H

#include <vector>
#include <deque>
#include <queue>
#include <iostream>

/*!
 *  \ingroup   Interpolation
 *  \brief     interpolation class tries to find readings that correspond to one another and transform the readings to be usable
 *  \details   This classes main task is to interpolate the robot positions reading to what it wouldve been at the laser time. It does this \n
 *             using a linear method (draws a line between two points and finds the reading that would fit at time inbetween those two points)\n
 *             The reason I interpolate the position and not the laser, is to reduce the amount of calculations necessary as interpolating the laser
 *             would involve performing the calculation of every reading (all 720). \n \n
 *
 *             This class will also set up all the readings for later use, as the readings are generated they get pushed into this class which holds\n
 *             them for later use. Looping a few times ensures that we have enough readings to ensure that we have at least one laser time sitting inbetween two robot readings.
 *  \author    Denis Draca
 *  \version   1.0
 *  \date      2016
 *  \pre       none
 *  \bug       none reported as of 2016-07-02
 *  \warning
 */

class Interpolation
{

private:
    ///
    /// \brief The InterHolder struct - This struct is used to hold all the data members that will be used for interpolation process
    ///
    struct InterHolder
    {
        std::deque<double> ODT;                                 //!< All of the robot time readings before interpolation takes place
        std::queue<double> YAW;                                 //!< All of the robot yaw readings before interpolation takes place
        std::queue<double> XPOS;                                //!< All of the robot x-position readings before interpolation takes place
        std::queue<double> YPOS;                                //!< All of the robot y-position readings before interpolation takes place

        std::vector< std::vector<float> > laser;                //! All of the laser distance readings before interpolation takes place
        std::vector<double> lasertime;                          //! All of the laser time readings before interpolation takes place

        double ODT1 = 0;                                        //!Robot time reading 1 (used as x1 for interpolation to work out the line between the two points)
        double ODT2 = 0;                                        //!Robot time reading 2 (used as x2 for interpolation to work out the line between the two points)
        //YAW DATA HOLDER
        double YAW1 = 0;                                        //!Robot yaw reading 1 (used as y1 for interpolation to work out the line between the two points)
        double YAW2 = 0;                                        //!Robot yaw reading 2 (used as y2 for interpolation to work out the line between the two points)
        //X DATA HOLDER
        double XPOS1 = 0;                                       //!Robot x-position reading 1 (used as y1 for interpolation to work out the line between the two points)
        double XPOS2 = 0;                                       //!Robot x-position reading 2 (used as y2 for interpolation to work out the line between the two points)
        //Y DATA HOLDER
        double YPOS1 = 0;                                       //!Robot y-position reading 1 (used as y1 for interpolation to work out the line between the two points)
        double YPOS2 = 0;                                       //!Robot y-position reading 2 (used as y2 for interpolation to work out the line between the two points)
    };

private:
    InterHolder hold;                            //!< Object of the interholder struct

    std::vector<float> rangerFound;              //!< Data member storing the located ranger data where its time stamp fits between two position readings
    double laserTimeFound;                       //!< data member storing the timestamp that fits between two position readings
private:
    ///
    /// \brief findTime - This is a special method. Given the laser time reading it will search through all the position readings (using the time as a reference)
    /// and pick two points that the laser reading sits inbetween, it will then set up the members in the interholder struct
    /// \param[in] (double) laserTime - The laser time that will be compared to the stored robot readings
    ///
    void findTime(double laserTime);
public:
    Interpolation(); //!< Constructor

public:


    ///
    /// \brief interx - transforms the x position of the robot using interpolation and multiple other readings as the line points
    /// \param[in] (double) tillTime - the time unit that the x position will be interpreted to. Where in this case it will be the laser time at that point
    /// \return (double) - new x value
    ///
    double interx(double tillTime);
    ///
    /// \brief interY - transforms the y position of the robot using interpolation and multiple other readings as the line points
    /// \param[in] (double) tillTime - the time unit that the y position will be interpreted to. Where in this case it will be the laser time at that point
    /// \return (double) - new y value
    ///
    double interY(double tillTime);
    ///
    /// \brief interYaw - transforms the yaw of the robot using interpolation and multiple other readings as the line points
    /// \param[in] (double) tillTime - the time unit that the yaw will be interpreted to. Where in this case it will be the laser time at that point
    /// \return (double) - new yaw value
    ///
    double interYaw(double tillTime);
    ///
    /// \brief pushYAW - pushes the inputted yaw reading into the appropriate member of the interholder struct
    /// \param[in] (double) yaw - the raw yaw reading
    ///
    void pushYAW(double yaw);
    ///
    /// \brief pushX- pushes the inputted robot x-position reading into the appropriate member of the interholder struct
    /// \param[in] (double) X - the raw x-position reading
    ///
    void pushX(double X);
    ///
    /// \brief pushY- pushes the inputted robot y-position reading into the appropriate member of the interholder struct
    /// \param[in] (double) Y - the raw y-position reading
    ///
    void pushY(double Y);
    ///
    /// \brief pushODtime - pushes the inputted robot time reading into the appropriate member of the interholder struct
    /// \param[in] (double) time - the raw time reading
    ///
    void pushODtime(double time);

    ///
    /// \brief pushRanger - Pushes the global ranger readings into a local struct holder for later calculation
    /// \param[in] (std::vector<float>) ranger - Global laser readings
    ///
    void pushRanger(std::vector<float> ranger);
    ///
    /// \brief pushlaserTime - Pushes the global ranger time readings into a local struct holder for later calculation
    /// \param[in] (double) lasertime - globle time stamp for a laser reading
    ///
    void pushlaserTime (double lasertime);
    ///
    /// \brief setup - sets up the holder struct variables (clears the queues when done) and prepares everything for interpolation, after this the 'inter' methods can be
    /// called to return new variables. This essentially looks through all the available readings and finds which time readings are closest together and the interpolates
    /// to get the result as accurate as possible.
    ///
    void setup();
    ///
    /// \brief returnFoundRanger - Returns the ranger reading that corresponds to a time between two robot readings
    /// \return (std::vector<float>) - the laser reading
    ///
    std::vector<float> returnFoundRanger();
    ///
    /// \brief returnLaserTimer - returns the timestamp that corresponds to a time between two robot readings
    /// \return (double) - The timestamp
    ///
    double returnLaserTimer();


};

#endif // INTERPOLATION_H
