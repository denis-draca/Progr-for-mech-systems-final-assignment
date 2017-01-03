#include "a3_help/interpolation.h"

#define PI 3.14159265359

Interpolation::Interpolation()
{
}

double Interpolation::interx(double tillTime)
{
    double interpolated;
    double grad;
    if (hold.XPOS1 - hold.XPOS2 > -0.0001 && hold.XPOS1 - hold.XPOS2 <0.0001 )
    {
        return hold.XPOS2;
    }


    grad = (hold.XPOS2 - hold.XPOS1)/(hold.ODT2 - hold.ODT1);

    interpolated = grad*(tillTime - hold.ODT1) + hold.XPOS1;

    return interpolated;
}

double Interpolation::interY(double tillTime)
{
    double interpolated;
    double grad;
    if (hold.YPOS1 - hold.YPOS2 > -0.0001 && hold.YPOS1 - hold.YPOS2 <0.0001 )
    {
        return hold.YPOS2;
    }

    grad = (hold.YPOS2 - hold.YPOS1)/(hold.ODT2 - hold.ODT1);

    interpolated = grad*(tillTime - hold.ODT1) + hold.YPOS1;

    return interpolated;
}

double Interpolation::interYaw(double tillTime)
{
    double interpolated;
    double grad;

    if(hold.YAW1 > 1.5*PI)
    {
        if(hold.YAW2 >0 && hold.YAW2 < PI/2)
        {
            hold.YAW2 += 2*PI;
        }
    }

    if(hold.YAW1 < PI/2)
    {
        if(hold.YAW2 < (2*PI) && hold.YAW2 > 1.5*PI)
        {
            hold.YAW2 = hold.YAW2 - 2*PI;
        }
    }

    if (hold.YAW1 - hold.YAW2 > -0.0001 && hold.YAW1 - hold.YAW2 <0.0001 )
    {
        return hold.YAW2;
    }

    grad = (hold.YAW2 - hold.YAW1)/(hold.ODT2 - hold.ODT1);

    interpolated = grad*(tillTime - hold.ODT1) + hold.YAW1;

    return interpolated;
}

void Interpolation::pushYAW(double yaw)
{

    hold.YAW1 = hold.YAW2;
    hold.YAW2 = yaw;

    hold.YAW.push(yaw);
}

void Interpolation::pushX(double X)
{
    hold.XPOS1 = hold.XPOS2;
    hold.XPOS2 = X;

    hold.XPOS.push(X);
}

void Interpolation::pushY(double Y)
{
    hold.YPOS1 = hold.YPOS2;
    hold.YPOS2 = Y;

    hold.YPOS.push(Y);
}

void Interpolation::pushODtime(double time)
{
    hold.ODT1 = hold.ODT2;
    hold.ODT2 = time;

    hold.ODT.push_back(time);
}

void Interpolation::findTime(double laserTime)
{
    while(!hold.ODT.empty())
    {
        if(hold.ODT.front() <= laserTime)
        {
            hold.ODT1 = hold.ODT.front();
            hold.XPOS1 = hold.XPOS.front();
            hold.YPOS1 = hold.YPOS.front();
            hold.YAW1 = hold.YAW.front();
            break;
        }
        else
        {
            break;
        }

        hold.ODT.pop_front();
        hold.XPOS.pop();
        hold.YPOS.pop();
        hold.YAW.pop();
    }

    while(!hold.ODT.empty())
    {
        if(hold.ODT.front() >= laserTime)
        {
            hold.ODT2 = hold.ODT.front();
            hold.XPOS2 = hold.XPOS.front();
            hold.YPOS2 = hold.YPOS.front();
            hold.YAW2 = hold.YAW.front();
            break;

        }

        hold.ODT.pop_front();
        hold.XPOS.pop();
        hold.YPOS.pop();
        hold.YAW.pop();

    }

    while(!hold.ODT.empty())
    {
        hold.ODT.pop_front();
        hold.XPOS.pop();
        hold.YPOS.pop();
        hold.YAW.pop();
    }

}

void Interpolation::pushRanger(std::vector<float> ranger)
{
    hold.laser.push_back(ranger);
}

void Interpolation::pushlaserTime(double lasertime)
{
    hold.lasertime.push_back(lasertime);
}

void Interpolation::setup()
{
    for(int idx = 0; idx<hold.lasertime.size();idx++)
    {
        if(hold.lasertime.at(idx) < hold.ODT.back() && hold.lasertime.at(idx) > hold.ODT.front())
        {
            rangerFound = hold.laser.at(idx);
            laserTimeFound = hold.lasertime.at(idx);
            break;
        }
    }

    hold.laser.clear();
    hold.lasertime.clear();

    findTime(laserTimeFound);
}

std::vector<float> Interpolation::returnFoundRanger()
{
    return rangerFound;
}

double Interpolation::returnLaserTimer()
{
    return laserTimeFound;
}
