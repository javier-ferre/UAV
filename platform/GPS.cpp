#include "GPS.hpp"

namespace GPSUtils
{
    double latitude = 0, longitude = 0, altitude = 0;
    double xEcef = 0, yEcef = 0, zEcef = 0;
    double xEnu = 0, yEnu = 0, zEnu = 0;
    double xEnuInit = 0, yEnuInit = 0, zEnuInit = 0;
    double xLocal = 0, yLocal = 0, zLocal = 0;
    double velocityEast = 0, velocityNorth = 0, velocityUp = 0;
    const double a = 6378137.0;
    const double b = 6356752.314245;
    bool flag = false;

    void GpsToEcef()
    {
        double e = 1-(pow(b,2)/pow(a,2));
        double N = a/(sqrt(1-e*pow(sin(latitude*3.14159/180),2)));
        xEcef = (N+altitude)*cos(latitude*3.14159/180)*cos(longitude*3.14159/180);
        yEcef = (N+altitude)*cos(latitude*3.14159/180)*sin(longitude*3.14159/180);
        zEcef = ((pow(b,2)/pow(a,2))*N+altitude)*sin(latitude*3.14159/180);
    }

    void EcefToEnu()
    {
        xEnu = -sin(longitude*3.14159/180)*xEcef+cos(longitude*3.14159/180)*yEcef;
        yEnu = -sin(latitude*3.14159/180)*cos(longitude*3.14159/180)*xEcef-sin(latitude*3.14159/180)*sin(longitude*3.14159/180)*yEcef+cos(latitude*3.14159/180)*zEcef;
        zEnu = cos(latitude*3.14159/180)*cos(longitude*3.14159/180)*xEcef+cos(latitude*3.14159/180)*sin(longitude*3.14159/180)*yEcef+sin(latitude*3.14159/180)*zEcef;

        xLocal = xEnu - xEnuInit;
        yLocal = yEnu - yEnuInit;
        zLocal = zEnu - zEnuInit;
    }

    void GPSCallback(ConstGPSPtr &_msg)
    {
        latitude = _msg->latitude_deg();
        longitude = _msg->longitude_deg();
        altitude = _msg->altitude();
        velocityEast = _msg->velocity_east();
        velocityNorth = _msg->velocity_north();
        velocityUp = _msg->velocity_up();

        GPSUtils::GpsToEcef();
        GPSUtils::EcefToEnu();

        if (flag == false)
        {
            flag = true;
            xEnuInit = xEnu;
            yEnuInit = yEnu;
            zEnuInit = zEnu;
        }
    }

    gazebo::transport::SubscriberPtr sub;
}

GPS::GPS()
{
    GPSUtils::sub = node->Subscribe("~/iris_custom/gps_link/gps_receiver", GPSUtils::GPSCallback);
}

double GPS::getLatitude()
{
    return GPSUtils::latitude;
}

double GPS::getLongitude()
{
    return GPSUtils::longitude;
}

double GPS::getAltitude()
{
    return GPSUtils::altitude;
}

double GPS::getVelocityEast()
{
    return GPSUtils::velocityEast;
}

double GPS::getVelocityNorth()
{
    return GPSUtils::velocityNorth;
}

double GPS::getVelocityUp()
{
    return GPSUtils::velocityUp;
}

double GPS::getX()
{
    return GPSUtils::xLocal;
}

double GPS::getY()
{
    return GPSUtils::yLocal;
}

double GPS::getZ()
{
    return GPSUtils::zLocal;
}