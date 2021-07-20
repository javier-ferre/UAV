#include "GPS.hpp"

namespace GPSUtils
{
    double latitude = 0, longitude = 0, altitude = 0;
    double latitudeRef = 0, longitudeRef = 0, altitudeRef = 0;
    double xEcef = 0, yEcef = 0, zEcef = 0;
    double xEnu = 0, yEnu = 0, zEnu = 0;
    double xEnuInit = 0, yEnuInit = 0, zEnuInit = 0;
    double xRef = 0, yRef = 0, zRef = 0;
    double velocityEast = 0, velocityNorth = 0, velocityUp = 0;
    const double a = 6378137.0;
    const double b = 6356752.314245;
    const double DEG2RAD = 3.14159/180;
    bool flag = false;

    void GpsToEcef()
    {
        double e = 1-(pow(b,2)/pow(a,2));
        double N = a/(sqrt(1-e*pow(sin(latitude),2)));
        xEcef = (N+altitude)*cos(latitude)*cos(longitude);
        yEcef = (N+altitude)*cos(latitude)*sin(longitude);
        zEcef = ((pow(b,2)/pow(a,2))*N+altitude)*sin(latitude);
    }

    void EcefToEnu()
    {
        xEnu = -sin(longitudeRef)*(xEcef-xRef)+cos(longitudeRef)*(yEcef-yRef);
        yEnu = -sin(latitudeRef)*cos(longitudeRef)*(xEcef-xRef)-sin(latitudeRef)*sin(longitudeRef)*(yEcef-yRef)+cos(latitudeRef)*(zEcef-zRef);
        zEnu = cos(latitudeRef)*cos(longitudeRef)*(xEcef-xRef)+cos(latitudeRef)*sin(longitudeRef)*(yEcef-yRef)+sin(latitudeRef)*(zEcef-zRef);
    }

    void GPSCallback(ConstGPSPtr &_msg)
    {
        latitude = (_msg->latitude_deg())*DEG2RAD;
        longitude = (_msg->longitude_deg())*DEG2RAD;
        altitude = _msg->altitude();
        velocityEast = _msg->velocity_east();
        velocityNorth = _msg->velocity_north();
        velocityUp = _msg->velocity_up();

        GPSUtils::GpsToEcef();

        if (flag == false)
        {
            flag = true;

            latitudeRef = latitude;
            longitudeRef = longitude;
            altitudeRef = altitude;

            xRef = xEcef;
            yRef = yEcef;
            zRef = zEcef;
        }

        GPSUtils::EcefToEnu();


    }

    gazebo::transport::SubscriberPtr sub;
}

GPS::GPS()
{
    GPSUtils::sub = node->Subscribe("~/iris/gps_link/gps_receiver", GPSUtils::GPSCallback);
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
    return GPSUtils::xEnu;
}

double GPS::getY()
{
    return GPSUtils::yEnu;
}

double GPS::getZ()
{
    return GPSUtils::zEnu;
}