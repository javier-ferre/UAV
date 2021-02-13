#include "GPS.hpp"

namespace GPSUtils
{
    double latitude=0, longitude=1, altitude=1;

    void GPSCallback(ConstGPSPtr &_msg)
    {
        latitude = _msg->latitude_deg();
        longitude = _msg->longitude_deg();
        altitude = _msg->altitude();
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