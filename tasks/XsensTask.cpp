#include "XsensTask.hpp"


using namespace imu;



XsensTask::XsensTask(std::string const& name)
    : XsensTaskBase(name), m_driver(NULL)
{
    _mode = xsens_imu::CAL_AND_ORI_DATA;
}

XsensTask::~XsensTask()
{
    delete m_driver;
}
 

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See XsensTask.hpp for more detailed
// documentation about them.

bool XsensTask::configureHook()
{
    std::auto_ptr<xsens_imu::XsensDriver> driver(new xsens_imu::XsensDriver());
    if( !driver->open( _port.value() ) ) {
        std::cerr << "Error opening device '" << _port.value() << "'" << std::endl;
        return false;
    }

    if( !driver->setReadingMode( _mode ) ) {
        std::cerr << "Error changing reading mode to " << _mode;
        return false;
    }

    m_driver = driver.release();
    return true;
}

// bool IMUAcquisition::startHook()
// {
//     return true;
// }

void XsensTask::updateHook()
{
    xsens_imu::errorCodes retval;
    retval = m_driver->getReading();

    if( retval == xsens_imu::NO_ERROR ) {
        if( _mode == xsens_imu::CAL_AND_ORI_DATA || _mode == xsens_imu::ONLY_ORI_DATA ) {
            DFKI::OrientationReading reading;
            reading.value = m_driver->getOrientation();
            _orientation_readings.write( reading );
        }
        
        if( _mode == xsens_imu::CAL_AND_ORI_DATA || _mode == xsens_imu::ONLY_CAL_DATA ) {
            {
                DFKI::AccelerometerReading reading;
                reading.value = m_driver->getCalibratedAccData();
                _acc_readings.write( reading );
            }

            {
                DFKI::AngularRateReading reading;
                reading.value = m_driver->getCalibratedGyroData();
                _gyro_readings.write( reading );
            }

            {
                DFKI::MagnetometerReading reading;
                reading.value = m_driver->getCalibratedGyroData();
                _mag_readings.write( reading );
            }
        }
   } 

    if( retval == xsens_imu::ERROR_TIMEOUT ) {
        // a time out is not yet an error per se
        // TODO: add something like a timeout count which then would be an
        // error
        std::cerr << "IMU driver timout." << std::endl;
    }

    if( retval == xsens_imu::ERROR_OTHER ) {
        std::cerr << "IMU driver error" << std::endl;
        return error();
    }
}

// void XsensTask::errorHook()
// {
// }
// void XsensTask::stopHook()
// {
// }
// void XsensTask::cleanupHook()
// {
// }

