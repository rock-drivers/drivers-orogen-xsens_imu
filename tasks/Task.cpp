#include "Task.hpp"

#include <rtt/FileDescriptorActivity.hpp>
#include <TimestampEstimator.hpp>


using namespace xsens_imu;


RTT::FileDescriptorActivity* Task::getFileDescriptorActivity()
{ return dynamic_cast< RTT::FileDescriptorActivity* >(getActivity().get()); }


Task::Task(std::string const& name)
    : TaskBase(name), m_driver(NULL), timeout_counter(0), timestamp_estimator(0)
{
}

Task::~Task()
{
    delete m_driver;
    delete timestamp_estimator;
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    timestamp_estimator = new aggregator::TimestampEstimator(base::Time::fromSeconds(2));

    std::auto_ptr<xsens_imu::XsensDriver> driver(new xsens_imu::XsensDriver());
    if( !driver->open( _port.value() ) ) {
        std::cerr << "Error opening device '" << _port.value() << "'" << std::endl;
        return false;
    }

    if( !driver->setReadingMode( xsens_imu::CAL_AND_ORI_DATA ) ) {
        std::cerr << "Error changing reading mode to CAL_AND_ORI_DATA";
        return false;
    }
    
    // if( !driver->setScenario( _scenario.get() ) ) {
    //     std::cerr << "Error changing scenario to " << _scenario.get() << std::endl;
    //     return false;
    // }
    
    getFileDescriptorActivity()->watch(driver->getFileHandle());

    m_driver = driver.release();
    return true;
}

bool Task::startHook()
{
    int retval;
    do
    {
	retval = m_driver->getReading();
    }
    while(retval == xsens_imu::NO_ERROR);
}

void Task::updateHook()
{
    // since this task is fd driven, we assume that the reception of data on
    // the port is the closest we can get to the actual acquisition time. This
    // can later be fixed to using a hardware sync-out from the IMU.

    xsens_imu::errorCodes retval;
    retval = m_driver->getReading();
    base::Time ts = timestamp_estimator->update(base::Time::now());

    if( retval == xsens_imu::NO_ERROR ) {
        timeout_counter = 0;

	base::samples::RigidBodyState reading;
	reading.time = ts;
	reading.orientation = m_driver->getOrientation();
        _orientation_samples.write( reading );

        base::samples::IMUSensors sensors;
        sensors.time = ts;
	sensors.acc   = m_driver->getCalibratedAccData();
	sensors.gyro  = m_driver->getCalibratedGyroData();
	sensors.mag   = m_driver->getCalibratedMagData();
	_calibrated_sensors.write( sensors );
   } 

    if( retval == xsens_imu::ERROR_TIMEOUT ) {
        timeout_counter++;

        if( timeout_counter >= _max_timeouts ) {
            std::cerr << "IMU driver timout." << std::endl;
            return fatal(IO_ERROR);
        }
   }

    if( retval == xsens_imu::ERROR_OTHER ) {
        std::cerr << "IMU driver error" << std::endl;
        return fatal(DRIVER_ERROR);
    }
}

// void Task::errorHook()
// {
// }
// void Task::stopHook()
// {
// }

void Task::cleanupHook()
{
    getFileDescriptorActivity()->unwatch(m_driver->getFileHandle());
    m_driver->close();
    delete timestamp_estimator;
    timestamp_estimator = 0;
}

