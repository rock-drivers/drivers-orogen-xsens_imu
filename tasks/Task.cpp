#include "Task.hpp"

#include <rtt/extras/FileDescriptorActivity.hpp>
#include "xsens_imu/XsensDriver.hpp"
#include <aggregator/TimestampEstimator.hpp>


using namespace xsens_imu;

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
    timestamp_estimator = new aggregator::TimestampEstimator(
	base::Time::fromSeconds(20),
	base::Time::fromSeconds(1.0 / xsens_imu::XsensDriver::DEFAULT_SAMPLE_FREQUENCY),
	base::Time::fromSeconds(0),
	INT_MAX);

    std::auto_ptr<xsens_imu::XsensDriver> driver(new xsens_imu::XsensDriver());
    if( !driver->open( _port.value() ) ) {
        std::cerr << "Error opening device '" << _port.value() << "'" << std::endl;
        return false;
    }

    if( !driver->setSyncOut(true, true, true) ) {
        std::cerr << "Error configuring SyncOut pin";
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

    m_driver = driver.release();
    return true;
}

bool Task::startHook()
{
    RTT::extras::FileDescriptorActivity* fd_activity =
        getActivity<RTT::extras::FileDescriptorActivity>();
    if (fd_activity)
    {
        fd_activity->watch(m_driver->getFileHandle());
        fd_activity->setTimeout(_timeout);
    }

    // Clear the IO buffers, to avoid sending really old data as our first
    // samples
    m_driver->setTimeout(500 / xsens_imu::XsensDriver::DEFAULT_SAMPLE_FREQUENCY);
    int retval;
    do
    {
	retval = m_driver->getReading();
    }
    while(retval != xsens_imu::ERROR_TIMEOUT);
    m_driver->setTimeout(_timeout);

    timestamp_estimator->reset();

    return true;
}

void Task::updateHook()
{
    // since this task is fd driven, we assume that the reception of data on
    // the port is the closest we can get to the actual acquisition time. This
    // can later be fixed to using a hardware sync-out from the IMU.

    xsens_imu::errorCodes retval;
    retval = m_driver->getReading();

    if( retval == xsens_imu::NO_ERROR ) {
	base::Time recvts = base::Time::now();

	int packet_counter = m_driver->getPacketCounter();

	base::Time ts = timestamp_estimator->update(recvts,packet_counter);
        timeout_counter = 0;

	base::samples::RigidBodyState reading;
	reading.time = ts;
	reading.sourceFrame = _imu_frame.value();
	reading.targetFrame = _world_frame.value();
	reading.orientation = m_driver->getOrientation();
        _orientation_samples.write( reading );

        base::samples::IMUSensors sensors;
        sensors.time = ts;
	sensors.acc   = m_driver->getCalibratedAccData();
	sensors.gyro  = m_driver->getCalibratedGyroData();
	sensors.mag   = m_driver->getCalibratedMagData();
	_calibrated_sensors.write( sensors );
	
	_timestamp_estimator_status.write(timestamp_estimator->getStatus());
   } 

    if( retval == xsens_imu::ERROR_TIMEOUT ) {
        timeout_counter++;

        if( timeout_counter >= _max_timeouts ) {
            std::cerr << "IMU driver timout." << std::endl;
            return exception(IO_ERROR);
        }
   }

    if( retval == xsens_imu::ERROR_OTHER ) {
        std::cerr << "IMU driver error" << std::endl;
        return exception(DRIVER_ERROR);
    }

    base::Time hard_ts;

    while (_hard_timestamps.read(hard_ts) == RTT::NewData)
	timestamp_estimator->updateReference(hard_ts);
}

// void Task::errorHook()
// {
// }
void Task::stopHook()
{
    RTT::extras::FileDescriptorActivity* fd_activity =
        getActivity<RTT::extras::FileDescriptorActivity>();
    if (fd_activity)
    {
        fd_activity->clearAllWatches();
        //set timeout back so we don't timeout on the rtt's pipe
        fd_activity->setTimeout(0);
    }
}

void Task::cleanupHook()
{
    m_driver->close();
    delete m_driver;
    m_driver = 0;
    delete timestamp_estimator;
    timestamp_estimator = 0;
}

