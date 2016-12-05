#include <ros/ros.h>
#include <adis16350/mathematics_operations.h>
#include <adis16350/adis_interface.h>
#include <adis16350/moving_average.h>

#include <adis16350/Write.h>
#include <adis16350/Read.h>
#include <std_srvs/Trigger.h>

#include <diagnostic_updater/diagnostic_updater.h>

class Adis16350: public MovingAverage{
public:
	Adis16350() : MovingAverage(10), diagnostic_( ){

		ros::NodeHandle n;
		std::string port;
		int baud;
		std::string topicName;
		double usingGyro, usingAkcel;
		double status_period;

		int statusCode = 1;
		bool runningCalibration = false;

		n.param<std::string>("port_name", port, "/dev/ttyUSB0");
		n.param<int>("baudrate", baud, 230400);
		//n.param<int>("stop_bits", stopBits, 1);
		//n.param<int>("parity", parity, 1);
		//n.param<int>("byte_size", byteSize, 8);
		n.param<bool>("use_complementary_filter", useComplementary, false);
		n.param<bool>("use_moving_average", useMovingAverage, false);
		n.param<std::string>("data_topic_name", topicName, "imu_data");
		n.param<double>("using_gyro", usingGyro, 0.98);
		n.param<double>("using_akcel", usingAkcel, 0.02);
		n.param<double>("status_period", status_period, 1);

		diagnostic_.add("Wsg_50 Status", this, &Adis16350::diagnostics);
		diagnostic_.setHardwareID("none");

		imuPub = n.advertise<sensor_msgs::Imu>(topicName,100);
		imuPubTest = n.advertise<sensor_msgs::Imu>("test",100);

		serviceGetAngleZ = n.advertiseService("get_angle", &Adis16350::getAngleZSrv, this);
		serviceInit = n.advertiseService("init", &Adis16350::initSrv, this);
		serviceWrite = n.advertiseService("write", &Adis16350::writeSrv, this);
		serviceRead = n.advertiseService("read", &Adis16350::readSrv, this);
		serviceCalibrate = n.advertiseService("calibrate", &Adis16350::calibrateSrv, this);
		serviceAutoCalibrate = n.advertiseService("auto_calibrate", &Adis16350::autoCalibrateSrv, this);
		serviceZGyroCalibrate = n.advertiseService("z_gyro_calibrate", &Adis16350::zGyroCalibrateSrv, this);
		serviceRestoringCalibrate = n.advertiseService("restoring_calibrate",&Adis16350::restoringCalibrationSrv, this);

		adis = new AdisInterface();
		ROS_WARN("adis sa bude nastavovat idealne ho umiestnit rovnobezne so zemou, nehybat");

		operations = new MathematicsOperations();

		n.param<double>("status_period", status_period, 1);

		std::vector <double> pose_covariance_diagonal;
		std::vector <double> angular_velocity_covariance_diagonal;
		std::vector <double> linear_acceleration_covariance_diagonal;

		n.getParam("pose_covariance_diagonal", pose_covariance_diagonal);
		n.getParam("angular_velocity_covariance_diagonal", angular_velocity_covariance_diagonal);
		n.getParam("linear_acceleration_covariance_diagonal", linear_acceleration_covariance_diagonal);

		operations->createCovarianceMatrix(&imu, pose_covariance_diagonal, angular_velocity_covariance_diagonal, linear_acceleration_covariance_diagonal);
	    operations->setComplementaryFilterParams(usingGyro, usingAkcel);

    	status_timer = n.createTimer(ros::Duration(status_period), &Adis16350::timerCallback, this); //citanie prebieha periodickym spustanim casovaca

		if (!adis->init(port, baud)){
			ROS_ERROR("ADIS16350: adis port not open");
			return;
		}
			initControllRegisters();
	}

	void readAndPublish(){

		adis->getImuData(&imu);

		imuWithoutAverage = imu;
		imuPubTest.publish(imuWithoutAverage);

		if (useMovingAverage){				//ak sa ma pocitat plavajuci priemer, tak sa musi vytvorit buffer vzoriek
				addSample(imu);

				if (!computeAverage(&imu))//funkcia zistuje ci je uz buffer vyplneny, ak nie je nemoze sa pocitat priemer, preto retun;
					return;
			}

		vector = operations->computeOrientationFromGyro(&imu);

		if (useComplementary)
			operations->computeComplementary(imu);

		imu.orientation = operations->createQuaternion();
		imuPub.publish(imu);
	}

private:

	AdisInterface *adis;
	MathematicsOperations *operations;
	sensor_msgs::Imu imu;
	sensor_msgs::Imu imuWithoutAverage;

	geometry_msgs::Vector3 vector;

	bool useComplementary;
	bool useMovingAverage;

	int statusCode;
	bool runningCalibration;
	boost::mutex mutex;

	ros::ServiceServer serviceGetAngleZ;
	ros::ServiceServer serviceZGyroCalibrate;
	ros::ServiceServer serviceInit;
	ros::ServiceServer serviceWrite;
	ros::ServiceServer serviceRead;
	ros::ServiceServer serviceCalibrate;
	ros::ServiceServer serviceAutoCalibrate;
	ros::ServiceServer serviceRestoringCalibrate;

	ros::Publisher imuPub;
	ros::Publisher imuPubTest;

	ros::Timer status_timer;
	AdisInterface::STATUS_STRUCT status;
	diagnostic_updater::Updater diagnostic_;

	void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat){

		if (adis->isActive()){

			 stat.summary(0, "Connected");

			 if (runningCalibration)
				 stat.add<bool>("calibration is running", true);

			 int gyroRange = 300;
			 switch (adis->getRange()){
			 case 4: gyroRange = 300;
			 break;
			 case 2: gyroRange = 150;
			 break;
			 case 1: gyroRange = 75;

			 }
			 stat.add<int>("gyro_range", gyroRange);
			 stat.add<int>("status_code",statusCode);

			 if (status.alarm1)
				 stat.add<bool>("active_alarm_1", true);

			 if (status.alarm2)
				 stat.add<bool>("active_alarm_2", true);

			 if (status.gyro_err.x || status.gyro_err.y || status.gyro_err.z)
				 stat.add<std::string>("gyro err", "gyroscope self-diagnostic error flag");

			 if (status.akcel_err.x || status.akcel_err.y || status.akcel_err.z)
				stat.add<std::string>("akcel err", "accelerometer self-diagnostic error flag");


			 if (status.power_supply_above)
				 stat.add<std::string>("power supply above", "5.25V");

			 if (status.power_supply_below)
				stat.add<std::string>("power supply below", "4.75V");

			 if (status.ctr_update_err)
				stat.add<std::string>("control register", "update failed");

			 if (status.spi_err)
				 stat.add<std::string>("SPI communications", "failure");

			 if (status.over_range)
				stat.add<bool>("sensor over range", true);

			 if (status.self_test_err)
				stat.add<std::string>("Self-test diagnostic", "error flag");

		}else{
			stat.summary(2, "Disconnected");
			stat.add<std::string>("status", "disconnected");

		}
	}

	bool initControllRegisters(){

		ros::NodeHandle n;

		bool enable_write_to_registers;
		n.param<int>("enable_write_to_registers", enable_write_to_registers);

		if (!enable_write_to_registers)
			return false;

		int avg, range, smpl, msc, calibration_type;
		n.param<int>("digital_filtering", avg, 6);
		n.param<int>("range", range, 3);
		n.param<int>("sample_rate", smpl, 1);
		n.param<int>("msc_ctrl", msc, 0);
		n.param<int>("calibration_type", calibration_type, 1);


		if (avg > 6)
			avg = 6;
		if (smpl > 15)
			smpl = 15;

		if (range > 2)	//pre 1 ma byt rozsah 75°/s a ma sa zapisovat 0x01,
						//pre 2 ma byt rozsah 150°/s a ma sa zapisovat 0x02,
			range = 4;  //pre 3 ma byt rozsah 300°/s a ma sa zapisovat 0x04

		if (msc > 3)
			msc = 3;

		msc <<= 2;		//vyposuvat bity, tak aby sa zapisal 6. a 7. bit. napr. pre hodnotu 3 sa zapise 0xC0
						//[7] Linear acceleration bias compensation for gyroscopes
						//1 = enabled, 0 = disabled
						//[6] Linear accelerometer origin alignment
						//1 = enabled, 0 = disabled

		if (calibration_type < 0)
			adis->writeRegister(ZGYRO_OFF, 0);

		else{
				runningCalibration = true;
				diagnostic_.force_update();

				if (calibration_type == 0)
					sleep(2);
				if (calibration_type & 0x01)
					adis->autoCalibrate();

				if (calibration_type & 0x02){
					adis->writeRegister(ZGYRO_OFF, 0);
					adis->calibrate();
				}
				if (calibration_type & 0x04)
					adis->calibrationGyroZ();

				runningCalibration = false;
				diagnostic_.force_update();
		}

		if (!adis->setSENS_AVG(uint8_t (range), uint8_t(avg))){
			ROS_ERROR("ADIS16350: initial failed in init registers");
			return false;
		}

		if (!adis->setSMPL_PRD(uint8_t(smpl))){
			ROS_ERROR("ADIS16350: initial failed in init registers");
			return false;
		}
		if (!adis->setMSC_CTRL(uint8_t(msc))){
			ROS_ERROR("ADIS16350: initial failed in init registers");
			return false;
		}
		return true;
	}

	bool initSrv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){

		ROS_WARN("adis sa bude nastavovat idealne ho umiestnit rovnobezne so zemou, nehybat");
		ros::NodeHandle n;
		std::string port;
		int baud;

		n.param<std::string>("port_name", port, "/dev/ttyUSB0");
		n.param<int>("baudrate", baud, 230400);

		if (!adis->init(port, baud)){
			res.success = false;
			res.message = "initial failed in open port";
			return true;
		}

		//adis->autoCalibrate();
		//sleep(1);
		if (initControllRegisters()){
			res.success = true;
			res.message = "initial complete";
		}else{
			res.success = false;
			res.message = "initial failed in init registers";
		}
		return true;
	}

	bool writeSrv(adis16350::Write::Request  &req, adis16350::Write::Response &res){

		if (adis->writeRegister(req.reg, req.data)){
			res.success = true;
			res.message = "write to register was successfully";
		} else{
			res.success = false;
			res.message = "write failed";
		}

		return true;
	}

	bool readSrv(adis16350::Read::Request  &req, adis16350::Read::Response &res){

		res.data = adis->readRegister(req.reg);
		return true;
	}

	bool calibrateSrv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){

		runningCalibration = true;
		diagnostic_.force_update();

		if (adis->calibrate()){
			res.success = true;
			res.message = "calibration complete succesfully";
		} else{
			res.success = false;
			res.message = "calibration failed";
		}

		runningCalibration = false;
		diagnostic_.force_update();
		return true;
	}

	bool autoCalibrateSrv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){

		runningCalibration = true;
		diagnostic_.force_update();

		if (adis->autoCalibrate()){
			res.success = true;
			res.message = "calibration complete successfully";
		} else{
			res.success = false;
			res.message = "calibration failed";
		}

		runningCalibration = false;
		diagnostic_.force_update();
		return true;
	}

	bool zGyroCalibrateSrv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){

		runningCalibration = true;
		diagnostic_.force_update();

		adis->calibrationGyroZ();

		runningCalibration = false;
		diagnostic_.force_update();

		return true;
	}


	bool restoringCalibrationSrv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){

		if (adis->restoringCalibration()){
			res.success = true;
			res.message = "restoring calibration complete successfully";
		} else{
			res.success = false;
			res.message = "restoring calibration failed";
		}

		return true;
	}

	void timerCallback(const ros::TimerEvent&){

		//adis->getTemperature();
		statusCode = adis->getStatus(&status);
		diagnostic_.update();
	}

	bool getAngleZSrv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){

	ROS_ERROR("angle Z %f",operations->getOrientation().z);
			res.success = true;

		return true;
	}
};


int main(int argc, char **argv)
{
	//node init
	ros::init(argc, argv, "adis16350");
	ros::NodeHandle n;
	sleep(1);

    Adis16350 adis;

    int publish_rate;
     n.param<int>("publish_rate", publish_rate, 10);

	//start spinner (pre servisi)
	ros::AsyncSpinner spinner(1);

    spinner.start();

	//loop rate
	ros::Rate rate(publish_rate);
  while (n.ok())
  {
		adis.readAndPublish();
     rate.sleep();
  }
return 0;}
