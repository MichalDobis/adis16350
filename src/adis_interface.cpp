#include <adis16350/adis_interface.h>

AdisInterface::AdisInterface(){

	active = false;
	SCALE_AKCEL = 0.002522 * GRAVITY;
	SCALE_GYRO = 0.07326;
}

//init komunikacie
bool AdisInterface::init(std::string port, int baud){

	if (!active){
		try{
		my_serial = new serial::Serial(port, baud, serial::Timeout::simpleTimeout(100));

		}catch (std::exception& e){
			ROS_ERROR("ADIS16350: port don't open: %s", e.what());
			active = false;
			return false;
		}
	}

	active = true;

	return true;
}

//register pre filter a rozsah
bool AdisInterface::setSENS_AVG(uint8_t SENS, uint8_t AVG){

	mutex.lock();

	bool success = writeRegister(SENS_AVG,SENS, AVG);

	mutex.unlock();
	if (!success)
		ROS_ERROR("ADIS16350: neuspesny init, zapis do SENS_AVG reg");

	return success;
}

//register pre frekvenciu vzorkovania v adise
bool AdisInterface::setSMPL_PRD(uint8_t SMPL){

	mutex.lock();
	bool success = writeRegister(SMPL_PRD,0x00, SMPL);

	mutex.unlock();
	if (!success)
		ROS_ERROR("ADIS16350: neuspesny init, zapis do SMPL_PRD reg");

	return success;
}

//register msc_ctrl
bool AdisInterface::setMSC_CTRL(uint8_t MSC){

	mutex.lock();
	bool success = writeRegister(MSC_CTRL,0x00, MSC);

	mutex.unlock();
	if (!success)
		ROS_ERROR("ADIS16350: neuspesny init, zapis do MSC_CTRL reg");

	return success;
}

//citanie statusu
uint16_t AdisInterface::getStatus(STATUS_STRUCT *status){

	uint8_t data[] = {0x00,0x00};
	if (!calibrationRunning.try_lock())
	     return 0;

	mutex.lock();
	if (!readRegister(STATUS,data)){
		calibrationRunning.unlock();
		mutex.unlock();
		return 0;
	}
	calibrationRunning.unlock();
	mutex.unlock();

	status->alarm1 =  ((data[0] & 1) ? true : false);
	status->alarm2 =  ((data[0] & 2) ? true : false);
	status->gyro_err.x =  ((data[0] & 4) ? true : false);
	status->gyro_err.y =  ((data[0] & 8) ? true : false);
	status->gyro_err.z =  ((data[0] & 16) ? true : false);
	status->akcel_err.x =  ((data[0] & 32) ? true : false);
	status->akcel_err.y =  ((data[0] & 64) ? true : false);
	status->akcel_err.z =  ((data[0] & 128) ? true : false);

	status->power_supply_above =  ((data[1] & 1) ? true : false);
	status->power_supply_below =  ((data[1] & 2) ? true : false);
	status->ctr_update_err =  ((data[1] & 4) ? true : false);
	status->spi_err =  ((data[1] & 8) ? true : false);
	status->over_range =  ((data[1] & 16) ? true : false);
	status->self_test_err =  ((data[1] & 32) ? true : false);

	return ((uint16_t)data[0] << 8) | data[1];
}

//citanie lubovolneho registra
uint16_t AdisInterface::readRegister(uint8_t reg){

	uint8_t read_data[] = {0x00, 0x00};

	mutex.lock();
	if (!readRegister(reg,read_data)){
		mutex.unlock();
		return 0;
	}

	mutex.unlock();
	return  ((uint16_t)read_data[0] << 8) | read_data[1];
}

//zapis do lubovolneho registra
bool AdisInterface::writeRegister(uint8_t reg, uint16_t data){

	mutex.lock();
	bool success = writeRegister(reg, (uint8_t)(data >> 8),(uint8_t)(data & 0x00FF));
	mutex.unlock();

	return success;
}


//zmaze offsety z registrov
bool AdisInterface::restoringCalibration(){

	mutex.lock();
	bool success = writeRegister(COMMAND, 0x00, 0x02);
	mutex.unlock();

	if (!success)
		ROS_ERROR("ADIS16350: neuspesne mazanie offset registrov");
	return success;
}

//kratka kalibracia
bool AdisInterface::autoCalibrate(){

	calibrationRunning.lock();
	mutex.lock();
	ROS_WARN("ADIS16350: prebieha kratka kalibracia nehybat 2 sekundy zo zoriadenim");
	bool success = writeRegister(COMMAND, 0x00, 0x01);
	sleep(2);
	calibrationRunning.unlock();
	mutex.unlock();

	if (!success)
		ROS_ERROR("ADIS16350: neuspesne auto kalibracia");
	else ROS_INFO("ADIS16350: kratka kalibracia uspesne dokoncena");
	return success;
}

//dlha kalibracia
bool AdisInterface::calibrate(){

	calibrationRunning.lock();
	mutex.lock();
	ROS_WARN("ADIS16350: prebieha kalibracia nehybat 30 sekund zo zoriadenim");
	sleep(2);
	bool success = writeRegister(COMMAND, 0x00, 0x10);
	sleep(30);
	calibrationRunning.unlock();
	mutex.unlock();

	if (!success)
		ROS_ERROR("ADIS16350: neuspesna kalibracia");
	else ROS_INFO("ADIS16350: kalibracia uspesne dokoncena");
	return success;
}

//precita offset z registra GYRO_Z, zrata 2000 vzoriek a vypocita priemer, podla ktoreho zmeni offset v GYRO_Z
bool AdisInterface::calibrationGyroZ(){

	ROS_WARN("ADIS16350: prebieha kalibracia nehybat 30 sekund zo zoriadenim");
	sleep(2);
	uint16_t offset = readRegister(ZGYRO_OFF);
	sensor_msgs::Imu imu;
	double z = 0;
	int i = 0;
	calibrationRunning.lock();
	mutex.lock();
	for (i = 0; i < 2000; i++){
		if (readAxes(&imu))
			z += imu.angular_velocity.z;
		else i--;
	}

	z /= i;
	z /= 0.018315;  //Datasheet: Scale = 0.018315°/s per LSB
	offset -= z;

	bool success = true;
	if (!writeRegister(ZGYRO_OFF, (uint8_t)(offset >> 8),(uint8_t)(offset & 0x00FF))){
		success = false;
		ROS_ERROR("ADIS16350: neuspesny zapis do registru ZGYRO_OFF");
	} else
		ROS_INFO("ADIS16350: kratka kalibracia uspesne dokoncena");

	calibrationRunning.unlock();
	mutex.unlock();
	return success;

}

//vracia cez smernik IMU data
bool AdisInterface::getImuData(sensor_msgs::Imu *imu){

	if (!mutex.try_lock())
        return false;

	bool success = readAxes(imu);
	mutex.unlock();
	return success;

}

bool AdisInterface::isActive(){
	return active;
}

int AdisInterface::getRange(){
	return range;
}

//malo by zistovat teplotu, ale nefunguje
geometry_msgs::Vector3 AdisInterface::getTemperature(){

	geometry_msgs::Vector3 temperature;
	uint8_t write_data = 0x04;
	uint8_t read_data[12];

	if (active){

		mutex.lock();

		try{
			my_serial->flush();
			int count  = my_serial->write(&write_data,1);

				if (count != 1){

					mutex.unlock();
					ROS_ERROR("ADIS16350: read temperature: zly pocet zapisanych bytov %d", count);
					return temperature;
				}
				count = my_serial->read(read_data,6);

				if (count != 6){

					mutex.unlock();
					ROS_ERROR("ADIS16350: read temperature: zly pocet precitanych bytov %d", count);
					return temperature;
				}

		}catch (std::exception& e){
			mutex.unlock();
			ROS_ERROR("ADIS16350: exception: %s",e.what());
			active = false;
			return temperature;
			}
	}
	mutex.unlock();

	temperature.x = 0.1453*TWOCOMP12((((read_data[0] << 8) | read_data[1]) & 0x0FFF));
	temperature.y = 0.1453*TWOCOMP12((((read_data[2] << 8) | read_data[3]) & 0x0FFF));
	temperature.z = 0.1453*TWOCOMP12((((read_data[4] << 8) | read_data[5]) & 0x0FFF));

	ROS_ERROR("teplota X: %.2f\n",temperature.x);
	ROS_ERROR("teplota y: %.2f\n",temperature.y);
	ROS_ERROR("teplota z: %.2f\n\n",temperature.z);

	return temperature;
}

			/////////////////////
			//PRIVATE FUNCTIONS//
			////////////////////

bool AdisInterface::readAxes(sensor_msgs::Imu *imu){

	if (!active)
		return false;

	static uint8_t read_data[12];
	 static uint8_t write_data = 0x03;

	 try{

		my_serial->flush();
		int count  = my_serial->write(&write_data,1);

		if (count != 1){

			//ROS_ERROR("ADIS16350: read axes: zly pocet zapisanych bytov %d", count);
			return false;
		}
		count = my_serial->read(read_data,12);

		if (count != 12){

			//ROS_ERROR("ADIS16350: read axes: zly pocet precitanych bytov %d", count);
			return false;
		}

	}catch (std::exception& e){

		ROS_ERROR("ADIS16350: exception: %s",e.what());
		active = false;
		my_serial->close();
		return false;
		}

	imu->header.stamp = ros::Time::now();

			 //konverzia precitanych udajov
	imu->angular_velocity.x = SCALE_GYRO * TWOCOMP14((((read_data[0] << 8) | read_data[1]) & 0x3FFF));
	imu->angular_velocity.y = SCALE_GYRO * TWOCOMP14((((read_data[2] << 8) | read_data[3]) & 0x3FFF));
	imu->angular_velocity.z = SCALE_GYRO * TWOCOMP14((((read_data[4] << 8) | read_data[5]) & 0x3FFF));

	imu->linear_acceleration.x = SCALE_AKCEL *TWOCOMP14((((read_data[6] << 8) | read_data[7]) & 0x3FFF));
	imu->linear_acceleration.y = -1 * SCALE_AKCEL * TWOCOMP14((((read_data[8] << 8) | read_data[9]) & 0x3FFF));
	imu->linear_acceleration.z = -1 * SCALE_AKCEL * TWOCOMP14((((read_data[10] << 8) | read_data[11]) & 0x3FFF));

	return true;
}

bool AdisInterface::writeRegister(uint8_t reg, uint8_t hi, uint8_t lo){

	if (!active)
		return false;

	uint8_t command[] = {0x01, reg, hi, lo};
	bool success = false;

	try{

		my_serial->flush();
		if (my_serial->write(command,4) < 4)
			success = false;
		else
		success = checkRegister(reg, &command[2]);

	}catch (std::exception& e){

		ROS_ERROR("ADIS16350: exception: %s",e.what());
		my_serial->close();
		active = false;
		success = false;
		}

		if (!success)
			ROS_ERROR("ADIS16350: neuspesny init, zapis do registra %X", reg);

	return success;
}

bool AdisInterface::readRegister(uint8_t reg, uint8_t *read_data){

	if (!active)
		return 0;

	uint8_t command[] = {0x02,reg};
	try{
		my_serial->flush();
		int count  = my_serial->write(command,2);

		if (count != 2){
				ROS_ERROR("ADIS16350: read_register: zly pocet zapisanych bytov %d", count);
				return false;
		}
		count = my_serial->read(read_data,2);

		if (count != 2){
			ROS_ERROR("ADIS16350: read_register: zly pocet precitanych bytov %d", count);
			return false;
		}
		else
			ROS_INFO("register 0x%X = 0x%X%X",reg, read_data[0],read_data[1]);

		}catch (std::exception& e){

				active = false;
				my_serial->close();
				ROS_ERROR("ADIS16350: exception: %s",e.what());
				return false;
			}
		return true;
}

//kontrola ci, zadavane data na zapis boli naozaj zapisane
bool AdisInterface::checkRegister(uint8_t reg, uint8_t *controlled_data){

	uint8_t read_data[] = {0x00, 0x00};

	if (reg == COMMAND) //register 0x3E sa neda citat,  zapis musel byt uspesny
		{
			return true;
		}

	if (!readRegister(reg,read_data))
		return false;

	if (reg == SENS_AVG){
			SCALE_GYRO = 0.018315 * read_data[0];
			range = read_data[0];
	}

		else if ((read_data[0] != controlled_data[0]) || (read_data[1] != controlled_data[1])) //overenie ci je odpoved spravna
		{
			return false;
		}

	return true;
}
