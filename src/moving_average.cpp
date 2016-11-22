#include <adis16350/moving_average.h>

MovingAverage::MovingAverage(int buffer_size){
		size = buffer_size;
		gyro.resize(buffer_size);
		akcel.resize(buffer_size);

		for (int i = 0; i < buffer_size; i++){
			gyro[i].resize(3);
			akcel[i].resize(3);
		}
		average_counter = 0;
		start = false;
	}

	void MovingAverage::addSample(sensor_msgs::Imu imu){

		average_counter++;

		this->gyro[average_counter % size][0] = imu.angular_velocity.x;
		this->akcel[average_counter % size][0] = imu.linear_acceleration.x;
		this->gyro[average_counter % size][1] = imu.angular_velocity.y;
		this->akcel[average_counter % size][1] = imu.linear_acceleration.y;
		this->gyro[average_counter % size][2] = imu.angular_velocity.z;
		this->akcel[average_counter % size][2] = imu.linear_acceleration.z;

		if (average_counter > size)
			start = true;

}

	bool MovingAverage::computeAverage(sensor_msgs::Imu *imu){

		double gyro[3];
		double akcel[3];

		if (start){
			for(int i = 0; i < 3; i++){
				gyro[i] = 0;
				akcel[i] = 0;
			}

			for (int i = average_counter + 1; i <= average_counter + size - 2; i++)
				for(int j = 0; j < 3; j++){

					gyro[j] += this->gyro[i % size][j];
					akcel[j] += this->akcel[i % size][j];
				}


			for(int i = 0; i < 3; i++){
						gyro[i] += this->gyro[((uint8_t)(average_counter - 1)) % size][i] * 2;
						akcel[i] += this->akcel[((uint8_t)(average_counter - 1)) % size][i] * 2;

						gyro[i] += this->gyro[average_counter  % size][i] * 3;
						akcel[i] += this->akcel[average_counter % size][i] * 3;

						gyro[i] /= size + 3;
						akcel[i] /= size + 3;
					}
			arrayToImuMsg(gyro, akcel, imu);
			return true;
		}
		else return false;
	}

	void MovingAverage::arrayToImuMsg(double gyro[], double akcel[], sensor_msgs::Imu *imu){

			imu->angular_velocity.x = gyro[0];
			imu->angular_velocity.y = gyro[1];
			imu->angular_velocity.z = gyro[2];
			imu->linear_acceleration.x = akcel[0];
			imu->linear_acceleration.y = akcel[1];
			imu->linear_acceleration.z = akcel[2];

	}
