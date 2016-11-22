/**aplikovanie filtra plavajuci priemer**/
#include <stdio.h>
#include <sensor_msgs/Imu.h>
class MovingAverage{

public:
	MovingAverage(int buffer_size);			//constructor - parameter: buffer_size urcuje z kolkych vzoriek sa bude pocitat priemer
	void addSample(sensor_msgs::Imu imu);		//prida novu vzorku do filtra (najstarsia bude odstranena)
	bool computeAverage(sensor_msgs::Imu *imu); //vypocitaju priemer a ulozi do parametrov, false vracia pokial buffer este nie je naplneni



private:
	std::vector<std::vector<double> > gyro, akcel;	//buffer vzoriek merania

	uint8_t average_counter;							//pocitadlo
	int size;											//velkost buffera
	bool start;											//bude true ak sa naplni buffer, potom sa az moze zacat pocitat priemer

	void arrayToImuMsg(double gyro[], double akcel[], sensor_msgs::Imu *imu);
};
