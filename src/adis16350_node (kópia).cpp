#include <string>
#include <ros/ros.h>

//#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>

#include <ros/callback_queue.h>
#include <sensor_msgs/Imu.h>
//#include <geometry_msgs/Quaternion.h>
#include "serial/serial.h"
#include <math.h>
#include <stdlib.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Float32.h>
#include "adis16350/Write.h"
#include "adis16350/Read.h"
#define G 9.80665
#define TWOCOMP14(x) (((int16_t)(x)<0x2000)?(int16_t)(x):-(0x4000-(int16_t)(x)))
#define TWOCOMP12(x) (((int16_t)(x)<0x0800)?(int16_t)(x):-(0x1000-(int16_t)(x)))

#define SUPPLY_OUT	0x02
#define XGYRO_OUT	0x04
#define YGYRO_OUT	0x06
#define ZGYRO_OUT	0x08
#define XACCL_OUT	0x0A
#define YACCL_OUT	0x0C
#define ZACCL_OUT	0x0E
#define XTEMP_OUT	0x10
#define YTEMP_OUT	0x12
#define ZTEMP_OUT	0x14
#define AUX_ADC		0x16

#define ENDURANCE	0x00
#define XGYRO_OFF	0x1A
#define YGYRO_OFF	0x1C
#define ZGYRO_OFF	0x1E
#define XACCL_OFF	0x20
#define YACCL_OFF	0x22
#define ZACCL_OFF	0x24
#define ALM_MAG1	0x26
#define ALM_MAG2	0x28
#define ALM_SMPL1	0x2A
#define ALM_SMPL2	0x2C
#define ALM_CTRL	0x2E
#define AUX_DAC		0x30
#define GPIO_CTRL	0x32
#define MSC_CTRL	0x34
#define SMPL_PRD	0x36
#define SENS_AVG	0x38
#define SLP_CNT		0x3A
#define STATUS		0x3C
#define COMMAND		0x3E

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

ros::Publisher imu_pub;
ros::Publisher imu_pub2;
ros::Publisher imu_pubc;
ros::Time current_time;
ros::Time last_time;
serial::Serial* my_serial;

pthread_mutex_t RS_422_mutex = PTHREAD_MUTEX_INITIALIZER;

//citane, zpracovane a publikovane data
double gyro[] = {0,0,0};
double akcel[] = {0,0,0};
double uhol[] = {0,0,0};

//polia pre vzorky do priemer filtra
double xgyro[10];
double ygyro[10];
double zgyro[10];

double xakcel[10];
double yakcel[10];
double zakcel[10];

//parametre
bool complementary_filter = true;
bool priemer_filter = false;
bool filter_g = false;
bool temperature = false;
bool isCalibrate = false;
std::string frame_id = "imu";
double range = 1;
int reading_period = 10;

 std::string port = "/dev/ttyUSB0";
int baud=230400;

/*void clearBuffer()
{
 if (my_serial->available())
  {
  	ROS_ERROR("adis: cistim buffer");
  	uint8_t cistenie;
  	while (my_serial->read(&cistenie,1))
  		ROS_ERROR("adis: vycistene %x",cistenie);
  }
}*/

void init()
{
	//zapis rozlisenie a filter (rozlizenie nefunguje)
	uint8_t command[] = {0x01,SENS_AVG,0x04,0x04};
	if (my_serial->write(command,4) < 4)
		ROS_ERROR("adis16350: neuspesny init, zapis do SENS_AVG reg");
		
	command[1] = SMPL_PRD;
	command[2] = 0x00;
	command[3] = 0x01;
	if (my_serial->write(command,4) < 4)
		ROS_ERROR("adis16350: neuspesny init, zapis do SMPL_PRD reg");
	
	command[1] = MSC_CTRL;
	command[2] = 0x00;
	command[3] = 0xC0;
	if (my_serial->write(command,4) < 4)
		ROS_ERROR("adis16350: neuspesny init, zapis do MSC_CTRL reg");
	
}

void setROSParam()
{
	ros::param::get("adis16350/complementary_filter",complementary_filter);
	ros::param::get("adis16350/filter_G",filter_g);
	ros::param::get("adis16350/priemer_filter",priemer_filter);
	ros::param::get("adis16350/reading_period",reading_period);
	ros::param::get("adis16350/is_calibrate",isCalibrate);
	ros::param::get("adis16350/frame_id",frame_id);
	ros::param::get("adis16350/baud",baud);
	ros::param::get("adis16350/port",port);
	
	ros::param::set("adis16350/complementary_filter",complementary_filter);
	ros::param::set("adis16350/filter_G",filter_g);
	ros::param::set("adis16350/priemer_filter",priemer_filter);
	ros::param::set("adis16350/reading_period",reading_period);
	ros::param::set("adis16350/is_calibrate",isCalibrate);
	ros::param::set("adis16350/frame_id",frame_id);
	ros::param::set("adis16350/range",300);
}

//nastavi zaciatocnu poziciu
void setRotation()
{
	uint8_t write_data = 0x03;
	uint8_t read_data[12];	
	double uhol_pom = 0;
	akcel[0] = 0.0;
	akcel[1] = 0.0;
	akcel[2] = 0.0;
	int i=0;
	int lenght=200;
	
	//citanie udajov a scitavanie na priemer
	for (i;i<lenght;i++)
	{
		my_serial->write(&write_data,1);
	    	my_serial->read(read_data,12);
	    	 
	    		
	    	gyro[0] = -0.07326 * range *TWOCOMP14((((read_data[0] << 8) | read_data[1]) & 0x3FFF));
	    	gyro[1] = -0.07326 * range * TWOCOMP14((((read_data[2] << 8) | read_data[3]) & 0x3FFF));
	    	gyro[2] = 0.07326 * range * TWOCOMP14((((read_data[4] << 8) | read_data[5]) & 0x3FFF));
	    		
	    	akcel[0] += -0.002522*G*TWOCOMP14((((read_data[6] << 8) | read_data[7]) & 0x3FFF));
	    	akcel[1] += 0.002522*G*TWOCOMP14((((read_data[8] << 8) | read_data[9]) & 0x3FFF));
	    	akcel[2] += -0.002522*G*TWOCOMP14((((read_data[10] << 8) | read_data[11]) & 0x3FFF));
	    	
	    	//data pre priemer filter
	    	if  (i >= lenght - 10)
	    	{
	    		
	    		xgyro[i - (lenght -10)] = gyro[0];
	    		ygyro[i - (lenght -10)] = gyro[1];
	    		zgyro[i - (lenght -10)] = gyro[2];
	    		
	    		xakcel[i - (lenght -10)] = akcel[0];
	    		yakcel[i - (lenght -10)] = akcel[1];
	    		zakcel[i - (lenght -10)] = akcel[2];
		}    		
	}
	
	//priemer
	for (i=0;i<3;i++)
		akcel[i]/=lenght;
		
	ROS_WARN("\n\n\nzrychlenie pre offsets \nX: %.2f m/s^2\nY: %.2f m/s^2\nZ: %.2f m/s^2\n\n\n",akcel[0],akcel[1],
	   akcel[2]);
	    	
	double velkost = sqrt(pow(akcel[0],2) + pow(akcel[1],2) + pow(akcel[2],2));
	ROS_WARN("velkost tiaze %.2f",velkost);
	
	//uhly vzhladom na G 	
	uhol[0] =  asin(akcel[1]/velkost);
	uhol[1]=  asin(akcel[0]/velkost);   
	ROS_WARN("\nuhol x je: %4.2f\nuhol y je: %4.2f\nuhol z je: %4.2f\n",uhol[0],uhol[1],uhol[2]);
}

void complementaryFilter()
{
	
	double velkost = sqrt(pow(akcel[0],2) + pow(akcel[1],2) + pow(akcel[2],2));	
	uhol[0] = 0.98 * uhol[0] + 0.02  * asin(akcel[1]/velkost);
	uhol[1] = 0.98 * uhol[1] + 0.02  * asin(akcel[0]/velkost);
	
	//uhol[0] = 0.98 * uhol[0] + 0.02 * (atan2(akcel[1], akcel[2]) * 180 / M_PI);
	//uhol[1] = 0.98 * uhol[1] + 0.02 * (atan2(akcel[0], akcel[2]) * 180 / M_PI);
}


double pocitaj_priemer(double *data, double new_data)
{
	double max = -1000;	
	double min = 1000;
	int max_pos = 0;
	int min_pos = 0;
	double result = 0;
	for (int i = 0; i < 9; i++)
	{
		data[i] = data [i + 1];
		if (data[i] < min) 
		{
			min = data[i];
			min_pos = i;
		}
		if (data[i] > max) 
		{
			max = data[i];
			max_pos = i;
		}
	}
	data[9] = new_data;
	if (data[9] < min) 
		{
			min = data[9];
			min_pos = 9;
		}
		if (data[9] > max) 
		{
			max = data[9];
			max_pos = 9;
		}
		
		for (int i = 0; i < 10; i++)
			if  ((i != max_pos) && (i != min_pos))
				result += data[i];
		return result /= 8;

}

void priemerFilter()
{
		gyro[0] = pocitaj_priemer(xgyro,gyro[0]);
		gyro[1] = pocitaj_priemer(ygyro,gyro[1]);
		gyro[2] = pocitaj_priemer(zgyro,gyro[2]);
		akcel[0] = pocitaj_priemer(xakcel,akcel[0]);
		akcel[1] = pocitaj_priemer(yakcel,akcel[1]);
		akcel[2] = pocitaj_priemer(zakcel,akcel[2]);
}

//odstraneni tiaze z akcelerometra
void filterG()
{
	  double x_g = G * sin(uhol[1]);
	  double y_g = G * sin(uhol[0]);
	  double yx = pow(x_g,2.0) + pow(y_g,2.0);
	  if (yx > pow(G,2.0))
	  	yx = G; 
	  double z_g = sqrt(pow(G,2.0) - yx);
	  akcel[0] -= x_g;
	  akcel[1] -= y_g;
	 akcel[2] = z_g - akcel[2];
}

//citanie zakladnych udajov otacanie, zrychlenie
void read_adis(const ros::TimerEvent&)
{
	//if (!isCalibrate) //ak neni nakalibrovany tak z neho necitaj
	//	return;
	if (pthread_mutex_trylock(&RS_422_mutex))
                return;
	//pthread_mutex_lock(&RS_422_mutex);
	//ROS_INFO("start timer");
	static uint8_t read_data[12];
	 static sensor_msgs::Imu msg;
	 static uint8_t write_data = 0x03;
	 //ROS_INFO("alokacia");
	// clearBuffer();
	my_serial->flush();
	my_serial->write(&write_data,1);
	my_serial->read(read_data,12);
	  //ROS_INFO("citanie");
	 //priprava hlavicky spravy   		
	 current_time = ros::Time::now();
	 double dt = (current_time - last_time).toSec();
	 last_time = current_time;	  
	 msg.header.stamp = current_time;
	 
	 //konverzia precitanych udajov
	 gyro[0] = 0.07326 * range * TWOCOMP14((((read_data[0] << 8) | read_data[1]) & 0x3FFF));
	 gyro[1] = 0.07326 * range * TWOCOMP14((((read_data[2] << 8) | read_data[3]) & 0x3FFF));
	 gyro[2] = 0.07326 * range * TWOCOMP14((((read_data[4] << 8) | read_data[5]) & 0x3FFF));
	    	
	 akcel[0] = 0.002522*G*TWOCOMP14((((read_data[6] << 8) | read_data[7]) & 0x3FFF));
	 akcel[1] = -0.002522*G*TWOCOMP14((((read_data[8] << 8) | read_data[9]) & 0x3FFF));
	 akcel[2] = -0.002522*G*TWOCOMP14((((read_data[10] << 8) | read_data[11]) & 0x3FFF));
	    		
	 uhol[0] += gyro[0] * dt * M_PI/180;
	// uhol[1] += gyro[1] * dt * M_PI/180;
	if (akcel[2] < 0)
	{
	 	uhol[2] += 2 * M_PI - (gyro[2] * dt * M_PI/180);
	 	//akcel[0] *= -1;
	 	uhol[1] += 2 * M_PI - (gyro[1] * dt * M_PI/180);
	 	akcel[1] *= -1;
	 }
	 else
	 {
	  	uhol[2] += gyro[2] * dt * M_PI/180;
	  	 uhol[1] += gyro[1] * dt * M_PI/180;
	  }
	    // ROS_INFO("prepocet");			
	 //prepinanie filtrov   		
	    if (complementary_filter)
	    	complementaryFilter();	
	   if (priemer_filter)
	   	priemerFilter();
	    if (filter_g)
	   	filterG();
	   	// ROS_INFO("filter");
	   //zapis na topic imu/data
	msg.linear_acceleration.x = akcel[0];
	msg.linear_acceleration.y = akcel[1];
	msg.linear_acceleration.z = akcel[2];
	
	msg.angular_velocity.x = gyro[0];
	msg.angular_velocity.y = gyro[1];
	msg.angular_velocity.z = gyro[2];
	
	
	msg.linear_acceleration_covariance[0] = 0.005;
	msg.linear_acceleration_covariance[4] = 0.005;
	msg.linear_acceleration_covariance[8] = 100000;
	
	msg.angular_velocity_covariance[0] = 0.0001;
	msg.angular_velocity_covariance[4] = 0.0001;
	msg.angular_velocity_covariance[8] = 0.0005;
	
	msg.orientation_covariance[0] = 0.00001;
	msg.orientation_covariance[4] = 0.00001;
	msg.orientation_covariance[8] = 0.00005;
	
	msg.orientation = tf::createQuaternionMsgFromRollPitchYaw(uhol[0],uhol[1],uhol[2]);
	msg.header.frame_id = frame_id;
	 //ROS_INFO("vyplnenie spravy");
        imu_pub.publish(msg);
        pthread_mutex_unlock(&RS_422_mutex);
         //ROS_INFO("odoslanie na topic");
}

//citanie teploty, neimplementovane!! 
void read_temperature(uint8_t read_data[])
{	
	pthread_mutex_lock(&RS_422_mutex);
	static double temperature[3];
	 static sensor_msgs::Imu msg;
	 static uint8_t write_data = 0x04;
	 my_serial->flush();
	my_serial->write(&write_data,1);
	my_serial->read(read_data,12);
	    		
	 current_time = ros::Time::now();
	   double dt = (current_time - last_time).toSec();
	   last_time = current_time;	  
	     msg.header.stamp = current_time;
	   temperature[0] = 0.1453*TWOCOMP12((((read_data[0] << 8) | read_data[1]) & 0x0FFF));
	   temperature[1] = 0.1453*TWOCOMP12((((read_data[2] << 8) | read_data[3]) & 0x0FFF));
	   temperature[2] = 0.1453*TWOCOMP12((((read_data[4] << 8) | read_data[5]) & 0x0FFF));
	 	    		
	printf("teplota X: %.2f\n",temperature[0]);
	printf("teplota y: %.2f\n",temperature[1]);
	printf("teplota z: %.2f\n\n",temperature[2]);
	pthread_mutex_unlock(&RS_422_mutex);
}

//dlha kalibracia
bool calibrate()
{
	pthread_mutex_lock(&RS_422_mutex);
	uint8_t command[] = {0x01,0x3E,0x00,0x10};
	isCalibrate = false;
	ros::param::set("adis16350/is_calibrate",isCalibrate);
	ROS_WARN("prebieha kalibracia nehybat 30 sekund zo zoriadenim");
	my_serial->flush();
	if (my_serial->write(command,4) < 4)
	{
		ROS_ERROR("adis16350: neuspesna kalibracia na port sa nezapisali 4 byty");
		return false;
	}
	sleep(30);
	isCalibrate = true;
	
	
	//treba vycitat hodnoty lebo po kalibracii pridu najprv blbosti
	uint8_t read_data[12];
	 uint8_t write_data = 0x03;
	 
	for (int i = 0; i < 5; i++){
		my_serial->flush();
		my_serial->write(&write_data,1);
		usleep(100000);
		my_serial->read(read_data,12);

		 gyro[0] = 0.07326 * range * TWOCOMP14((((read_data[0] << 8) | read_data[1]) & 0x3FFF));
		 gyro[1] = 0.07326 * range * TWOCOMP14((((read_data[2] << 8) | read_data[3]) & 0x3FFF));
		 gyro[2] = 0.07326 * range * TWOCOMP14((((read_data[4] << 8) | read_data[5]) & 0x3FFF));
		    	
		 akcel[0] = 0.002522*G*TWOCOMP14((((read_data[6] << 8) | read_data[7]) & 0x3FFF));
		 akcel[1] = -0.002522*G*TWOCOMP14((((read_data[8] << 8) | read_data[9]) & 0x3FFF));
		 akcel[2] = -0.002522*G*TWOCOMP14((((read_data[10] << 8) | read_data[11]) & 0x3FFF)); 
		usleep(100000);
	}
	last_time = ros::Time::now();
	ros::param::set("adis16350/is_calibrate",isCalibrate);
	ROS_WARN("kalibracia dokoncena");
	pthread_mutex_unlock(&RS_422_mutex);
	return true;
}

//kratka kalibracia
bool auto_calibrate()
{
	pthread_mutex_lock(&RS_422_mutex);
	uint8_t command[] = {0x01,0x3E,0x00,0x01};
	isCalibrate = false;
	ros::param::set("adis16350/is_calibrate",isCalibrate);
	ROS_WARN("prebieha kratka kalibracia nehybat 2 sekundy zo zoriadenim");
	my_serial->flush();
	if (my_serial->write(command,4) < 4)
	{
		ROS_ERROR("adis16350: neuspesna kalibracia na port sa nezapisali 4 byty");
		return false;
	}
	sleep(2);
	 
		//treba vycitat hodnoty lebo po kalibracii pridu najprv blbosti
	 uint8_t read_data[12];
	 uint8_t write_data = 0x03;
	 
	for (int i = 0; i < 5; i++){
		my_serial->flush();
		
		my_serial->write(&write_data,1);
		usleep(100000);
		my_serial->read(read_data,12);

		 gyro[0] = 0.07326 * range * TWOCOMP14((((read_data[0] << 8) | read_data[1]) & 0x3FFF));
		 gyro[1] = 0.07326 * range * TWOCOMP14((((read_data[2] << 8) | read_data[3]) & 0x3FFF));
		 gyro[2] = 0.07326 * range * TWOCOMP14((((read_data[4] << 8) | read_data[5]) & 0x3FFF));
		    	
		 akcel[0] = 0.002522*G*TWOCOMP14((((read_data[6] << 8) | read_data[7]) & 0x3FFF));
		 akcel[1] = -0.002522*G*TWOCOMP14((((read_data[8] << 8) | read_data[9]) & 0x3FFF));
		 akcel[2] = -0.002522*G*TWOCOMP14((((read_data[10] << 8) | read_data[11]) & 0x3FFF)); 
		usleep(100000);
	}
	isCalibrate = true;
	ros::param::set("adis16350/is_calibrate",isCalibrate);
	ROS_WARN("kalibracia dokoncena");
	last_time = ros::Time::now();
	pthread_mutex_unlock(&RS_422_mutex);
	return true;
}

bool calibrateCallback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res) 
{
	res.success = calibrate();
	return true;
}
bool autoCalibrateCallback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res) 
{
	res.success = auto_calibrate();
	return true;
}

//mazanie kalibracie
bool restoringCalibration()
{
	pthread_mutex_lock(&RS_422_mutex);
	uint8_t command[] = {0x01,0x3E,0x00,0x02};
	isCalibrate = false;
	ros::param::set("adis16350/is_calibrate",isCalibrate);
	ROS_WARN("restoring calibration: nastavia sa povodne hodnoty offsetov a filtrov");
	my_serial->flush();
	if (my_serial->write(command,4) < 4)
	{
		ROS_ERROR("adis16350: neuspes pri mazani kalibracie");
		return false;
	}
	pthread_mutex_unlock(&RS_422_mutex);
	return true;
}

bool restoringCalibrationCallback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res) 
{
	res.success = restoringCalibration();
	return true;
}

//nastavenie rozsahu. nefunkcne!!!
bool setRangeCallback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res) 
{
	pthread_mutex_lock(&RS_422_mutex);
	int r;
	ros::param::get("adis16350/range",r);
	if ((r != 300) && (r!=150) && (r!=75))
	{
		r = 300;
		ros::param::set("adis16350/range",300);
	}
	uint8_t data_h;
	uint8_t data_l;
	
	if (r == 300)
	{
		data_h = 0x04;
		data_l = 0x04;
	}
	else if (r == 150)
	{
		data_h = 0x02;
		data_l = 0x04;
	}
	else if (r == 75)
	{
		data_h = 0x01;
		data_l = 0x05;
	}
	
	uint8_t command[] = {0x01,SENS_AVG,data_h,data_l};
	uint8_t read_data[] = {0x00,0x00};
	my_serial->flush();
	if (my_serial->write(command,4) < 4)
	{
		ROS_ERROR("adis16350: neuspesny init, zapis do SENS_AVG reg");
		res.success = false;
		res.message = "nezapisalo sa potrebny pocet bytov";
	}	
	command[0] = 0x01;
	my_serial->write(command,2);
	my_serial->read(read_data,2);
	if ((read_data[0] != command[2]) || (read_data[1] != command[3]))
	{
		res.success = false;
		res.message = "do registra sa pokusal zapisat nezmysel";
		ros::param::set("adis16350/range",range * 300);
	}
	else{
	 res.success = true;
	 res.message = "ok";
	 range = r/300.0;
	}
	pthread_mutex_unlock(&RS_422_mutex);
	return true;
}

//zapis do lubovolneho registra
bool writeCallback(adis16350::Write::Request  &req, adis16350::Write::Response &res) 
{
	pthread_mutex_lock(&RS_422_mutex);
	uint8_t read_data[] = {0x00,0x00};
	uint8_t command[] = {0x01,req.reg,(uint8_t)(req.data >> 8),(uint8_t)(req.data & 0x00FF)};
	if (my_serial->write(command,4) < 4)
	{
		res.success = false;
		res.message = "nezapisalo sa potrebny pocet bytov";
		return true;
	}
	command[0] = 0x02;
	command[1] = req.reg;
	my_serial->flush();
	my_serial->write(command,2);
	my_serial->read(read_data,2);
	if (command[1] = 0x3E) //register 0x3E sa neda citat,  zapis musel byt uspesny
	{
		res.success = true;
		res.message = "ok";
	}
	else if ((read_data[0] != command[2]) || (read_data[1] != command[3])) //overenie ci je odpoved spravna
	{
		res.success = false;
		res.message = "do registra sa pokusal zapisat nezmysel";
	}
	else{
		 res.success = true;
	 	res.message = "ok";
	}
	pthread_mutex_unlock(&RS_422_mutex);
	return true;
}

//citanie lubovolneho registra
bool readCallback(adis16350::Read::Request  &req, adis16350::Read::Response &res) 
{
	pthread_mutex_lock(&RS_422_mutex);
	uint8_t read_data[] = {0x00,0x00};
	uint8_t command[] = {0x02,req.reg};
	my_serial->flush();
	my_serial->write(command,2);
	my_serial->read(read_data,2);;
	res.data = ((uint16_t)read_data[0] << 8) | read_data[1];
	pthread_mutex_unlock(&RS_422_mutex);
	return true;
}

//uprava Z rotacie z prijatych dat z externeho kompasu topic /bearing
void kompasCallback(const std_msgs::Float32::ConstPtr& msg) 
{
	uhol[2] = msg->data;
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "adis16350");
    ros::NodeHandle n;
    imu_pub = n.advertise<sensor_msgs::Imu>("imu_data",100);    
     ros::ServiceServer serviceWrite = n.advertiseService("adis16350/write",writeCallback);
    ros::ServiceServer serviceRead = n.advertiseService("adis16350/read",readCallback);
    ros::ServiceServer serviceCalibrate = n.advertiseService("adis16350/calibrate",calibrateCallback);
    ros::ServiceServer serviceAutoCalibrate = n.advertiseService("adis16350/auto_calibrate",autoCalibrateCallback);
    ros::ServiceServer serviceRestoringCalibrate = n.advertiseService("adis16350/restoring_calibrate",restoringCalibrationCallback);
        ros::ServiceServer serviceSetRange = n.advertiseService("adis16350/set_range",setRangeCallback);
        ros::Subscriber sub_kompas = n.subscribe("bearing", 1, kompasCallback);
 	
    	 setROSParam();
     
     
   	
	/*otvorenie portu timeout 10ms*/
	
	my_serial = new serial::Serial(port, baud, serial::Timeout::simpleTimeout(100)); 
	
	if(my_serial->isOpen())
	    ROS_INFO("port otvoreny");
	else
	{
	    ROS_ERROR("port sa neotvoril");
	    while (!my_serial->isOpen())
	    {
	    	ROS_ERROR("port sa neotvoril pravdepodobne nie je pripojene zariadenie");
	    	my_serial = new serial::Serial(port, baud, serial::Timeout::simpleTimeout(100)); 
		sleep(1);
		
	   }
	    return 0;
	  }
	    
	  //start inicializacie
	   ROS_WARN("adis sa bude nastavovat idealne ho umiestnit rovnobezne so zemou");
	    sleep(2);
	    auto_calibrate();
	  //  calibrate();
	    init();
	    ROS_WARN("zistuju sa pociatocne suradnice, nehybat");
	  //  setRotation();
	    sleep(1);
	    last_time = ros::Time::now();
	    
	     ros::Timer timer = n.createTimer(ros::Duration(reading_period/1000.0), read_adis); //citanie prebieha periodickym spustanim casovaca
	    ros::MultiThreadedSpinner spinner(2);
	    spinner.spin();
	
   	return 0;
   }
