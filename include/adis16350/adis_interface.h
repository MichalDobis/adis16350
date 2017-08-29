#include <stdio.h>
#include <math.h>
#include <boost/thread/mutex.hpp>
#include <serial/serial.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>


#define TWOCOMP14(x) (((int16_t)(x)<0x2000)?(int16_t)(x):-(0x4000-(int16_t)(x)))
#define TWOCOMP12(x) (((int16_t)(x)<0x0800)?(int16_t)(x):-(0x1000-(int16_t)(x)))
#define DEG2RAG(x) 0.0174532925199433*(x)

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

//todo
//dokoncit diagnostiku , stav ze prebieha kalibracia
//skusit vyriesit teplotu a hodnotu napajania

class AdisInterface{
public:
	typedef struct axes{
		bool x;
		bool y;
		bool z;
	}AXES;

	typedef struct status{
		AXES akcel_err;		//Z-axis accelerometer self-diagnostic error flag
							//		1 = failure, 0 = passing
							//		Y-axis accelerometer self-diagnostic error flag
							//		1 = failure, 0 = passing
							//		X-axis accelerometer self-diagnostic error flag
							//		1 = failure, 0 = passing
		AXES gyro_err;		//		Z-axis gyroscope self-diagnostic error flag
							//		1 = failure, 0 = passing
							//		Y-axis gyroscope self-diagnostic error flag
							//		1 = failure, 0 = passing
							//		X-axis gyroscope self-diagnostic error flag
							//		1 = failure, 0 = passing
		bool alarm2;					//		Alarm 2 status
							//		1 = active, 0 = inactive
		bool alarm1;					//		Alarm 1 status
							//		1 = active, 0 = inactive
		bool self_test_err;					//		Self-test diagnostic error flag
							//		1 = error condition, 0 = normal operation
		bool over_range;					//		Sensor over range (any of the six)
							//		1 = error condition, 0 = normal operation
		bool spi_err;					//		SPI communications failure
							//		1 = error condition, 0 = normal operation
		bool ctr_update_err;//		Control register update failed
							//		1 = error condition, 0 = normal operation
		bool power_supply_above;	//		Power supply in range above 5.25 V
									//		1 = above 5.25 V, 0 = below 5.25 V (normal)
		bool power_supply_below;		//		Power supply below 4.75 V
									//		1 = below 4.75 V, 0 = above 4.75 V (normal)
	}STATUS_STRUCT;

	AdisInterface();
	bool init(std::string port, int baud);			//otvorenie portu
	bool setSENS_AVG(uint8_t SENS, uint8_t AVG);	//nastavenie citlivosti gyra a nastavenie filtra
	bool setSMPL_PRD(uint8_t SMPL);					//nastavenie periody vzorkovania
	bool setMSC_CTRL(uint8_t MSC);					//nastavenie registra MSC_CTRL Data-Ready Input/Output Indicator

	uint16_t getStatus(STATUS_STRUCT *status);		//vravia status

	uint16_t readRegister(uint8_t reg);				//precita lubovolny register
	bool writeRegister(uint8_t reg, uint16_t data); //zapise hodnota do lubovolneho registra

	bool restoringCalibration();					//zmaze obsah z offset registrov
	bool autoCalibrate();							//rychla autokalibracia
	bool calibrate();								//30 sekundova precizna kalibracia
	bool calibrationGyroZ();						//rucne nastavenie offset pre gyro Z. Zrata 2000 vzoriek vypocita priemer a podla toho zapise do registra
	bool computeOffset();
	bool getImuData(sensor_msgs::Imu *imu);			//vracia precitane data


	bool isActive();								//informacia, ci je port otvoreny
	int getRange();									//informacia o nastavenom rozsahu gyra 0x01 - 75°/s 0x02 - 150°/s 0x04 - 300°/s
	geometry_msgs::Vector3 getTemperature();		//vracia teplotu

	static const double GRAVITY = 9.80665;

private:
	bool active;
	int range;
	serial::Serial* my_serial;
	boost::mutex mutex;
	boost::mutex calibrationRunning;
	double SCALE_GYRO;
	double SCALE_AKCEL;

	double z_offset;

	bool readAxes(sensor_msgs::Imu *imu);					    //cita imu data zo zbernice
	bool writeRegister(uint8_t reg, uint8_t hi, uint8_t lo);    //zapisuje do registra
	bool readRegister(uint8_t reg, uint8_t *read_data);			//citanie z lubovolneho registra
	bool checkRegister(uint8_t reg, uint8_t *controlled_data);  //porovna controlled_data z precitanymi datami
	double meassureAverageZ();
};
