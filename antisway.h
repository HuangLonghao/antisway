#ifndef ANTISWAY
#define ANTISWAY
#define FSAMP 100
#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <fstream>
#include <memory>
#include <mutex>
#include <math.h>
#include "Butter1.h"
#include <vector>
#define API __declspec(dllexport)

using namespace std;
#ifdef __cplusplus 
extern "C"
{
	API void PID_INIT(int mode, int direction,int filter_flag,double rope_length, double v_max,double torque_max, double current_x, double target_x,double static_angle=0,double input_gain_X = 480, double input_Kd_X= 4,double input_Ki_X=0.001, double input_gain_theta = 2.3, double input_Kp_theta = 15000, double input_Kd_theta = 10000);
	API double PID_REALIZE(double sampling_time, double angle, double current_x);
	API int PID_IN_PLACE_FLAG(double delta_x=0.04);
	API void INPUTSHAPER_INIT();
	API double INPUTSHAPER_STAGE1(double acceleration, double maxspeed, double deltaspeed,double rope_length);
	API double INPUTSHAPER_STAGE2(double theta1, double theta2, double t, double rope_length, double acceleration);
	API double INPUTSHAPER_STAGE2_DELTA_T(double theta1, double theta2, double t, double rope_length, double acceleration);
}
#endif
class PIDantisway
{
private:
	clock_t start;//开始时间
	int ctrl_mode;//控制模式，0表示手动自适应控制模式，1表示自动自适应控制模式,2表示手动控制调试模式，3表示自动控制调试模式
	int butterworth_filter_flag;//开启滤波器标志
	int in_place_flag;//到位标志
	double load_mass;//载荷质量
	double small_car_mass;//小车重量
	double current_x_saved;
	double target_x_saved;
	double max_torque,vmax;//最大转矩,最大速度
	double err;//定义偏差值
	double last_err;//定义上一个偏差值
	double last_theta;//上一个角度
	double last_x;//上一个位移
	double integral;
	double gain_x, gain_theta, Kd_x, Ki_x, Kp_theta, Kd_theta;//定义前向增益/比例系数和微分系数
	double torque;//定义电机转矩(控制执行器的变量)
	double static_angle;//静态角	
	double distance;
	static shared_ptr<PIDantisway> my_shared_ptr;	
	static std::mutex wrapper_mutex;
	static shared_ptr<Butter1> filt1;
	std::vector<float> angle_vec;
	std::vector<float> angle_vec_filtered;
	int index;
	//static bool b_init;
public:
	static shared_ptr<PIDantisway> GetInstance();	

	void PID_init(int mode,int direction,int filter_flag,double rope_length, double v_max, double torque_max, double current_x,double target_x,
		double static_angle=0, double input_gain_X=480, double input_Kd_X=4, double input_Ki_X = 0.001, double input_gain_theta=2.3, double input_Kp_theta=15000, double input_Kd_theta=10000);//增加了海陆侧方位，0表示海侧，1表示陆侧，增加了目标位置信息
	double PID_realize(double sampling_time, double angle, double current_x);//去掉了目标位置输入参数，改为在初始化阶段完成
	int PID_in_place_flag(double delta_x);
};
class InputShaperAntisway
{
private:
	static shared_ptr<InputShaperAntisway> my_shared_ptr2;
	double amax;//最大加速度
	double vmax;//最大速度
	double l;//绳长
	double w;//角频率
	double t;//加速时间
	double a;//加速度
	static std::mutex wrapper_mutex2;
public:
	static shared_ptr<InputShaperAntisway> GetInstance();
	double InputShaper_stage1(double acceleration, double maxspeed, double settingspeed, double rope_length);//返回匀加速到0.3最大速度后匀速前进的时间
	double InputShaper_stage2(double theta1, double theta2, double t, double rope_length, double acceleration);//返回阶段二开始后进行匀加速并匀减速（各用一个t）的时间
	double InputShaper_stage2_delta_t(double theta1, double theta2, double t, double rope_length, double acceleration);//返回在测量开始后到开始阶段二之间进行匀速前进的时间
};

#endif