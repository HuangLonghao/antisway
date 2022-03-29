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
	clock_t start;//��ʼʱ��
	int ctrl_mode;//����ģʽ��0��ʾ�ֶ�����Ӧ����ģʽ��1��ʾ�Զ�����Ӧ����ģʽ,2��ʾ�ֶ����Ƶ���ģʽ��3��ʾ�Զ����Ƶ���ģʽ
	int butterworth_filter_flag;//�����˲�����־
	int in_place_flag;//��λ��־
	double load_mass;//�غ�����
	double small_car_mass;//С������
	double current_x_saved;
	double target_x_saved;
	double max_torque,vmax;//���ת��,����ٶ�
	double err;//����ƫ��ֵ
	double last_err;//������һ��ƫ��ֵ
	double last_theta;//��һ���Ƕ�
	double last_x;//��һ��λ��
	double integral;
	double gain_x, gain_theta, Kd_x, Ki_x, Kp_theta, Kd_theta;//����ǰ������/����ϵ����΢��ϵ��
	double torque;//������ת��(����ִ�����ı���)
	double static_angle;//��̬��	
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
		double static_angle=0, double input_gain_X=480, double input_Kd_X=4, double input_Ki_X = 0.001, double input_gain_theta=2.3, double input_Kp_theta=15000, double input_Kd_theta=10000);//�����˺�½�෽λ��0��ʾ���࣬1��ʾ½�࣬������Ŀ��λ����Ϣ
	double PID_realize(double sampling_time, double angle, double current_x);//ȥ����Ŀ��λ�������������Ϊ�ڳ�ʼ���׶����
	int PID_in_place_flag(double delta_x);
};
class InputShaperAntisway
{
private:
	static shared_ptr<InputShaperAntisway> my_shared_ptr2;
	double amax;//�����ٶ�
	double vmax;//����ٶ�
	double l;//����
	double w;//��Ƶ��
	double t;//����ʱ��
	double a;//���ٶ�
	static std::mutex wrapper_mutex2;
public:
	static shared_ptr<InputShaperAntisway> GetInstance();
	double InputShaper_stage1(double acceleration, double maxspeed, double settingspeed, double rope_length);//�����ȼ��ٵ�0.3����ٶȺ�����ǰ����ʱ��
	double InputShaper_stage2(double theta1, double theta2, double t, double rope_length, double acceleration);//���ؽ׶ζ���ʼ������ȼ��ٲ��ȼ��٣�����һ��t����ʱ��
	double InputShaper_stage2_delta_t(double theta1, double theta2, double t, double rope_length, double acceleration);//�����ڲ�����ʼ�󵽿�ʼ�׶ζ�֮���������ǰ����ʱ��
};

#endif