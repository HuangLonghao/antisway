#include "antisway.h"

using namespace std;
shared_ptr<PIDantisway>PIDantisway::my_shared_ptr = nullptr;
shared_ptr<InputShaperAntisway>InputShaperAntisway::my_shared_ptr2 = nullptr;
std::mutex PIDantisway::wrapper_mutex;
shared_ptr<Butter1>PIDantisway::filt1 = nullptr;
std::mutex InputShaperAntisway::wrapper_mutex2;
API void PID_INIT(int mode, int direction,int filter_flag,double rope_length, double v_max, double torque_max, double current_x,double target_x, double static_angle, double input_gain_X, double input_Kd_X, double input_Ki_x, double input_gain_theta, double input_Kp_theta, double input_Kd_theta)
{
	//zhinengzhineng
	shared_ptr<PIDantisway> temp_ptr = PIDantisway::GetInstance();
	temp_ptr->PID_init(mode, direction, filter_flag,rope_length, v_max, torque_max, current_x, target_x,static_angle, input_gain_X, input_Kd_X, input_Ki_x,input_gain_theta, input_Kp_theta, input_Kd_theta);
	//init
	return;
}
API int PID_IN_PLACE_FLAG(double delta_x)
{
	shared_ptr<PIDantisway> temp_ptr = PIDantisway::GetInstance();
	return temp_ptr->PID_in_place_flag(delta_x);
}
API void INPUTSHAPER_INIT()
{
	//zhinengzhineng
	shared_ptr<InputShaperAntisway> temp_ptr2 = InputShaperAntisway::GetInstance();
	//init
	return;
}
API double PID_REALIZE(double sampling_time, double angle,double current_x)
{
	shared_ptr<PIDantisway> temp_ptr = PIDantisway::GetInstance();
	return temp_ptr->PID_realize( sampling_time,angle, current_x);
	//get_shared_ptr
	//return object.PID_realize();
}
API double INPUTSHAPER_STAGE1(double acceleration, double maxspeed, double settingspeed,double rope_length)
{
	shared_ptr<InputShaperAntisway> temp_ptr2 = InputShaperAntisway::GetInstance();
	return temp_ptr2->InputShaper_stage1(acceleration, maxspeed, settingspeed, rope_length);
}
API double INPUTSHAPER_STAGE2(double theta1, double theta2, double t, double rope_length, double acceleration)
{
	shared_ptr<InputShaperAntisway> temp_ptr2 = InputShaperAntisway::GetInstance();
	return temp_ptr2->InputShaper_stage2(theta1,theta2,t,rope_length,acceleration);
}
API double INPUTSHAPER_STAGE2_DELTA_T(double theta1, double theta2, double t, double rope_length, double acceleration)
{
	shared_ptr<InputShaperAntisway> temp_ptr2 = InputShaperAntisway::GetInstance();
	return temp_ptr2->InputShaper_stage2_delta_t(theta1, theta2, t, rope_length, acceleration);
}
shared_ptr<PIDantisway>PIDantisway::GetInstance()
{
	wrapper_mutex.lock();
	if (my_shared_ptr == nullptr)
	{
 		PIDantisway *temp_ptr = new PIDantisway();
 		my_shared_ptr = shared_ptr<PIDantisway>(temp_ptr);
	}
	wrapper_mutex.unlock();
	return my_shared_ptr;
};
shared_ptr<InputShaperAntisway>InputShaperAntisway::GetInstance()
{
	wrapper_mutex2.lock();
	if (my_shared_ptr2 == nullptr)
	{
		InputShaperAntisway *temp_ptr = new InputShaperAntisway();
		my_shared_ptr2 = shared_ptr<InputShaperAntisway>(temp_ptr);
	}
	wrapper_mutex2.unlock();
	return my_shared_ptr2;
};
void PIDantisway::PID_init(int mode,int direction,int filter_flag,double rope_length, double v_max, double torque_max, double current_x,double target_x,
	double input_static_angle, double input_gain_X, double input_Kd_X, double input_Ki_X, double input_gain_theta, double input_Kp_theta, double input_Kd_theta)
{	
	cout << "PID_init begin ..."<<endl;	
	last_theta = input_static_angle;
	last_x = current_x;
	torque = 0.0;
	distance = 0;
	integral = 0;
	ctrl_mode = mode;
	in_place_flag = 0;
	target_x_saved = target_x;
	static_angle = input_static_angle;
	vmax = v_max;
	max_torque = torque_max;
	index = 0;
	angle_vec.clear();
	angle_vec_filtered.clear();
	butterworth_filter_flag = filter_flag;
	Butter1 *temp_filter= new Butter1(5, FSAMP);
	filt1 = shared_ptr<Butter1>(temp_filter);
	//if(abs(target_x-current_x)<=0.1)
	//if (abs(target_x - current_x) <= 0.1)//当位移小于等于3m时采用以下参数
	//{
	//	gain_x = 50;
	//	Kd_x = 3;
	//}
	//if (abs(target_x - current_x) <= 3)//当位移小于等于3m时采用以下参数
	//{
	//	gain_x = 28;
	//	Kd_x = 3.6;
	//}
	//else if (abs(target_x - current_x) <= 6)//当位移小于等于3m时采用以下参数
	//{
	//	gain_x = 20;
	//	Kd_x = 3.8;
	//}
	//else if (abs(target_x - current_x) <= 9)
	//{
	//	gain_x = 17;
	//	Kd_x = 4.2;		
	//}
	//else//否则采用另一组参数
	//{
	//	gain_x = 14;
	//	Kd_x = 4.8;				
	//}
	//x相关参数与位移距离以及海陆侧方向有关
	
	gain_x = 37.25 - 3.5*abs(target_x - current_x) + 0.14*pow(abs(target_x - current_x), 2);
	cout << "gain_x:" << gain_x << endl;
	Kd_x=3.6-0.033*abs(target_x - current_x)+0.011*pow(abs(target_x - current_x), 2);
	cout << "Kd_x:" << Kd_x << endl;
	double rope_length_factor_ki_x = 1.4 - 0.056*rope_length;
	if (direction==0)//0表示海侧方向，海侧方向需要加积分作为补偿
	{
		Ki_x = 0.086*exp(-0.1915*abs(target_x - current_x))*rope_length_factor_ki_x;//新加入绳长补偿参数
	}
	else//表示陆侧方向
	{
		Ki_x = 0;
		Kd_x += 0.2;//陆侧方向微分系数增加0.2
	}
	cout << "Ki_x:"<<Ki_x << endl;
	gain_theta = 23;
	double distance_factor_Kp_theta = 1.25 - 0.117*abs(target_x - current_x);//加入角度修正系数
	if (distance_factor_Kp_theta<0)
	{
		distance_factor_Kp_theta = 0;
	}
	
	//根据绳长来调整kp_theta与kd_theta,并且采用分段函数形式
	//kp_theta分段函数
	if (rope_length<4.4)
	{
		Kp_theta = 150;
	}
	else if (rope_length==4.4)
	{
		Kp_theta = 0;
	}
	else if (rope_length>4.4)
	{
		Kp_theta = 200 - 45 * rope_length;
	}
	Kp_theta = Kp_theta*distance_factor_Kp_theta;
	//kd_theta分段函数
	Kd_theta = 24 - 31.2*rope_length;
	cout << "rope_length:" << rope_length << endl;
	cout << "Kp_theta:"<< Kp_theta << endl;
	cout << "Kd_theta:" << Kd_theta << endl;
	//如果采用调试模式的话将采用外界输入参数
	//if(ctrl_mode=2||ctrl_mode==3){gain_x=input_gain_x;gain_x=input_gain_xgaintheta=inputOgain_theta;Ki_x=input_ki_x}
	if (ctrl_mode==2||ctrl_mode==3)
	{
		gain_x = input_gain_X;
		gain_theta = input_gain_theta;
		Ki_x = input_Ki_X;
		Kp_theta = input_Kp_theta;
		Kd_theta = input_Kd_theta;
		Kd_x = input_Kd_X;
	}
	cout << "PID_init end \n";
}


double PIDantisway::PID_realize(double sampling_time, double angle, double current_x)
{
	clock_t now = clock();
	//if (angle > umax)
	//{
	//	//抗积分饱和的实现
	//	if (abs(err) > 0.003)
	//		//积分分离过程
	//	{
	//		index = 0;
	//	}
	//	else {
	//		index = 1;
	//		if (err < 0)
	//		{
	//			integral += err;
	//		}
	//	}
	//}
	//else if (angle < umin) {
	//	if (abs(err) > 0.003)
	//		//积分分离过程
	//	{
	//		index = 0;
	//	}
	//	else {
	//		index = 1;
	//		if (err > 0)
	//		{
	//			integral += err;
	//		}
	//	}
	//}
	//else {
	//	if (abs(err) > 0.003)
	//		//积分分离过程
	//	{
	//		index = 0;
	//	}
	//	else {
	//		index = 1;
	//		integral += err;
	//	}
	//}
	cout << "before filtered:" << current_x <<endl;
	if (butterworth_filter_flag==1)
	{
		double filterd_current_x;
		if (index >= 10)
		{
			filterd_current_x = filt1->process(current_x);
		}
		else
		{
			filterd_current_x = current_x;
		}			
		current_x = filterd_current_x;
		cout << "after filtered:" << current_x << endl;
	}
	current_x_saved = current_x;

	if (distance < abs(current_x - target_x_saved))
	{
		distance = abs(current_x - target_x_saved);
		cout << "distance:" << distance << endl;
	}
	integral += target_x_saved - (Kd_x / sampling_time*(current_x - last_x) + current_x);
	if (ctrl_mode==1||ctrl_mode==3)//控制模式为1或者3的时候为自动模式
	{
		torque = gain_theta*(gain_x*((target_x_saved - (Kd_x / sampling_time*(current_x - last_x) + current_x)) + Ki_x*integral*sampling_time) - (Kd_theta / sampling_time*(
			angle - static_angle - last_theta) + Kp_theta*(angle - static_angle)));
	}
	else//否则为手动模式（只控制摆角）
	{
		torque = gain_theta*gain_x*(0- (Kd_theta / sampling_time*(
			angle - static_angle - last_theta) + Kp_theta*(angle - static_angle)));
	}	
	if (torque > max_torque)
	{
		torque = max_torque;
	}
	if (torque<-max_torque)
	{
		torque = -max_torque;
	}
	cout << "当前转矩" << torque << endl;
	last_x = current_x;
	last_theta = angle- static_angle;
	index++;
	return torque;
}
int PIDantisway::PID_in_place_flag(double delta_x)
{
	if (abs(current_x_saved - target_x_saved)<= delta_x)
	{
		in_place_flag = 1;
	}
	else
	{
		in_place_flag = 0;
	}
	return in_place_flag;
}
double InputShaperAntisway::InputShaper_stage2(double theta1, double theta2, double t, double rope_length, double acceleration)
{
	double T = t;//计算采样周期
	w = sqrt(9.8 / rope_length);//计算角频率
	double A = std::sqrt(std::pow((theta1 - theta2), 2) / pow(sin(-T*w*0.5), 2) + std::pow((theta1 + theta2), 2) / pow(cos(-T*w*0.5), 2));//计算单摆幅值
	double t0 = sqrt(rope_length*sin(A) / acceleration);
	return t0;
}
//
double InputShaperAntisway::InputShaper_stage1(double acceleration, double maxspeed, double deltaspeed,double rope_length)
{
	cout << "等效周期" << 2*3.14*sqrt(rope_length / 9.8)*0.9558 - 0.351 << endl;
	return 3.14*sqrt(rope_length / 9.8)*0.9558-0.351/2 - 0.5*deltaspeed / acceleration;
}
double InputShaperAntisway::InputShaper_stage2_delta_t(double theta1, double theta2, double t, double rope_length, double acceleration)
{
	double T = t;//计算采样周期
	w = sqrt(9.8 / rope_length);//计算角频率
	double A = std::sqrt(std::pow((theta1 - theta2), 2) / pow(sin(-T*w*0.5), 2) + std::pow((theta1 + theta2), 2) / pow(cos(-T*w*0.5), 2));//计算单摆幅值
	cout << "A=" << A <<" w="<<w<< endl;
	double delta_t = (3.14 - acos(theta2 / A)) / w;
	if (theta2>theta1)//此时应该处于[-pi,0]区间，调用arccos函数值域不对
	{
		delta_t = (3.14+ acos(theta2 / A)) / w;
	}
	else//此时应该处于[0,pi]区间，可以直接调用arccos
	{
		delta_t = (3.14 -acos(theta2 / A)) / w;
	}
	return delta_t;
}
//