// ED_FM_Template.cpp : Defines the exported functions for the DLL application.
#include "stdafx.h"
#include "A6E_FM.h"
#include "A6E_FM_Utility.h"
#include <Math.h>
#include <stdio.h>
#include <string>

// start user define class
#include "Interface/A6eInterface.h"

namespace A6E
{
	// A6eAtmosphere Atmos;
	A6eInterface Interface;
}

// template 给的参考算法
Vec3	common_moment;
Vec3	common_force;
Vec3    center_of_gravity;
Vec3	wind;
Vec3	velocity_world_cs;
double  throttle		  = 0;
double  stick_roll		  = 0;
double  stick_pitch		  = 0;

double  internal_fuel     = 0;
double  fuel_consumption_since_last_time  = 0;
double  atmosphere_density = 0;
double  aoa = 0;
double  speed_of_sound = 320;

float 	test_ptn_521 = 0;

double mach_table[] = {
	0,	
	0.2,
	0.4,
	0.6,
	0.7,
	0.8,
	0.9,
	1,	
	1.05,
	1.1,
	1.2,
	1.3,
	1.5,
	1.7,
	1.8,
	2,	
	2.2,
	2.5,
	3.9,
};

double cx0[] =
{
	0.0165,
	0.0165,
	0.0165,
	0.0165,
	0.0170,
	0.0178,
	0.0215,
	0.0310,
	0.0422,
	0.0440,
	0.0432,
	0.0423,
	0.0416,
	0.0416,
	0.0416,
	0.0410,
	0.0395,
};

double Cya[] = {
	   0.077,	
	   0.077,	
	   0.077,	
	   0.080,	
	   0.083,	
	   0.087,	
	   0.091,	
	   0.094,	
	   0.094,	
	   0.091,	
	   0.085,	
	   0.068,	
	   0.051,	
	   0.043,	
	   0.037,	
	   0.036,	
	   0.033,	
};

double B[] ={
	0.1,	
	0.1,	
	0.1,	 
	0.094,	
	0.094,	
	0.094,	
	0.11,	
	0.15,	
	0.15,	
	0.14,	
	0.17,	
	0.23,	
	0.23,	
	0.08,	
	0.16,	
	0.25,	
	0.35,	
};

double B4[] ={
	0.032,	
	0.032,	 
	0.032,	
	0.043,	
	0.045,	
	0.048,	
	0.050,	
	0.1,	
	0.1,	
	0.1,	
	0.096,	
	0.09,	
	0.38,	
	2.5,	
	3.2,	
	4.5,	
	6.0,	
};

double CyMax[] = {
	1.6,
	1.6,
	1.6,
	1.5,
	1.45,
	1.4,
	1.3,
	1.2,
	1.1,
	1.05,
	1.0,
	0.9,
	0.7,
	0.55,
	0.4,
	0.4,
	0.4,
};



void add_local_force(const Vec3 & Force, const Vec3 & Force_pos)
{
	common_force.x += Force.x;
	common_force.y += Force.y;
	common_force.z += Force.z;

	Vec3 delta_pos(Force_pos.x - center_of_gravity.x,
				   Force_pos.y - center_of_gravity.y,
				   Force_pos.z - center_of_gravity.z);

	Vec3 delta_moment = cross(delta_pos, Force);

	common_moment.x += delta_moment.x;
	common_moment.y += delta_moment.y;
	common_moment.z += delta_moment.z;
}

void simulate_fuel_consumption(double dt)
{
	fuel_consumption_since_last_time =  10 * throttle * dt; //10 kg persecond
	if (fuel_consumption_since_last_time > internal_fuel)
		fuel_consumption_since_last_time = internal_fuel;
	internal_fuel -= fuel_consumption_since_last_time;
}
// 到这之前都是一个简单的算法来算力，是一个参考

// 这里开始预定义的dll函数，就是默认DCS的核心会调用的

/*	function of force source in body axis 
	x,y,z			  - force components in body coordinate system 不同方向上的力
	pos_x,pos_y,pos_z - position of force source in body coordinate system 合力作用位置

	body coordinate system system is always X - positive forward ,
											Y - positive up,
											Z - positive to right 
	轴向是反的。Z轴为横向的，Y轴为纵向的
*/

// 每帧之后运算，用来让气动模型把当前的作用力(可能是所有力的矢量和)和重心位置（是合力位置，F16 模组描述不明确， 参考前面的英文提示）传递给dcs
void ed_fm_add_local_force(double & x,double &y,double &z,double & pos_x,double & pos_y,double & pos_z)
{
	x = common_force.x;
	y = common_force.y;
	z = common_force.z;
	pos_x = center_of_gravity.x;
	pos_y = center_of_gravity.y;
	pos_z = center_of_gravity.z;
}
// 全局力，不知道是啥
void ed_fm_add_global_force(double & x,double &y,double &z,double & pos_x,double & pos_y,double & pos_z)
{

}

// same but in component form , return value bool : function will be called until return value is true
// 与上面两个add force 相同的函数，但是用的是“组件形式”（不知道是哪里的术语），函数会保持请求直到返回真（？？？16为啥返回的否
bool ed_fm_add_local_force_component(double & x,double &y,double &z,double & pos_x,double & pos_y,double & pos_z)
{
	return false;
}
bool ed_fm_add_global_force_component(double & x,double &y,double &z,double & pos_x,double & pos_y,double & pos_z)
{
	return false;
}

// 反馈力矩信息，决定pitch的，全局力矩还是不知道是啥用的
void ed_fm_add_local_moment(double & x,double &y,double &z)
{
	x = common_moment.x;
	y = common_moment.y;
	z = common_moment.z;
}

void ed_fm_add_global_moment(double & x,double &y,double &z)
{

}

// 一样的component模式，直接turn false完事
bool ed_fm_add_local_moment_component(double & x,double &y,double &z)
{
	return false;
}

bool ed_fm_add_global_moment_component(double & x,double &y,double &z)
{
	return false;
}

// 主模拟函数， dt是预期每帧时间，按理是固定的
// 按理说这里会同步数据和处理暂停（希望

void ed_fm_simulate(double dt)
{

	common_force  = Vec3();
	common_moment = Vec3();

	Vec3 airspeed;

	airspeed.x = velocity_world_cs.x - wind.x;
	airspeed.y = velocity_world_cs.y - wind.y;
	airspeed.z = velocity_world_cs.z - wind.z;

	
	Vec3 thrust_pos(-5.0,0,0);
	Vec3 thrust(throttle * 5000, 0 , 0);

	double V_scalar =  sqrt(airspeed.x * airspeed.x + airspeed.y * airspeed.y + airspeed.z * airspeed.z);

	double Mach		= V_scalar/ speed_of_sound;

	double CyAlpha_ = lerp(mach_table,Cya  ,sizeof(mach_table)/sizeof(double),Mach);
	double Cx0_     = lerp(mach_table,cx0  ,sizeof(mach_table)/sizeof(double),Mach);
	double CyMax_   = lerp(mach_table,CyMax,sizeof(mach_table)/sizeof(double),Mach);
	double B_	    = lerp(mach_table,B    ,sizeof(mach_table)/sizeof(double),Mach);
	double B4_	    = lerp(mach_table,B4   ,sizeof(mach_table)/sizeof(double),Mach);


	double Cy  = (CyAlpha_ * 57.3) * aoa;
	if (fabs(aoa) > 90/57.3)
		Cy = 0;
	if (Cy > CyMax_)
		Cy = CyMax_;

	double Cx  = 0.05 + B_ * Cy * Cy + B4_ * Cy * Cy * Cy * Cy;

	double q	   =  0.5 * atmosphere_density * V_scalar * V_scalar;
	const double S = 25;
	double Lift =  Cy * q * S;
	double Drag =  Cx * q * S;
	
	Vec3 aerodynamic_force(-Drag , Lift , 0 );
	Vec3 aerodynamic_force_pos(1.0,0,0);

	add_local_force(aerodynamic_force,aerodynamic_force_pos);
	add_local_force(thrust			 ,thrust_pos);

	Vec3 aileron_left (0 , 0.05 * Cy * (stick_roll) * q * S , 0 );
	Vec3 aileron_right(0 ,-0.05 * Cy * (stick_roll) * q * S , 0 );

	Vec3 aileron_left_pos(0,0,-5.0);
	Vec3 aileron_right_pos(0,0, 5.0);


	add_local_force(aileron_left ,aileron_left_pos);
	add_local_force(aileron_right,aileron_right_pos);

	simulate_fuel_consumption(dt);
}

// 获取大气数据，这个函数会在simulation前调用
void ed_fm_set_atmosphere(double h,//altitude above sea level
							double t,//current atmosphere temperature , Kelwins
							double a,//speed of sound
							double ro,// atmosphere density
							double p,// atmosphere pressure
							double wind_vx,//components of velocity vector, including turbulence in world coordinate system
							double wind_vy,//components of velocity vector, including turbulence in world coordinate system
							double wind_vz //components of velocity vector, including turbulence in world coordinate system
						)
{
	wind.x = wind_vx;
	wind.y = wind_vy;
	wind.z = wind_vz;

	atmosphere_density = ro;
	speed_of_sound     = a;
}

/*
called before simulation to set up your environment for the next step
*/
// 此处会在模拟开始时调用，提供默认状态
void ed_fm_set_current_mass_state (double mass, // 不包含油量的干重
									double center_of_mass_x,
									double center_of_mass_y,
									double center_of_mass_z,
									double moment_of_inertia_x,
									double moment_of_inertia_y,
									double moment_of_inertia_z
									)
{
	center_of_gravity.x  = center_of_mass_x;
	center_of_gravity.y  = center_of_mass_y;
	center_of_gravity.z  = center_of_mass_z;
}
/*
called before simulation to set up your environment for the next step
*/
// 在模拟函数前设置基于世界参考系的状态
void ed_fm_set_current_state (double ax,//linear acceleration component in world coordinate system
							double ay,//linear acceleration component in world coordinate system
							double az,//linear acceleration component in world coordinate system
							double vx,//linear velocity component in world coordinate system
							double vy,//linear velocity component in world coordinate system
							double vz,//linear velocity component in world coordinate system
							double px,//center of the body position in world coordinate system
							double py,//center of the body position in world coordinate system
							double pz,//center of the body position in world coordinate system
							double omegadotx,//angular accelearation components in world coordinate system
							double omegadoty,//angular accelearation components in world coordinate system
							double omegadotz,//angular accelearation components in world coordinate system
							double omegax,//angular velocity components in world coordinate system
							double omegay,//angular velocity components in world coordinate system
							double omegaz,//angular velocity components in world coordinate system
							double quaternion_x,//orientation quaternion components in world coordinate system
							double quaternion_y,//orientation quaternion components in world coordinate system
							double quaternion_z,//orientation quaternion components in world coordinate system
							double quaternion_w //orientation quaternion components in world coordinate system
							)
{
	velocity_world_cs.x = vx;
	velocity_world_cs.y = vy;
	velocity_world_cs.z = vz;
}

// 在模拟函数前设置基于飞机坐标系的状态
void ed_fm_set_current_state_body_axis(double ax,//linear acceleration component in body coordinate system
	double ay,//linear acceleration component in body coordinate system
	double az,//linear acceleration component in body coordinate system
	double vx,//linear velocity component in body coordinate system
	double vy,//linear velocity component in body coordinate system
	double vz,//linear velocity component in body coordinate system
	double wind_vx,//wind linear velocity component in body coordinate system
	double wind_vy,//wind linear velocity component in body coordinate system
	double wind_vz,//wind linear velocity component in body coordinate system

	double omegadotx,//angular accelearation components in body coordinate system
	double omegadoty,//angular accelearation components in body coordinate system
	double omegadotz,//angular accelearation components in body coordinate system
	double omegax,//angular velocity components in body coordinate system
	double omegay,//angular velocity components in body coordinate system
	double omegaz,//angular velocity components in body coordinate system
	double yaw,  //radians
	double pitch,//radians
	double roll, //radians
	double common_angle_of_attack, //AoA radians
	double common_angle_of_slide   //AoS radians
	)
{
	aoa = common_angle_of_attack;
}

/*
input handling
处理传入按键状态，刷新操作面等
*/
void ed_fm_set_command (int command,
							float value)
{
	if (command == 2004)//iCommandPlaneThrustCommon
	{
		throttle = 0.5 * (-value + 1.0);
	}
	else if (command == 2001)//iCommandPlanePitch
	{
		stick_pitch		  = value;
	}
	else if (command == 2002)//iCommandPlaneRoll
	{
		stick_roll		  = value;
	}
}
/*
	Mass handling 

	will be called  after ed_fm_simulate :
	you should collect mass changes in ed_fm_simulate 

	double delta_mass = 0;
	double x = 0;
	double y = 0; 
	double z = 0;
	double piece_of_mass_MOI_x = 0;
	double piece_of_mass_MOI_y = 0; 
	double piece_of_mass_MOI_z = 0;
 
	//
	while (ed_fm_change_mass(delta_mass,x,y,z,piece_of_mass_MOI_x,piece_of_mass_MOI_y,piece_of_mass_MOI_z))
	{
	//internal DCS calculations for changing mass, center of gravity,  and moments of inertia
	}
*/
// 在simulation函数后调用，需要反馈这次simulation中的燃油消耗
bool ed_fm_change_mass  (double & delta_mass,
						double & delta_mass_pos_x,
						double & delta_mass_pos_y,
						double & delta_mass_pos_z,
						double & delta_mass_moment_of_inertia_x,
						double & delta_mass_moment_of_inertia_y,
						double & delta_mass_moment_of_inertia_z
						)
{
	if (fuel_consumption_since_last_time > 0)
	{
		delta_mass		 = fuel_consumption_since_last_time;
		delta_mass_pos_x = -1.0;
		delta_mass_pos_y =  1.0;
		delta_mass_pos_z =  0;

		delta_mass_moment_of_inertia_x	= 0;
		delta_mass_moment_of_inertia_y	= 0;
		delta_mass_moment_of_inertia_z	= 0;

		fuel_consumption_since_last_time = 0; // set it 0 to avoid infinite loop, because it called in cycle 
		// better to use stack like structure for mass changing 
		return true;
	}
	else 
	{
		return false;
	}
}
/*
	set internal fuel volume , init function, called on object creation and for refueling , 
	you should distribute it inside at different fuel tanks
	初始化内部燃油数据
*/
void   ed_fm_set_internal_fuel(double fuel)
{
	internal_fuel = fuel;
}
/*
	get internal fuel volume 
	ed获取内油情况
*/
double ed_fm_get_internal_fuel()
{
	return internal_fuel;
}
/*
	set external fuel volume for each payload station , called for weapon init and on reload
	在初始化和装载时提供此燃油调用设置
*/
void  ed_fm_set_external_fuel (int	 station,
								double fuel,
								double x,
								double y,
								double z)
{

}
/*
	get external fuel volume 
	ed获取外部燃油数据
*/
double ed_fm_get_external_fuel ()
{
	return 0;
}

// 获取加油量更新
double ed_fm_refueling_add_fuel(double fuel)
{
	return 0;
}

// 绘制动画函数
// drawargs[argnum].f = (float)your transfer data
void ed_fm_set_draw_args (EdDrawArgument * drawargs,size_t size)
{
	drawargs[28].f   = (float)throttle;
	drawargs[29].f   = (float)throttle;

	if (size > 616)
	{	
		drawargs[611].f = drawargs[0].f;
		drawargs[614].f = drawargs[3].f;
		drawargs[616].f = drawargs[5].f;
	}

}


void ed_fm_configure(const char * cfg_path)
{

}

// 获取头部抖动情况
double ed_fm_get_shake_amplitude()
{
	return 0;
}

// 基础的一些数据反馈例如引擎状况等
double test_gear_state = 0;
double ed_fm_get_param(unsigned index)
{
	if (index <= ED_FM_END_ENGINE_BLOCK)
	{
		switch (index)
		{
		case ED_FM_ENGINE_0_RPM:			
		case ED_FM_ENGINE_0_RELATED_RPM:	
		case ED_FM_ENGINE_0_THRUST:			
		case ED_FM_ENGINE_0_RELATED_THRUST:	
			return 0; // APU
		case ED_FM_ENGINE_1_RPM:
			return throttle * 3000;
		case ED_FM_ENGINE_1_RELATED_RPM:
			return throttle;
		case ED_FM_ENGINE_1_THRUST:
			return throttle * 5000 * 9.81;
		case ED_FM_ENGINE_1_RELATED_THRUST:
			return throttle;
		}
	}
	else if (index >= ED_FM_SUSPENSION_0_RELATIVE_BRAKE_MOMENT &&
			 index < ED_FM_OXYGEN_SUPPLY)
	{
		static const int block_size = ED_FM_SUSPENSION_1_RELATIVE_BRAKE_MOMENT - ED_FM_SUSPENSION_0_RELATIVE_BRAKE_MOMENT;
		switch (index)
		{
		case 0 * block_size + ED_FM_SUSPENSION_0_GEAR_POST_STATE:
		case 1 * block_size + ED_FM_SUSPENSION_0_GEAR_POST_STATE:
		case 2 * block_size + ED_FM_SUSPENSION_0_GEAR_POST_STATE:
			return test_gear_state;
		}
	}
	return 0;

}

// 启动数据初始化
void ed_fm_cold_start()
{

}

void ed_fm_hot_start()
{

}

void ed_fm_hot_start_in_air()
{

}

bool ed_fm_enable_debug_info()
{
	return false;
}

size_t ed_fm_debug_watch(int level, char *buffer, size_t maxlen)
{
	return  sprintf_s(buffer,maxlen,"TempParameter: %.2f", A6E::Interface.getParamValue("NoseWPOS_IND"));;
}

// 维修和损伤
void ed_fm_on_damage(int Element, double element_integrity_factor)
{

}

void ed_fm_repair()
{

}

bool ed_fm_need_to_be_repair()
{
	return false;
}

// 这俩主要暂时给航母起飞用
bool ed_fm_pop_simulation_event (ed_fm_simulation_event & out)
{
	return true;
}

	// bool ed_fm_push_simulation_event(const ed_fm_simulation_event & in) // same as pop . but function direction is reversed -> DCS will call it for your FM when ingame event occurs
bool ed_fm_push_simulation_event(const ed_fm_simulation_event & in)
{
	return true;
}