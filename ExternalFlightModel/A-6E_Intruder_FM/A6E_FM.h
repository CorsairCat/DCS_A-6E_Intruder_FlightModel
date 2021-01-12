#include "A6E_FM_API.h"
#include "../../include/FM/wHumanCustomPhysicsAPI.h"

extern "C" 
{
	/*/////////////////////////////////////////////////////////////////////////
	function of force source in body axis 
	x,y,z			  - force components in body coordinate system
	pos_x,pos_y,pos_z - position of force source in body coordinate system 

	body coordinate system system is always X - positive forward ,
											Y - positive up,
											Z - positive to right 
	*/
	A6E_FM_API void ed_fm_add_local_force(double & x,double &y,double &z,double & pos_x,double & pos_y,double & pos_z);
	A6E_FM_API void ed_fm_add_global_force(double & x,double &y,double &z,double & pos_x,double & pos_y,double & pos_z);
	
	/* same but in component form , return value bool : function will be called until return value is true
		while (ed_fm_add_local_force_component(x,y,z,pos_xpos_y,pos_z))
		{
			--collect 
		}
		ed_fm_add_local_force_component
		ed_fm_add_global_force_component
	*/
	A6E_FM_API bool ed_fm_add_local_force_component(double & x,double &y,double &z,double & pos_x,double & pos_y,double & pos_z);
	A6E_FM_API bool ed_fm_add_global_force_component(double & x,double &y,double &z,double & pos_x,double & pos_y,double & pos_z);

	A6E_FM_API void ed_fm_add_global_moment(double & x,double &y,double &z);
	A6E_FM_API void ed_fm_add_local_moment(double & x,double &y,double &z);
	
	/* same but in component form , return value bool : function will be called until return value is true
	while (ed_fm_add_local_moment_component(x,y,z))
	{
		--collect 
	}
	ed_fm_add_local_moment_component
	ed_fm_add_global_moment_component
	*/
	A6E_FM_API bool ed_fm_add_local_moment_component(double & x,double &y,double &z);
	
	A6E_FM_API bool ed_fm_add_global_moment_component(double & x,double &y,double &z);

	A6E_FM_API void ed_fm_simulate(double dt);

	A6E_FM_API void ed_fm_set_atmosphere(double h,//altitude above sea level
							 double t,//current atmosphere temperature , Kelwins
							 double a,//speed of sound
							 double ro,// atmosphere density
							 double p,// atmosphere pressure
							 double wind_vx,//components of velocity vector, including turbulence in world coordinate system
							 double wind_vy,//components of velocity vector, including turbulence in world coordinate system
							 double wind_vz //components of velocity vector, including turbulence in world coordinate system
							);
	/*
	called before simulation to set up your environment for the next step
	*/
	A6E_FM_API void ed_fm_set_current_mass_state (double mass,
										double center_of_mass_x,
										double center_of_mass_y,
										double center_of_mass_z,
										double moment_of_inertia_x,
										double moment_of_inertia_y,
										double moment_of_inertia_z
										);
	/*
	called before simulation to set up your environment for the next step
	*/
	A6E_FM_API void ed_fm_set_current_state (double ax,//linear acceleration component in world coordinate system
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
													   );
	/*
	additional state information in body axis

	ed_fm_set_current_state_body_axis
	*/
	A6E_FM_API void ed_fm_set_current_state_body_axis(double ax,//linear acceleration component in body coordinate system
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
															);
	/*
	input handling
	*/
	A6E_FM_API void ed_fm_set_command (int command,
							 float value);
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
	A6E_FM_API bool ed_fm_change_mass  (double & delta_mass,
												double & delta_mass_pos_x,
												double & delta_mass_pos_y,
												double & delta_mass_pos_z,
												double & delta_mass_moment_of_inertia_x,
												double & delta_mass_moment_of_inertia_y,
												double & delta_mass_moment_of_inertia_z
												);
	/*
		set internal fuel volume , init function, called on object creation and for refueling , 
		you should distribute it inside at different fuel tanks
	*/
	A6E_FM_API void   ed_fm_set_internal_fuel(double fuel);
	/*
		get internal fuel volume 
	*/
	A6E_FM_API double ed_fm_get_internal_fuel();
	/*
		set external fuel volume for each payload station , called for weapon init and on reload
	*/
	A6E_FM_API void  ed_fm_set_external_fuel (int	 station,
														double fuel,
														double x,
														double y,
														double z);
	/*
		get external fuel volume 
	*/
	A6E_FM_API double ed_fm_get_external_fuel();

	A6E_FM_API double ed_fm_refueling_add_fuel(double fuel);
	
	A6E_FM_API void ed_fm_set_draw_args (EdDrawArgument * drawargs,size_t size);
	A6E_FM_API void ed_fm_configure		(const char * cfg_path);


	A6E_FM_API double ed_fm_get_param(unsigned index);

	/*
	call backs for diffrenrt starting conditions
	*/
	A6E_FM_API void ed_fm_cold_start();
	A6E_FM_API void ed_fm_hot_start();
	A6E_FM_API void ed_fm_hot_start_in_air();

	// debug functions
	A6E_FM_API bool ed_fm_enable_debug_info();
	A6E_FM_API size_t ed_fm_debug_watch(int level, char *buffer, size_t maxlen);
	
	// damage and repair functions
	A6E_FM_API void ed_fm_on_damage(int Element, double element_integrity_factor);
	A6E_FM_API void ed_fm_repair();
	A6E_FM_API bool ed_fm_need_to_be_repair();

	// shake of head simulation
	A6E_FM_API double ed_fm_get_shake_amplitude();

	A6E_FM_API bool ed_fm_pop_simulation_event (ed_fm_simulation_event & out);

	// bool ed_fm_push_simulation_event(const ed_fm_simulation_event & in) // same as pop . but function direction is reversed -> DCS will call it for your FM when ingame event occurs
	A6E_FM_API bool ed_fm_push_simulation_event(const ed_fm_simulation_event & in);

	// suspension feed back from main
	A6E_FM_API void ed_fm_suspension_feedback(int index, const ed_fm_suspension_info * info);
};