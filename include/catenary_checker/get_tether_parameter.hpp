#ifndef GET_TETHER_PARAMETERS_HPP
#define GET_TETHER_PARAMETERS_HPP

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>

#include "catenary_checker/bisection_catenary_3D.h"
#include "catenary_checker/solve_parabola_parameter.hpp"
#include "catenary_checker/solve_catenary_parameter.hpp"

using namespace std;

struct points_2D
{
    double x;
    double y;
};

struct parabola_parameters
{
    double p;
    double q;
    double r;
};

struct catenary_parameters
{
    double x0;
    double y0;
    double a;
};

struct tether_parameters
{
    double a;
    double b;
    double c;
};


class GetTetherParameter
{
	public:
		GetTetherParameter();
		GetTetherParameter(vector<geometry_msgs::Vector3> v_p_init_ugv_, vector<geometry_msgs::Vector3> v_p_init_uav_, 
							vector<float> &v_l_cat_init_, vector<geometry_msgs::Quaternion> v_init_r_ugv_, 
							geometry_msgs::TransformStamped p_reel_local_, vector <tether_parameters> v_tether_init_param_, float ws_z_max_);
		GetTetherParameter(vector<geometry_msgs::Vector3> v_p_init_ugv_, vector<geometry_msgs::Vector3> v_p_init_uav_, 
						   vector<float> &v_l_cat_init_, vector<geometry_msgs::Quaternion> v_init_r_ugv_, geometry_msgs::TransformStamped p_reel_local_);

		// ~GetTetherParameter(){};
		virtual void ParametersParabola(vector<float> v_param_x_, vector<float> v_param_y_, vector<float> v_param_c_);
		virtual void ParametersCatenary();
		virtual void ComputeCatenaryArea(geometry_msgs::Vector3 p1_, geometry_msgs::Vector3 p2_, double length_, float x0_, float y0_, float c_);
		virtual geometry_msgs::Vector3 getReelPoint(const float px_, const float py_, const float pz_,
													const float qx_, const float qy_, const float qz_, const float qw_);	
		virtual void getParabolaInPlane(geometry_msgs::Vector3 v1_, geometry_msgs::Vector3 v2_, points_2D &pA_, points_2D &pB_);
    	virtual void getPointParabolaStraight(geometry_msgs::Vector3 p1_, geometry_msgs::Vector3 p2_, vector<geometry_msgs::Vector3> &v_p_, float length_);
		virtual void getParabolaPoints(geometry_msgs::Vector3 p1_, geometry_msgs::Vector3 p2_, tether_parameters param_, vector<geometry_msgs::Vector3> &v_p_);
		virtual void getCatenaryPoints(geometry_msgs::Vector3 p1_, geometry_msgs::Vector3 p2_, tether_parameters param_, vector<geometry_msgs::Vector3> &v_p_, float length_);

		

		vector<geometry_msgs::Vector3> v_p_ugv, v_p_uav;
		vector<geometry_msgs::Quaternion> v_r_ugv;
		std::vector <points_2D> v_pts_A_2D, v_pts_B_2D;
		std::vector <tether_parameters> v_tether_params, v_tether_init_params;

		std::vector<float> vec_len_cat_init;
		std::vector<double> vec_areas;
		geometry_msgs::TransformStamped pose_reel_local;
		/*
		Map:
		 For Catenary :
		 	param_a = x0, param_b = y0, param_c = a;
		 For Prarable :
		 	param_a = p,  param_b = q,  param_c = r;
		*/
		
		double param_a, param_b, param_c;

		int num_point_per_unit_length;
		double Length_1, Length_2;
		float ws_z_max;

	protected:

	private:
		
};

inline GetTetherParameter::GetTetherParameter(){}

inline GetTetherParameter::GetTetherParameter(vector<geometry_msgs::Vector3> v_p_init_ugv_, vector<geometry_msgs::Vector3> v_p_init_uav_, 
											vector<float> &v_l_cat_init_, vector<geometry_msgs::Quaternion> v_init_r_ugv_, 
											geometry_msgs::TransformStamped p_reel_local_)
{
	v_p_ugv.clear();
	v_p_uav.clear();
	vec_len_cat_init.clear();
	v_r_ugv.clear();
	v_tether_init_params.clear();

	pose_reel_local = p_reel_local_;

	v_p_ugv = v_p_init_ugv_;
	v_p_uav = v_p_init_uav_;
	v_r_ugv = v_init_r_ugv_;
	vec_len_cat_init = v_l_cat_init_;
}

inline GetTetherParameter::GetTetherParameter(vector<geometry_msgs::Vector3> v_p_init_ugv_, vector<geometry_msgs::Vector3> v_p_init_uav_, 
											vector<float> &v_l_cat_init_, vector<geometry_msgs::Quaternion> v_init_r_ugv_, 
											geometry_msgs::TransformStamped p_reel_local_, vector <tether_parameters> v_tether_init_param_, float ws_z_max_)
{
	v_p_ugv.clear();
	v_p_uav.clear();
	vec_len_cat_init.clear();
	v_r_ugv.clear();
	v_tether_init_params.clear();

	pose_reel_local = p_reel_local_;

	v_p_ugv = v_p_init_ugv_;
	v_p_uav = v_p_init_uav_;
	v_r_ugv = v_init_r_ugv_;
	vec_len_cat_init = v_l_cat_init_;
	v_tether_init_params = v_tether_init_param_;
	ws_z_max = ws_z_max_;
}

inline void GetTetherParameter::ParametersCatenary()
{
	CatenaryParametersSolver CPS;
	geometry_msgs::Vector3 p_reel_;
	vec_areas.clear(); v_pts_A_2D.clear(); v_pts_B_2D.clear(); v_tether_params.clear();

	tether_parameters catenary_params_;

	for(size_t i = 0 ; i < v_p_ugv.size() ; i++){
		// std::cout << "		***** 		["<< i <<"]:"<< std::endl;
		p_reel_ = getReelPoint(v_p_ugv[i].x, v_p_ugv[i].y, v_p_ugv[i].z, v_r_ugv[i].x, v_r_ugv[i].y, v_r_ugv[i].z, v_r_ugv[i].w);
		CPS.loadInitialSolution(i, v_tether_init_params[i].a, v_tether_init_params[i].b, v_tether_init_params[i].c);
		double d_ = sqrt(pow(p_reel_.x-v_p_uav[i].x,2)+pow(p_reel_.y-v_p_uav[i].y,2));
		float dist_ = 1.01* sqrt(pow(p_reel_.x-v_p_uav[i].x,2)+pow(p_reel_.y-v_p_uav[i].y,2)+pow(p_reel_.z-v_p_uav[i].z,2));
		CPS.solve(0, p_reel_.z, d_, v_p_uav[i].z, vec_len_cat_init[i], dist_,20.0, ws_z_max);
		CPS.getCatenaryParameters(param_a, param_b, param_c);
		catenary_params_.a = param_a;
		catenary_params_.b = param_b;
		catenary_params_.c = param_c;
		Length_1 = param_c * sinh((param_a)/param_c) + param_c * sinh((d_-param_a)/param_c) ;
		Length_2 = param_c * sinh((d_-param_a)/param_c) - param_c * sinh((0.0-param_a)/param_c) ;
		v_tether_params.push_back(catenary_params_);
	}
} 

inline void GetTetherParameter::ParametersParabola(vector<float> v_param_x_, vector<float> v_param_y_, vector<float> v_param_c_)
{
	ParabolaParametersSolver PPS;
	geometry_msgs::Vector3 p_reel_;
	vec_areas.clear(); v_pts_A_2D.clear(); v_pts_B_2D.clear(); v_tether_params.clear();

	tether_parameters parabola_params_;

	for(size_t i = 0 ; i < v_p_ugv.size() ; i++){
		p_reel_ = getReelPoint(v_p_ugv[i].x, v_p_ugv[i].y, v_p_ugv[i].z, v_r_ugv[i].x, v_r_ugv[i].y, v_r_ugv[i].z, v_r_ugv[i].w);
		ComputeCatenaryArea(p_reel_, v_p_uav[i], vec_len_cat_init[i], v_param_x_[i], v_param_y_[i], v_param_c_[i]);
		PPS.loadInitialSolution(0.0, 0.01, p_reel_.z);
		double d_ = sqrt(pow(p_reel_.x-v_p_uav[i].x,2)+pow(p_reel_.y-v_p_uav[i].y,2));
		PPS.solve(0.0, p_reel_.z, d_, v_p_uav[i].z, vec_areas[i], i );
		PPS.getParabolaParameters(param_a, param_b, param_c);
		parabola_params_.a = param_a;
		parabola_params_.b = param_b;
		parabola_params_.c = param_c;
		v_tether_params.push_back(parabola_params_);
	}
} 

inline void GetTetherParameter::ComputeCatenaryArea(geometry_msgs::Vector3 p1_, geometry_msgs::Vector3 p2_, double length_, float x0_, float y0_, float c_)
{
    double x1_, x2_, y1_, y2_, Y_1_, Y_2_, area_;
	points_2D pt_A_2D_, pt_B_2D_;

	// Here are set the 3D tie-point and length to compute the parameter of each catenary in a plane
	double d_ = sqrt((p2_.x -p1_.x)*(p2_.x -p1_.x)+(p2_.y -p1_.y)*(p2_.y -p1_.y));
	x1_ = 0.0;
	x2_ = d_;
    y1_ = (c_ * cosh((x1_ - x0_)/c_)+ (y0_ - c_)); // evalute CatenaryChain
    y2_ = (c_ * cosh((x2_ - x0_)/c_)+ (y0_ - c_)); // evalute CatenaryChain

	// Compute the Integral 2D of catenary:   f(x) = (c_ * cosh((x - x0_)/c_)+ (y0_ - c_)); 
    Y_1_ = (c_ * c_ * sinh((x1_ - x0_)/c_)+ (y0_ - c_)*x1_); 
    Y_2_ = (c_ * c_ * sinh((x2_ - x0_)/c_)+ (y0_ - c_)*x2_); 
	area_ = Y_2_ - Y_1_;
	getParabolaInPlane(p1_, p2_, pt_A_2D_, pt_B_2D_);
	vec_areas.push_back(area_);
	v_pts_A_2D.push_back(pt_A_2D_);
	v_pts_B_2D.push_back(pt_B_2D_);
} 


inline geometry_msgs::Vector3 GetTetherParameter::getReelPoint(const float px_, const float py_, const float pz_,const float qx_, const float qy_, const float qz_, const float qw_)
{
	geometry_msgs::Vector3 ret;

	double roll_, pitch_, yaw_;
	tf::Quaternion q_(qx_,qy_,qz_,qw_);
	tf::Matrix3x3 M_(q_);	
	M_.getRPY(roll_, pitch_, yaw_);

	double lengt_vec =  sqrt(pose_reel_local.transform.translation.x*pose_reel_local.transform.translation.x + pose_reel_local.transform.translation.y*pose_reel_local.transform.translation.y);
	ret.x = px_ + lengt_vec *cos(yaw_); 
	ret.y = py_ + lengt_vec *sin(yaw_);
	ret.z = pz_ + pose_reel_local.transform.translation.z ;

	return ret;
}

inline void GetTetherParameter::getParabolaInPlane(geometry_msgs::Vector3 v1_, geometry_msgs::Vector3 v2_, points_2D &pA_, points_2D &pB_){
	geometry_msgs::Vector3 vd, va;
	double _A, _B, _C, _d0 , _d1;
	_d0 =_d1 = 0.0;
	// First get General Plane Ecuation: A(x-x1) + B(y-y1) + C(z-z1) = 0
	vd.x = v2_.x -v1_.x;
	vd.y = v2_.y -v1_.y;
	vd.z = v2_.z -v1_.z;
	va.x = v2_.x + 1.0;
	va.y = v2_.y ;
	va.z = v2_.z ;
	_A = vd.y * va.z - vd.z * va.y;
	_B = vd.z * va.x - vd.x * va.z;
	_C = vd.x * va.y - vd.y * va.x;
	// Second get intersection in axes (X and Y)
	double x_axe, y_axe;
	if(_A > 0.00001 & _A < -0.00001){
		x_axe = (_A*v1_.x +_B*v1_.y + _C*v1_.z)/_A;
		if (x_axe * v1_.x > 0)
			_d0 = sqrt( pow(v1_.x - x_axe , 2) + pow(v1_.y - 0.0 , 2));
	}
	if(_B > 0.00001 & _B < -0.00001){
		y_axe = (_A*v1_.x +_B*v1_.y + _C*v1_.z)/_B;
		if (y_axe * v1_.y > 0)
			_d0 = sqrt( pow(v1_.x - 0.0 , 2) + pow(v1_.y - y_axe , 2));
	}
	
	_d1 = sqrt( pow(v2_.x - v1_.x , 2) + pow(v2_.y - v1_.y , 2));
	pA_.x = _d0;
	pA_.y = v1_.z;
	pB_.x = _d0 + _d1;
	pB_.y = v2_.z;
}

inline void GetTetherParameter::getParabolaPoints(geometry_msgs::Vector3 p1_, geometry_msgs::Vector3 p2_, tether_parameters param_, 
													std::vector<geometry_msgs::Vector3> &v_p_)
{
  	num_point_per_unit_length = 20;
	geometry_msgs::Vector3 p_;
	int	num_point_catenary;
	double d_, x_, y_, tetha_, dx_, dy_, dz_, dxy_;
	d_ = x_ = y_ = 0.0;
  	v_p_.clear();

	dx_ = (p2_.x - p1_.x);
	dy_ = (p2_.y - p1_.y);
	dz_ = (p2_.z - p1_.z);
	d_ = sqrt(dx_*dx_ + dy_*dy_ + dz_*dz_);
	dxy_ = sqrt(dx_* dx_ + dy_* dy_ );
	
	if (d_ < 1.0)
		num_point_catenary = ceil( (double)num_point_per_unit_length * 1.0);
	else
		num_point_catenary = ceil( (double)num_point_per_unit_length * d_);

	double u_x = dx_ /dxy_;
	double u_y = dy_ /dxy_;

	if(dxy_ > 0.001){ 
	// std::cout << "getParabolaPoints :  NOT STRAIGHT" << std::endl;
		for(int i = 0; i < num_point_catenary; i++){
			x_ = x_ + (dxy_/(double)num_point_catenary);
			p_.x = p1_.x + u_x * x_;
			p_.y = p1_.y + u_y * x_;
			p_.z = param_.a * x_* x_ + param_.b * x_ + param_.c;
      		v_p_.push_back(p_);
		}		
	}
	else{
	// std::cout << "getParabolaPoints :  STRAIGHT d[" << dx_ << "," << dy_<< "," << dz_ <<"] dist["<< d_ << "," << dxy_<<"]"<<std::endl;
		getPointParabolaStraight(p1_, p2_, v_p_, d_);
	}
}

inline void GetTetherParameter::getCatenaryPoints(geometry_msgs::Vector3 p1_, geometry_msgs::Vector3 p2_, tether_parameters param_, 
												  vector<geometry_msgs::Vector3> &v_p_, float length_)
{
  	num_point_per_unit_length = 20;
	geometry_msgs::Vector3 p_;
	int	num_point_catenary;
	double d_, x_, u_x , u_y;
	d_ = x_ = 0.0;

  	v_p_.clear();
	
	double delta_x = (p2_.x - p1_.x);
	double delta_y = (p2_.y - p1_.y);
	double delta_z = (p2_.z - p1_.z);
	double dxy_ = sqrt(delta_x * delta_x + delta_y * delta_y );
	double dist_= sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z );
	d_ = dist_;
	// d_ = length_;
	if (dxy_ < 0.0001){
		u_x = u_y = 0.0;
	} else{
		u_x = delta_x /dxy_;
		u_y = delta_y /dxy_;
	}
	
	if (d_ < 1.0)
		num_point_catenary = ceil( (double)num_point_per_unit_length * 1.0);
	else
		num_point_catenary = ceil( (double)num_point_per_unit_length * d_);

	if (!(dxy_ < 0.0001) && param_.c < 1000.0){ 
		for(int i = 0; i < num_point_catenary; i++ ){
			p_.x = p1_.x + u_x * x_;
			p_.y = p1_.y + u_y * x_;
			p_.z = param_.c * cosh((x_ - param_.a)/param_.c) + (param_.b - param_.c);

			if (p_.z > p2_.z){	// This condition is to stop the parabolas vector points when parabola parameter doesnt cut the extreme points
				break;
			}
			
      		v_p_.push_back(p_);
			x_ += dxy_/ num_point_catenary;  
		}	
		if(v_p_.size() < 1){
			std::cout << "						Is v_p_.size() < 1  so : d_=["<< d_ <<"]length_=["<< length_ <<"] , num_point_catenary =[" << num_point_catenary << "]" << std::endl;
		}	
	}
	else{
		getPointParabolaStraight(p1_, p2_, v_p_, length_);
	}
}

inline void GetTetherParameter::getPointParabolaStraight(geometry_msgs::Vector3 p1_, geometry_msgs::Vector3 p2_, std::vector<geometry_msgs::Vector3> &v_p_, float length_)
{
	v_p_.clear();
	double dx_ = p2_.x - p1_.x ;
	double dy_ = p2_.y - p1_.y ;
	double dz_ = p2_.z - p1_.z ;
	int num_point_catenary;
	if (length_ < 1.0)
		num_point_catenary = ceil(10.0); //d_ * 10.0
	else
		num_point_catenary = ceil( (double)num_point_per_unit_length * length_);

    double x_step = dx_ / (double) num_point_catenary;
    double y_step = dy_ / (double) num_point_catenary;
    double z_step = dz_ / (double) num_point_catenary;

    for(int i=0; i < num_point_catenary ; i++)
    {       
        geometry_msgs::Vector3 p_;

        p_.x = (p1_.x + x_step* (double)i);
        p_.y = (p1_.y + y_step* (double)i);
        p_.z = (p1_.z + z_step* (double)i);    
        v_p_.push_back(p_);
    }
}

#endif