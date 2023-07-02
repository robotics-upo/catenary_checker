#ifndef GET_PARABLE_PARAMETERS_HPP
#define GET_PARABLE_PARAMETERS_HPP

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>

#include "catenary_checker/bisection_catenary_3D.h"
#include "catenary_checker/solve_parable_parameter.hpp"

using namespace std;

struct points_2D
{
    double x;
    double y;
};

struct parable_parameters
{
    double p;
    double q;
    double r;
};

class GetParableParameter
{
	public:
		GetParableParameter(vector<geometry_msgs::Vector3> v_p_init_ugv_, vector<geometry_msgs::Vector3> v_p_init_uav_, 
							vector<float> &v_l_cat_init_, vector<geometry_msgs::Quaternion> v_init_r_ugv_, geometry_msgs::TransformStamped p_reel_local_);
		// ~GetParableParameter(){};
		virtual void ParametersParable();
		virtual void ComputeParableArea(geometry_msgs::Vector3 p1_, geometry_msgs::Vector3 p2_, double length_);
		virtual geometry_msgs::Vector3 getReelPoint(const float px_, const float py_, const float pz_,
													const float qx_, const float qy_, const float qz_, const float qw_);	
		virtual std::vector<float> checkCatenaryLength(std::vector<float> v_l_in);
		

		vector<geometry_msgs::Vector3> v_p_ugv, v_p_uav;
		vector<geometry_msgs::Quaternion> v_r_ugv;
		std::vector <points_2D> v_pts_A_2D, v_pts_B_2D;
		std::vector <parable_parameters> v_parable_params;

		std::vector<float> vec_len_cat_init;
		std::vector<double> vec_areas;
		geometry_msgs::TransformStamped pose_reel_local;
		double param_p, param_q, param_r;

	protected:

	private:
		
};

inline GetParableParameter::GetParableParameter(vector<geometry_msgs::Vector3> v_p_init_ugv_, vector<geometry_msgs::Vector3> v_p_init_uav_, 
												vector<float> &v_l_cat_init_, vector<geometry_msgs::Quaternion> v_init_r_ugv_, 
												geometry_msgs::TransformStamped p_reel_local_)
{
	v_p_ugv.clear();
	v_p_uav.clear();
	vec_len_cat_init.clear();
	v_r_ugv.clear();

	pose_reel_local = p_reel_local_;

	v_p_ugv = v_p_init_ugv_;
	v_p_uav = v_p_init_uav_;
	v_r_ugv = v_init_r_ugv_;

	vec_len_cat_init = checkCatenaryLength(v_l_cat_init_);
}

inline void GetParableParameter::ParametersParable()
{
	ParableParametersSolver PPS;
	geometry_msgs::Vector3 p_reel_;
	vec_areas.clear(); v_pts_A_2D.clear(); v_pts_B_2D.clear(); v_parable_params.clear();

	parable_parameters parable_params_;

	for(size_t i = 0 ; i < v_p_ugv.size() ; i++){
		p_reel_ = getReelPoint(v_p_ugv[i].x, v_p_ugv[i].y, v_p_ugv[i].z, v_r_ugv[i].x, v_r_ugv[i].y, v_r_ugv[i].z, v_r_ugv[i].w);
		std::cout << "Cat ["<< i << "] " << std::endl;
		ComputeParableArea(p_reel_, v_p_uav[i], vec_len_cat_init[i]);
		PPS.solve(v_pts_A_2D[i].x, v_pts_A_2D[i].y, v_pts_B_2D[i].x, v_pts_B_2D[i].y, vec_areas[i]);
		PPS.getParableParameters(param_p, param_q, param_r);
		parable_params_.p = param_p;
		parable_params_.q = param_q;
		parable_params_.r = param_r;
		v_parable_params.push_back(parable_params_);
	}
} 

inline void GetParableParameter::ComputeParableArea(geometry_msgs::Vector3 p1_, geometry_msgs::Vector3 p2_, double length_)
{
	bisectionCatenary bc;
    double x1_, x2_, y1_, y2_, Y_1_, Y_2_, area_;
	points_2D pt_A_2D_, pt_B_2D_;

	// Here are set the 3D tie-point and length to compute the parameter of each catenary in a plane
	bc.configBisection(length_, p1_.x, p1_.y, p1_.z, p2_.x, p2_.y, p2_.z);
	x1_ = 0.0;
	x2_ = bc.XB;
    y1_ = (bc.c_value * cosh((x1_ - bc.Xc)/bc.c_value)+ (bc.Yc - bc.c_value)); // evalute CatenaryChain
    y2_ = (bc.c_value * cosh((x2_ - bc.Xc)/bc.c_value)+ (bc.Yc - bc.c_value)); // evalute CatenaryChain

	// Compute the Integral 2D of catenary:   f(x) = (bc.c_value * cosh((x - bc.Xc)/bc.c_value)+ (bc.Yc - bc.c_value)); 
    Y_1_ = (bc.c_value * bc.c_value * sinh((x1_ - bc.Xc)/bc.c_value)+ (bc.Yc - bc.c_value)*x1_); 
    Y_2_ = (bc.c_value * bc.c_value * sinh((x2_ - bc.Xc)/bc.c_value)+ (bc.Yc - bc.c_value)*x2_); 
	area_ = Y_2_ - Y_1_;
	pt_A_2D_.x = x1_;
	pt_A_2D_.y = y1_;
	pt_B_2D_.x = x2_;
	pt_B_2D_.y = y2_;
	std::cout << "   Y_1_= " << Y_1_ << " , Y_2_= " << Y_2_ << " , area = " << area_<<  std::endl;
	vec_areas.push_back(area_);
	v_pts_A_2D.push_back(pt_A_2D_);
	v_pts_B_2D.push_back(pt_B_2D_);
} 


inline geometry_msgs::Vector3 GetParableParameter::getReelPoint(const float px_, const float py_, const float pz_,const float qx_, const float qy_, const float qz_, const float qw_)
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

inline std::vector<float> GetParableParameter::checkCatenaryLength(std::vector<float> v_l_in){

	geometry_msgs::Vector3 p_reel_;
	std::vector<float>  v_l_out;
	v_l_out.clear();

	for(size_t i = 0 ; i < v_l_in.size() ; i++){
		p_reel_ = getReelPoint(v_p_ugv[i].x , v_p_ugv[i].y , v_p_ugv[i].z,
                           	   v_r_ugv[i].x, v_r_ugv[i].y, v_r_ugv[i].z, v_r_ugv[i].w);
		double d_ = sqrt(pow(v_p_uav[i].x - p_reel_.x,2) + 
						 pow(v_p_uav[i].y - p_reel_.y,2) + 
						 pow(v_p_uav[i].z - p_reel_.z,2));
		if (d_ > v_l_in[i]){
			v_l_out.push_back(d_ * 1.001);
			// printf("[%lu] d= %f , L = %f , new= %f \n",i, d_, v_l_in[i], d_ * 1.001);
		}
		else
			v_l_out.push_back(v_l_in[i]);
	}
	return v_l_out;
}

#endif