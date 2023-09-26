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
		GetParableParameter();
		GetParableParameter(vector<geometry_msgs::Vector3> v_p_init_ugv_, vector<geometry_msgs::Vector3> v_p_init_uav_, 
							vector<float> &v_l_cat_init_, vector<geometry_msgs::Quaternion> v_init_r_ugv_, geometry_msgs::TransformStamped p_reel_local_);
		// ~GetParableParameter(){};
		virtual void ParametersParable();
		virtual void ComputeParableArea(geometry_msgs::Vector3 p1_, geometry_msgs::Vector3 p2_, double length_);
		virtual geometry_msgs::Vector3 getReelPoint(const float px_, const float py_, const float pz_,
													const float qx_, const float qy_, const float qz_, const float qw_);	
		virtual void getParableInPlane(geometry_msgs::Vector3 v1_, geometry_msgs::Vector3 v2_, points_2D &pA_, points_2D &pB_);
		virtual void getParablePoints(geometry_msgs::Vector3 p1_, geometry_msgs::Vector3 p2_, parable_parameters param_, std::vector<geometry_msgs::Vector3> &v_p_);
    	virtual void getPointParableStraight(geometry_msgs::Vector3 p1_, geometry_msgs::Vector3 p2_, std::vector<geometry_msgs::Vector3> &v_p_);
    	virtual void checkStateParable(geometry_msgs::Vector3 p1_, geometry_msgs::Vector3 p2_);
		// virtual void compute3DParable(vector<geometry_msgs::Vector3> &v_p_, 
		// 							  geometry_msgs::Vector3 p1_, 
		// 							  geometry_msgs::Vector3 p2_, 
		// 							  double p, 
		// 							  double q, 
		// 							  double r);
		

		vector<geometry_msgs::Vector3> v_p_ugv, v_p_uav;
		vector<geometry_msgs::Quaternion> v_r_ugv;
		std::vector <points_2D> v_pts_A_2D, v_pts_B_2D;
		std::vector <parable_parameters> v_parable_params;

		std::vector<float> vec_len_cat_init;
		std::vector<double> vec_areas;
		geometry_msgs::TransformStamped pose_reel_local;
		double param_p, param_q, param_r;

		bool x_const, y_const, z_const;
    	double _direc_x , _direc_y, _direc_z;

		int num_point_per_unit_length;

	protected:

	private:
		
};

inline GetParableParameter::GetParableParameter(){}

inline GetParableParameter::GetParableParameter(vector<geometry_msgs::Vector3> v_p_init_ugv_, 
												vector<geometry_msgs::Vector3> v_p_init_uav_, 
												vector<float> &v_l_cat_init_, 
												vector<geometry_msgs::Quaternion> v_init_r_ugv_, 
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
	vec_len_cat_init = v_l_cat_init_;
}

inline void GetParableParameter::ParametersParable()
{
	ParableParametersSolver PPS;
	geometry_msgs::Vector3 p_reel_;
	vec_areas.clear(); v_pts_A_2D.clear(); v_pts_B_2D.clear(); v_parable_params.clear();

	parable_parameters parable_params_;

	for(size_t i = 0 ; i < v_p_ugv.size() ; i++){
		p_reel_ = getReelPoint(v_p_ugv[i].x, v_p_ugv[i].y, v_p_ugv[i].z, v_r_ugv[i].x, v_r_ugv[i].y, v_r_ugv[i].z, v_r_ugv[i].w);
		ComputeParableArea(p_reel_, v_p_uav[i], vec_len_cat_init[i]);
		PPS.loadInitialSolution(0.3, 0.01, p_reel_.z);
		// PPS.solve(v_pts_A_2D[i].x, v_pts_A_2D[i].y, v_pts_B_2D[i].x, v_pts_B_2D[i].y, vec_areas[i]);
		double d_ = sqrt(pow(p_reel_.x-v_p_uav[i].x,2)+pow(p_reel_.y-v_p_uav[i].y,2));
		// std::cout << "Solving ["<< i <<"]" << std::endl;
		PPS.solve(0, p_reel_.z, d_, v_p_uav[i].z, vec_areas[i]);
		PPS.getParableParameters(param_p, param_q, param_r);
		parable_params_.p = param_p;
		parable_params_.q = param_q;
		parable_params_.r = param_r;
		v_parable_params.push_back(parable_params_);
	}
} 

// inline void GetParableParameter::compute3DParable(
// 						vector<geometry_msgs::Vector3> &v_p_, 
// 						geometry_msgs::Vector3 p1_, 
// 						geometry_msgs::Vector3 p2_, 
// 						double p, 
// 						double q, 
// 						double r)
// {
// 	v_p_.clear();
// 	double u_x, u_y, d_;
// 	double fix_value = 0.01;
// 	bool change_x = true; 
// 	bool change_y = true;

// 	if(p1_.x > p2_.x)
// 		u_x = 1.0;
// 	else if(p2_.x < p2_.x)
// 		u_x = -1.0;
// 	else
// 		u_x = 0.0;    
		
// 	if(p2_.y > p2_.y)
// 		u_y = 1.0;
// 	else if(p2_.y < p2_.y)
// 		u_y = -1.0;
// 	else
// 		u_y = 0.0;  
	
// 	if ((p2_.x - p2_.x) < fix_value && (p2_.x - p2_.x) > -1.0*fix_value) 
// 		change_x = false;
// 	if ((p2_.y - p2_.y) < fix_value && (p2_.y - p2_.y) > -1.0*fix_value)
// 		change_y = false;

// 	// About distance between UGV and UAV in the plane
// 	if ( !change_x && !change_y )  //Not change in X-Y plane, so parable can't be compute
// 		d_ = sqrt(pow(p2_.z-p1_.z,2)); 
// 	else
// 		d_ = sqrt(pow(p2_.x-p1_.x,2)+pow(p2_.y-p1_.y,2)); 
	
// 	int num_point_per_unit_length = 10;
// 	int np_ = round( (double)num_point_per_unit_length * d_);
	
// 	double x_  =  0.0;
// 	double tetha_ = atan((p2_.y - p2_.y)/(p2_.x - p2_.x));

// 	geometry_msgs::Vector3 point_;
// 	for(int i = 0; i < np_; i++){  
// 		if ( !change_x && !change_y ){ // To check difference position between UGV and UAV only in z-axe, so parable it is not computed
// 			double _step = d_ / (double) np_;
// 			point_.x = p1_.x;
// 			point_.y = p1_.y;
// 			point_.z = p1_.z+ _step* (double)i;    
// 		}
// 		else{ 	// In case that UGV and UAV are not in the same point the plane X-Y
// 			x_ = x_ + (d_/ np_);
// 			point_.x = p1_.x + u_x * cos(tetha_) * x_;
// 			point_.y = p1_.y + u_y * sin(tetha_) * x_;
// 			point_.z = p * x_* x_ + q * x_ + r;
// 		}
// 		v_p_.push_back(point_);
// 	}
// }

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
	getParableInPlane(p1_, p2_, pt_A_2D_, pt_B_2D_);
	// std::cout << "   Y_1_= " << Y_1_ << " , Y_2_= " << Y_2_ << " , area = " << area_<<  std::endl;
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

inline void GetParableParameter::getParableInPlane(geometry_msgs::Vector3 v1_, geometry_msgs::Vector3 v2_, points_2D &pA_, points_2D &pB_){
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

inline void GetParableParameter::getParablePoints(geometry_msgs::Vector3 p1_, geometry_msgs::Vector3 p2_, parable_parameters param_, std::vector<geometry_msgs::Vector3> &v_p_)
{
  	num_point_per_unit_length = 20;
	geometry_msgs::Vector3 p_;
	int	num_point_catenary;
	double d_, x_, y_, tetha_, dx_, dy_, dz_, dxy_;
	// double dir_x_, dir_y_;
	d_ = x_ = y_ = 0.0;

  	checkStateParable(p1_, p2_);
  	v_p_.clear();

	d_ = sqrt(pow(p1_.x - p2_.x,2) + pow(p1_.y - p2_.y,2) + pow(p1_.z - p2_.z,2));
	
	dx_ = (p2_.x - p1_.x);
	// if (dx_ < 0.0)
	// 	dir_x_ = -1.0;
	// else
	// 	dir_x_ = 1.0;
	
	dy_ = (p2_.y - p1_.y);
	// if (dy_ < 0.0)
	// 	dir_y_ = -1.0;
	// else
	// 	dir_y_ = 1.0;
	
	dz_ = (p2_.z - p1_.z);
	
	dxy_ = sqrt(pow(p1_.x - p2_.x,2) + pow(p1_.y - p2_.y,2) );
	
	if (d_ < 1.0)
		num_point_catenary = ceil(d_ * 10.0);
	else
		num_point_catenary = ceil( (double)num_point_per_unit_length * d_);
	

	// if (!x_const && !y_const || (!(dxy_ > 0.4) &&  !(dz_ > 4.0)) ){ 
	if(!x_const || !y_const){ 
		// std::cout << "	Case 1" << std::endl;
		if (x_const)
			tetha_ = 1.5707;
		else if(y_const)
			tetha_ = 0.0;
		else
			tetha_ = atan(fabs(p2_.y - p1_.y)/fabs(p2_.x - p1_.x));

		for(int i = 0; i < num_point_catenary; i++){
			x_ = x_ + (dxy_/(double)num_point_catenary);
			p_.x = p1_.x + _direc_x* cos(tetha_) * x_;
			p_.y = p1_.y + _direc_y* sin(tetha_) * x_;
			p_.z = param_.p * x_* x_ + param_.q * x_ + param_.r;
			if (p_.z > p2_.z)	// This condition is to stop the parables vector points when parable parameter doesnt cut the extreme points
				break;
      		v_p_.push_back(p_);
				// std::cout << "		Case 1: num_point_catenary=" << num_point_catenary << " dxy_=" << dxy_ << " v_p_.size()=" << v_p_.size() <<" , fabs(p2_.y - p1_.y)="<< fabs(p2_.y - p1_.y)<< " , fabs(p2_.x - p1_.x)=" << fabs(p2_.x - p1_.x) <<std::endl;
				// std::cout << "			param_=[" <<param_.p <<","<< param_.q <<","<< param_.r <<"] , tetha_=" << tetha_ << " , cos(tetha_)=" << cos(tetha_)<< " , sin(tetha_)=" << sin(tetha_) << std::endl;
		}		
	// }else if(x_const && !y_const){ 
	// 	std::cout << "	Case 2" << std::endl;
	// 	for(int i = 0; i < num_point_catenary; i++){
	// 		x_ = x_ + dir_y_* (dxy_/(double)num_point_catenary);
	// 		p_.x = (p2_.x + p1_.x)/2.0;
	// 		p_.y = p1_.y  + x_;
	// 		p_.z = param_.p * x_* x_ + param_.q * x_ + param_.r;
	// 		if (p_.z > p2_.z)	// This condition is to stop the parables vector points when parable parameter doesnt cut the extreme points
	// 			break;
    //   		v_p_.push_back(p_);
	// 			// std::cout << "		Case 2: num_point_catenary=" << num_point_catenary << " dxy_=" << dxy_ << " v_p_.size()=" << v_p_.size() <<" , fabs(p2_.y - p1_.y)="<< fabs(p2_.y - p1_.y)<< " , fabs(p2_.x - p1_.x)=" << fabs(p2_.x - p1_.x) <<std::endl;
	// 			// std::cout << "			param_=[" <<param_.p <<","<< param_.q <<","<< param_.r <<"] , tetha_=" << tetha_ << " , cos(tetha_)=" << cos(tetha_)<< " , sin(tetha_)=" << sin(tetha_) << std::endl;
	// 	}
	// }
	// else if(!x_const && y_const){ 
	// 	std::cout << "	Case 3" << std::endl;
	// 	for(int i = 0; i < num_point_catenary; i++){
	// 		x_ = x_ + dir_x_* (dxy_/(double)num_point_catenary);
	// 		p_.x = p1_.x + x_;
	// 		p_.y = (p2_.y + p1_.y)/2.0;
	// 		p_.z = param_.p * x_* x_ + param_.q * x_ + param_.r;
	// 		if (p_.z > p2_.z)	// This condition is to stop the parables vector points when parable parameter doesnt cut the extreme points
	// 			break;
    //   		v_p_.push_back(p_);
	// 			// std::cout << "		Case 3: num_point_catenary=" << num_point_catenary << " dxy_=" << dxy_ << " v_p_.size()=" << v_p_.size() <<" , fabs(p2_.y - p1_.y)="<< fabs(p2_.y - p1_.y)<< " , fabs(p2_.x - p1_.x)=" << fabs(p2_.x - p1_.x) <<std::endl;
	// 			// std::cout << "			param_=[" <<param_.p <<","<< param_.q <<","<< param_.r <<"] , tetha_=" << tetha_ << " , cos(tetha_)=" << cos(tetha_)<< " , sin(tetha_)=" << sin(tetha_) << std::endl;
	// 	}
	}
	else{
		// std::cout << "		Case 4: Straight Tether"<<std::endl;
		// std::cout << "	Case 4" << std::endl;
		getPointParableStraight(p1_, p2_, v_p_);
	}
}

inline void GetParableParameter::getPointParableStraight(geometry_msgs::Vector3 p1_, geometry_msgs::Vector3 p2_, std::vector<geometry_msgs::Vector3> &v_p_)
{
	v_p_.clear();
	double dx_ = p2_.x - p1_.x ;
	double dy_ = p2_.y - p1_.y ;
	double dz_ = p2_.z - p1_.z ;
	int num_point_catenary = round( (double)num_point_per_unit_length * fabs(dz_));

    double x_step = dx_ / (double) num_point_catenary;
    double y_step = dy_ / (double) num_point_catenary;
    double z_step = dz_ / (double) num_point_catenary;

	// printf("GetParableParameter Straigth Tether: d_=[%.3f %.3f %.3f], num_point_catenary= %i , step=[%.3f %.3f %.3f] , p1_.z=%f, num_point_per_unit_length=%i\n",dx_,dy_,dz_, num_point_catenary, x_step, y_step,z_step, p1_.z ,  num_point_per_unit_length);

    for(int i=0; i < num_point_catenary ; i++)
    {       
        geometry_msgs::Vector3 p_;

        p_.x = (p1_.x + x_step* (double)i);
        p_.y = (p1_.y + y_step* (double)i);
        p_.z = (p1_.z + z_step* (double)i);    
        v_p_.push_back(p_);
		// std::cout << "straigth parable : " << p_.x << ", " << p_.y << ", " << p_.z << std::endl;
    }
}

inline void GetParableParameter::checkStateParable(geometry_msgs::Vector3 p1_, geometry_msgs::Vector3 p2_)
{
  	x_const = y_const = z_const = false;

	if( fabs(p2_.x - p1_.x) <  0.01)
			x_const = true;
	if( fabs(p2_.y - p1_.y) <  0.01)
			y_const = true;
	if (sqrt(pow(p2_.x - p1_.x,2) + pow(p2_.y - p1_.y,2)) < 0.25 && fabs(p2_.z - p1_.z) > 0.4)
		x_const = y_const= true;

  	if( fabs(p2_.z - p1_.z) <  0.01)
        z_const = true;

	if(p2_.x > p1_.x)
			_direc_x = 1.0;
	else if(p2_.x < p1_.x)
			_direc_x = -1.0;
	else if(p2_.x == p1_.x)
			_direc_x = 0.0;    
	
	if(p2_.y > p1_.y)
			_direc_y = 1.0;
	else if(p2_.y < p1_.y)
			_direc_y = -1.0;
	else if(p2_.y == p1_.y)
			_direc_y = 0.0;    
}



#endif