
#ifndef TIME_DERIVATIVE_HPP
#define TIME_DERIVATIVE_HPP
#include <cstddef>
#include <vector>
#include <string>
#include <fstream>
#include <stdio.h>
#include <eigen3/Eigen/Dense>
#include <Eigen/Dense>
#include <math.h>
template <typename var_type, typename time_type>
class timeDerivative
{
private:
	var_type previous_variable_;
	time_type previous_time_;
	bool _is_first_time;

public:
	timeDerivative()
	{
		_is_first_time = true;
        std::cout<<"Time Derivative Class Initialized!"<<std::endl;
	}
	var_type getDerivative(var_type var, time_type time)
	{
		if (_is_first_time)
		{
			_is_first_time = false;
			previous_time_ = time;
			previous_variable_ = var;
			// if (std::is_same<var_type, Eigen::Matrix<double,-1,-1>>::value || std::is_same<var_type, Eigen::Matrix<float,-1,-1>>::value)
			// 	return var.Zero();
			// else
				return var_type{};
			
		}
		var_type der = (var_type)(var - previous_variable_ / (time - previous_time_));
		std::cout<<"DT DERIVATIVE: "<<(time-previous_time_)<<std::endl;
		std::cout<<"DT DERIVATIVE: "<<(time-previous_time_)<<std::endl;
		previous_time_ = time;
		previous_variable_ = var;
		return der;
	}
};
#endif