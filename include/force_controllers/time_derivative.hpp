
#ifndef TIME_DERIVATIVE_HPP
#define TIME_DERIVATIVE_HPP
#include <cstddef>
#include <vector>
#include <string>
#include <fstream>
#include <stdio.h>
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
			return 0;
		}
		var_type der = (var_type)(var - previous_variable_ / (time - previous_time_));
		previous_time_ = time;
		previous_variable_ = var;
		return der;
	}
};
#endif