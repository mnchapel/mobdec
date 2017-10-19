/// @file   mVariant.h
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#pragma once



// C++
#include <iostream>



class MVariant {
private:
	union {
		// POD data types
		int i;
		float f;
		double d;
		// All non-POD data types
		void* v;
	} data;

	enum DataType {Integer, Float, Double, NonPod};
	int type = 0;

public:

	/// @brief Default constructor
	MVariant() = default;



	/// @brief Constructor for integer data type
	///
	/// @param i
	MVariant(int i)
	{
		data.i = i;
		type = Integer;
	}



	MVariant(float f)
	{
		data.f = f;
		type = Float;
	}



	MVariant(double d)
	{
		data.d = d;
		type = Double;
	}


	template <typename T>
	MVariant(const T& t)
	{
		data.v = (void*) new T(t);
		type = NonPod;
	}



	bool isInt() const { return type == Integer; }
	bool isFloat() const { return type == Float; }
	bool isDouble() const { return type == Double; }
	bool isNonPod() const {return type == NonPod; }



	operator int() const {return data.i;}
	operator float() const {return data.f;}
	operator double() const {return data.d;}
	operator std::string() const {return *static_cast<std::string*>(data.v);}
};
