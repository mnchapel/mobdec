/// @file   debug.h
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#pragma once

#ifdef NDEBUG
	#define DEBUG_MSG(str) do { std::cout << str << std::endl; } while(false)
	#define DEBUG_COMPONENT_MSG(str) do { std::cout << "\t" << str << std::endl; } while (false)
	//#define DEBUG_WRITE_RES() do {}
#else
	#define DEBUG_MSG(str) do { } while(false)
	#define DEBUG_COMPONENT_MSG(str) do { } while(false)
#endif
