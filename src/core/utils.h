/// @file   utils.h
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#pragma once



// MoBDec
#include <core/settings.h>



namespace utils {

/// @brief
///
/// @param val
///
/// @return the index at time t.
uint indexAtT(uint val) noexcept;



/// @brief
///
/// @param val:
///
/// @return the index at time t-1.
uint indexAtTM1(uint val) noexcept;



/// @brief
///
/// @param val:
///
/// @return the index at time t-delta t.
uint indexAtTMDelta(uint val) noexcept;

}
