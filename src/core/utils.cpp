/// @file   utils.cpp
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#include <core/utils.h>


namespace utils {



//---------------------------------------------------------------------------------------
uint indexAtT(uint val) noexcept
{
    return (val-1)%CST(int,cst::NB_ROWS);
}



//---------------------------------------------------------------------------------------
uint indexAtTM1(uint val) noexcept
{
    return (val-1+CST(int,cst::NB_ROWS)-1)%CST(int,cst::NB_ROWS);
}



//---------------------------------------------------------------------------------------
uint indexAtTMDelta(uint val) noexcept
{
    return val%CST(int,cst::NB_ROWS);
}



}
