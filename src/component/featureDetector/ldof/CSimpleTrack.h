/// @file CSimpleTrack.h
/// @brief
/// @author From ECCV

#ifndef CSIMPLE_TRACK_H
#define CSIMPLE_TRACK_H


/// @class CPoint
class CSimpleTrack {
public:
	///
	int mLabel = 0;

	///
	CVector<CPoint> mPoints;



	/// @brief default constructor
	CSimpleTrack() = default;
};



#endif /* CSIMPLE_TRACK_H */
