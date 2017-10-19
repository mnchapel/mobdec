#ifndef CTRACK_H
#define CTRACK_H



#include <vector>



/// @class CTrack
class CTrack {
public:

	/// original starting point of the track
	int mox = 0;

	/// original starting point of the track
	int moy = 0;

	/// current position of the track
	std::vector<float> mx;

	///  current position of the track
	std::vector<float> my;

	/// assignment to a region (ignored for tracking but makes the tracking files compatible to other tools I have)
	int mLabel = -1;

	/// tracking stopped due to occlusion, etc.
	bool mStopped = false;

	/// Time when track was created
	int mSetupTime = 0;

	///
	double mProba = 0.;



	/// @brief Default constructor
	CTrack() = default;
};



#endif  /* CTRACK_H */
