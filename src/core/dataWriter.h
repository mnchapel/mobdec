/// @file   dataWriter.h
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#pragma once



// C++
#include <fstream>
#include <vector>

// MoBDec
#include <core/data.h>



/// @brief
class DataWriter{

//-----------------------------------------------------------------------------
// PUBLIC MEMBER FUNCTION
public:

	/// @brief Destructor
	~DataWriter() = default;



    /// @brief
    ///
    /// @param data
    /// @param component_name
    /// @param cluster_feature_point_id_list
    static void paintCluster(const Data* data,
					  	  	 std::string component_name,
							 const std::vector<std::vector<uint>>& cluster_feature_point_id_list) noexcept;



    /// @brief paintConfidenceValue
    ///
    /// @param data
    /// @param component_name
    static void paintConfidenceValue(const Data* data,
                         	  	  	 std::string component_name) noexcept;



    /// @brief
    ///
    /// @param data
    /// @param component_name
    static void paintFeaturePoint(const Data* data,
    							  std::string component_name) noexcept;




	/// @brief
	///
	/// @param data
	/// @param component_name
	/// @param points
	/// @param labels
	static void paintKMeansCluster(const Data* data,
								   std::string component_name,
								   uint nb_cluster,
								   const std::vector<cv::Point2f>& points,
								   const cv::Mat& labels) noexcept;



    /// @brief
    ///
    /// @param data
    /// @param component_name
    /// @param centers
    static void paintKMeansClusterCenters(const Data* data,
    							  	  	  std::string component_name,
										  const std::vector<cv::Point2f>& centers) noexcept;



    /// @brief paintOpticalFlowOnCurrentImage
    ///
    /// @param data:
    /// @param component_name:
    static void paintOpticalFlow(const Data* data,
                          	  	 std::string component_name);



    /// @brief
    ///
    /// @param data
    /// @param component_name
    static void paintSlicSuperPixelContour(const Data* data,
         								   std::string component_name) noexcept;



    /// @brief
    ///
    /// @param data
    /// @param component_name
    static void paintSlicSuperPixelContourWithConfidenceValue(const Data* data,
    												   	   	  std::string component_name) noexcept;



    /// @brief
    ///
    /// @param data
    /// @param component_name
    static void paintSlicSuperPixelRegionWithConfidenceValue(const Data* data,
    												   	   	 std::string component_name) noexcept;



    /// @brief
    ///
    /// @param data
    /// @param component_name
    static void paintSlicSuperPixelRegionWithConfidenceValueBis(const Data* data,
    												   	   	 	std::string component_name) noexcept;



    /// @brief
    ///
    /// @param data
    /// @param component_name
    /// @param cluster_feature_point_id_list
    /// @param cluster_bonus_malus_value
    static void paintTrendBonusMalus(const Data* data,
                               	  	 std::string component_name,
									 const std::vector<std::vector<uint>>& cluster_feature_point_id_list,
									 const std::vector<double>& bonus_malus_value) noexcept;



    /// @brief
    ///
    /// @param data
    /// @param component_name
    /// @param cluster_feature_point_id_list
    /// @param cluster_bonus_malus_value
    static void paintTrendClusterBonusMalus(const Data* data,
                               	  	 	 	std::string component_name,
											const std::vector<std::vector<uint>>& cluster_feature_point_id_list,
											const std::vector<double>& cluster_bonus_malus_value) noexcept;



    /// @brief
    ///
    /// @param data
    /// @param component_name
    static void writeBlenderFile(const Data* data,
    					  	  	 std::string component_name) noexcept;



    /// @brief
    ///
    /// @param data:
    /// @param component_name:
    /// @param cluster_feature_point_id_list:
    static void writeCluster(const Data* data,
                      	  	 std::string component_name,
							 const std::vector<std::vector<uint>>& cluster_feature_point_id_list) noexcept;



    /// @brief
    ///
    /// @param data:
    /// @param component_name:
    static void writeConfidenceValue(const Data* data,
                              	  	 std::string component_name) noexcept;



    /// @brief writeDistanceToCamera
    ///
    /// @param data
    /// @param component_name
    static void writeDistanceToCamera(const Data* data,
                                      std::string component_name) noexcept;



    /// @brief writeExtrinsicParameters
    ///
    /// @param data
    /// @param component_name
    /// @param rotation
    /// @param translation
    static void writeExtrinsicParameters(const Data *data,
										 std::string component_name,
										 const cv::Mat& rotation,
                                         const cv::Mat& translation) noexcept;



    /// @brief writeFeaturePointPosition
    ///
    /// @param data
    /// @param component_name
    static void writeFeaturePointPosition(const Data* data,
    							  	  	  std::string component_name) noexcept;



    /// @brief writeFeaturePointPositionAge
    ///
    /// @param data
    /// @param component_name
    static void writeFeaturePointPositionAge(const Data* data,
    										 std::string component_name) noexcept;



    /// @brief writeLabel
    ///
    /// @param data
    /// @param component_name
    static void writeLabel(const Data* data,
                    	   std::string component_name) noexcept;



    /// @brief writeMedianScale
    ///
    /// @param data
    /// @param component_name
    /// @param median_scale
    static void writeMedianScale(const Data* data,
                          	  	 std::string component_name,
								 double median_scale) noexcept;



    /// @brief writeState
    ///
    /// @param data
    /// @param component_name
    /// @param state
    static void writeState(const Data* data,
                           std::string component_name,
						   std::string state) noexcept;



    /// @brief
    ///
    /// @param data
    /// @param component_name:
    static void writeStaticFeaturePointPositionAtT(const Data* data,
                          	   	   	        	   std::string component_name) noexcept;



    /// @brief
    ///
    /// @param data:
    /// @param component_name:
    static void writeStaticFeaturePointPositionAtTMDelta(const Data* data,
                           	   	   	        	  	  	 std::string component_name) noexcept;



    /// @brief
    ///
    /// @param data:
    /// @param component_name
    /// @param cluster_feature_point_id:
    /// @param bonus_malus_value:
    static void writeTrendBonusMalus(const Data* data,
                       	  	  std::string component_name,
							  const std::vector<std::vector<uint>>& cluster_feature_point_id_list,
							  const std::vector<double>& bonus_malus_value) noexcept;



    /// @brief
    ///
    /// @param data:
    /// @param component_name:
    /// @param cluster_feature_point_id_list:
    /// @param cluster_bonus_malus_value:
    static void writeTrendClusterBonusMalus(const Data* data,
                           	  	  	  	 	std::string component_name,
											const std::vector<std::vector<uint>>& cluster_feature_point_id_list,
											const std::vector<double>& cluster_bonus_malus_value) noexcept;



//-----------------------------------------------------------------------------
// PRIVATE MEMBER FUNCTION
private:

    /// @brief Default constructor
    DataWriter() = default;



	/// @brief Copy constructor
    DataWriter(const DataWriter&) = default;



	/// @brief Move constructor
    DataWriter(DataWriter&&) = default;



	/// @brief Copy assignment operator
    DataWriter& operator=(const DataWriter&) = default;



	/// @brief Move assignment operator
    DataWriter& operator=(DataWriter&&) = default;



    /// @brief buildColorCode
    ///
    /// @param age
    ///
    /// @return color
    static cv::Scalar ageColor(uint age) noexcept;



    /// @brief randomColor
    ///
    /// @param rng
    ///
    /// @return color
    static cv::Scalar randomColor(cv::RNG& rng) noexcept;

};
