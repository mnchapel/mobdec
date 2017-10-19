/// @file   falseNegativeSuppression.h
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#pragma once



// C++
#include <ctime>
#include <fstream>
#include <functional>
#include <iostream>
#include <mutex>
#include <stdlib.h>
#include <thread>
#include <vector>

// MoBDec
#include <core/dataWriter.h>
#include <core/phase.h>



/// @brief The FalseNegativeSuppression class
class FalseNegativeSuppression : public Phase{

//-----------------------------------------------------------------------------
// PUBLIC MEMBER FUNCTION
public:

    /// @brief Default constructor
	FalseNegativeSuppression() noexcept = default;



	/// @brief Copy constructor
    FalseNegativeSuppression(const FalseNegativeSuppression&) noexcept = delete;



	/// @brief Move constructor
	FalseNegativeSuppression(FalseNegativeSuppression&&) noexcept = default;



	/// @brief Copy assignment operator
    FalseNegativeSuppression& operator=(const FalseNegativeSuppression&) noexcept = delete;



	/// @brief Move assignment operator
	FalseNegativeSuppression& operator=(FalseNegativeSuppression&&) noexcept = default;



    /// @brief Destructor.
    ~FalseNegativeSuppression() noexcept = default;



    /// @brief Get the component name.
    std::string getComponentName() const noexcept override;



//-----------------------------------------------------------------------------
// PROTECTED MEMBER FUNCTION
protected:

    /// @brief compute
    void compute() noexcept override;



    /// @brief computeBis
    void computeBis() noexcept;



    /// @brief computeFalseNegativeSuppress
    ///
    /// @param cluster_label_value
    /// @param cluster_label_list
    /// @param cluster_optical_flow_list
    /// @param false_negative_label_cluster
    void computeFalseNegativeSuppress(const std::vector<uint>& cluster_label_value,
                                      const std::vector< std::vector<uint> >& cluster_optical_flow_list,
                                      std::vector<uint>& false_negative_label_cluster) noexcept;



    /// @brief computeOpticalFlowCluster
    ///
    /// @param isInCluster
    /// @param cluster
    void computeOpticalFlowCluster(std::vector<bool>& isInCluster,
                                   std::vector<uint>& cluster) noexcept;



    /// @brief computeOpticalFlowClusterList
    ///
    /// @param cluster_list
    void computeOpticalFlowClusterList(std::vector< std::vector<uint> >& cluster_list) noexcept;
    
    
    
    /// @brief .
    ///
    /// @return true if enough time elapsed to run the phase, false else.
    bool isEnoughTimeElapsed() const noexcept override;



    /// @brief isFalseNegativeCluster
    ///
    /// @param cluster_optical_flow
    ///
    /// @return
    bool isFalseNegativeCluster(const std::vector<uint> &cluster_optical_flow) noexcept;



    /// @brief isOpticalFlowNearToCluster
    ///
    /// @param i
    /// @param j
    ///
    /// @return
    bool isOpticalFlowNearToCluster(uint i,
                                    uint j) noexcept;



    /// @brief isPointNearToCluster
    ///
    /// @param index_1
    /// @param index_2
    ///
    /// @return
    bool isPointNearToCluster(uint index_1,
    						  uint index_2) noexcept;



    /// @brief
    void readFileData() noexcept override;



    /// @brief resetConfidenceIndex
    ///
    /// @param cluster_optical_flow
    void resetConfidenceIndex(const std::vector<uint>& cluster_optical_flow) noexcept;

};
