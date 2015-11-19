#pragma once

#include <opencv2/opencv.hpp>

class GraspingCheck
{
public:

 GraspingCheck( const cv::Mat &background_image, const cv::Mat &object_image,
                int mask_threshold = 30, int pyr_levels = 1 );

 void enableVisualDebug( bool enabled ){ debug_enabled_ = enabled; };
 void resetRefImages( const cv::Mat &background_image, const cv::Mat &object_image,
                      int mask_threshold, int pyr_levels );
 double getScore( const cv::Mat &test_image );


private:

//  double mutualInformation( const cv::Mat &test_image );

 void normalizeImage( const cv::Mat &src, cv::Mat &dst );
 void computeGradientMagnitude( const cv::Mat &src, cv::Mat &dst );
 double getNormalizedCorrelation( const cv::Mat& img1, const cv::Mat& img2, const cv::Mat& mask );
 double getNormalizedSquaredDiff( const cv::Mat& img1, const cv::Mat& img2, const cv::Mat& mask );

 bool debug_enabled_;
 int pyr_levels_;

 cv::Mat mask_;
 cv::Mat ref_img_;
};