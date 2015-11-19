#include <iostream>
#include <stdexcept>

#include "grasping_check.h"

GraspingCheck::GraspingCheck ( const cv::Mat &background_image, const cv::Mat &object_image,
                               int mask_threshold, int pyr_levels ) :
  debug_enabled_(false)
{
  resetRefImages ( background_image, object_image, mask_threshold, pyr_levels );
}

void GraspingCheck::resetRefImages ( const cv::Mat &background_image, const cv::Mat &object_image,
                                     int mask_threshold, int pyr_levels )
{
  pyr_levels_ = pyr_levels;
  cv::Mat norm_bkg_img, norm_obj_img;

  normalizeImage ( background_image, norm_bkg_img );
  normalizeImage ( object_image, norm_obj_img );

  cv::Mat abs_diff_img;
  cv::absdiff( norm_obj_img, norm_bkg_img, abs_diff_img );
  mask_ = abs_diff_img > mask_threshold;
  cv::Mat morph_elem3 = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                                  cv::Size( 3, 3 ), cv::Point( 1, 1 ) );
  cv::Mat morph_elem5 = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                                  cv::Size( 5, 5 ), cv::Point( 2, 2 ) );
  cv::erode(mask_, mask_, morph_elem5 );
  cv::dilate(mask_, mask_, morph_elem3 );

  computeGradientMagnitude( norm_obj_img, ref_img_);

  if( debug_enabled_ )
  {
    cv::imshow("Mask", mask_);
    cv::imshow("ref_img_", ref_img_);
  }
}

double GraspingCheck::getScore( const cv::Mat& test_image )
{
  cv::Mat norm_test_image, test_image_gm;

  normalizeImage ( test_image, norm_test_image );
  computeGradientMagnitude(norm_test_image, test_image_gm);

  return getNormalizedCorrelation( test_image_gm, ref_img_, mask_);
}


// double GraspingCheck::mutualInformation( const cv::Mat& test_image )
// {

//   cv::imshow("test_img", test_image_gm);
//
//   const int num_bins = 32;
//   const cv::Mat images[] = {ref_img_, test_image_gm };
//   const int channels[] = {0, 1};
//   const int hist_size[] = { num_bins, num_bins };
//   const float range[] = { 0, 256 };
//   const float* ranges[] = { range, range };
//   cv::Mat hist2d = cv::Mat( cv::Size( num_bins, num_bins), cv::DataType<float>::type );
//
//   cv::calcHist(images, 2, channels, /*cv::Mat()*/mask_, hist2d, 2, hist_size, ranges, true, false );
//
//   // Normalize the histogram
//   cv::Scalar hist_sum = cv::sum( hist2d );
//   float hist_normalizer = 1.0/hist_sum[0];
//   hist2d *= hist_normalizer;
//
//   cv::Mat dbg_hist2d;
//   cv::normalize ( hist2d,   dbg_hist2d, 0, 255, cv::NORM_MINMAX );
//   cv::imshow("hist2D", dbg_hist2d );
//
//   double h_x = 0.0, h_y = 0.0, h_xy = 0.0;
//
//   for( int i = 0; i < num_bins; i++ )
//   {
//     cv::Scalar sum_x = cv::sum( hist2d.row(i) ), sum_y = cv::sum( hist2d.col(i) );
//     double p_x = sum_x[0], p_y = sum_y[0];
//
//     h_x += - p_x * log2( p_x + (p_x == 0) );
//     h_y += - p_y * log2( p_y + (p_y == 0) );
//   }
//
//   for( int r = 0; r < num_bins; r++ )
//   {
//     float *hist_ptr = hist2d.ptr<float>(r);
//     for( int c = 0; c < num_bins; c++, hist_ptr++ )
//     {
//       float &p_xy = (*hist_ptr);
//       h_xy += - p_xy * log2( p_xy + (p_xy == 0) );
//     }
//   }
//
//   return h_x + h_y - h_xy;
// }

void GraspingCheck::normalizeImage ( const cv::Mat &src, cv::Mat &dst )
{
  cv::Mat gl_img;
  if( src.channels() > 1 )
  {
    cv::cvtColor(src, gl_img, cv::COLOR_BGR2GRAY );
    cv::GaussianBlur(gl_img, gl_img, cv::Size(5,5), 0);
  }
  else
    cv::GaussianBlur(src, gl_img, cv::Size(5,5), 0);

  for( int i = 0; i < pyr_levels_; i++ )
    cv::pyrDown(gl_img, gl_img);

  dst = gl_img;
}

void GraspingCheck::computeGradientMagnitude ( const cv::Mat& src, cv::Mat& dst )
{
  cv::Mat grad_x, grad_y, grad_mag;

  cv::Scharr ( src, grad_x, cv::DataType<float>::type, 1, 0, 1./32. );
  cv::Scharr ( src, grad_y, cv::DataType<float>::type, 0, 1, 1./32. );

  grad_mag = cv::abs ( grad_x ) + cv::abs ( grad_y );
  cv::normalize ( grad_mag, grad_mag, 0, 255, cv::NORM_MINMAX );
  grad_mag.convertTo(dst, cv::DataType<uchar>::type);
}

double GraspingCheck::getNormalizedCorrelation( const cv::Mat& img1, const cv::Mat& img2, const cv::Mat& mask )
{
  cv::Mat masked_img1 = img1&mask, masked_img2 = img2&mask;
  cv::Mat masked_img1_f, masked_img2_f;

  masked_img1.convertTo(masked_img1_f, cv::DataType<float>::type, 1.0/255.0 );
  masked_img2.convertTo(masked_img2_f, cv::DataType<float>::type, 1.0/255.0 );

  cv::Mat corr_mat = masked_img1_f.mul(masked_img2_f);
  masked_img1_f = masked_img1_f.mul(masked_img1_f);
  masked_img2_f = masked_img2_f.mul(masked_img2_f);
  cv::Scalar corr_sum = cv::sum( corr_mat ),
             img1_squred_sum = cv::sum( masked_img1_f ),
             img2_squred_sum = cv::sum( masked_img2_f );

  double norm_corr = corr_sum[0]/sqrt(img1_squred_sum[0]*img2_squred_sum[0]);

  if( debug_enabled_ )
  {
    cv::imshow("masked_img1", masked_img1);
    cv::imshow("masked_img2", masked_img2);
    std::cout<<"norm_corr : "<<norm_corr<<std::endl;
  }

  return norm_corr;
}

double GraspingCheck::getNormalizedSquaredDiff( const cv::Mat& img1, const cv::Mat& img2, const cv::Mat& mask )
{
  cv::Mat masked_img1 = img1&mask, masked_img2 = img2&mask;
  cv::Mat masked_img1_f, masked_img2_f;

  masked_img1.convertTo(masked_img1_f, cv::DataType<float>::type, 1.0/255.0 );
  masked_img2.convertTo(masked_img2_f, cv::DataType<float>::type, 1.0/255.0 );

  cv::Mat diff_mat = masked_img1_f - masked_img2_f;
  diff_mat.mul(diff_mat);
  masked_img1_f = masked_img1_f.mul(masked_img1_f);
  masked_img2_f = masked_img2_f.mul(masked_img2_f);
  cv::Scalar diff_sum = cv::sum( diff_mat ),
             img1_squred_sum = cv::sum( masked_img1_f ),
             img2_squred_sum = cv::sum( masked_img2_f );


  double norm_sqared_diff = diff_sum[0]/sqrt( img1_squred_sum[0]*img2_squred_sum[0] );

  if( debug_enabled_ )
  {
    cv::imshow("masked_img1", masked_img1);
    cv::imshow("masked_img2", masked_img2);
    std::cout<<"norm_sqared_diff : "<<norm_sqared_diff<<std::endl;
  }

  return norm_sqared_diff;
}