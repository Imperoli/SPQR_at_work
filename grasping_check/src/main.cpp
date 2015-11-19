#include "grasping_check.h"

int main(int argc, char** argv)
{
  cv::Mat background_image = cv::imread(argv[1]);
  cv::Mat object_image = cv::imread(argv[2]);
  cv::Mat test_image = cv::imread(argv[3]);

  GraspingCheck grasping_check( background_image, object_image, 30, 1 );

  grasping_check.enableVisualDebug(true);

  std::cout<<"Gripper score : "<<grasping_check.getScore( test_image  )<<std::endl;

  std::cout<<"Type \"ESC\" to exit\n";
  while(cv::waitKey() != 27);
  
  return 0;
}
