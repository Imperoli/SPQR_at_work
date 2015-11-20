#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>

#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
 #include <tf/transform_broadcaster.h>

#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_F 0x66
#define KEYCODE_G 0x67
#define KEYCODE_H 0x68
#define KEYCODE_T 0x74
#define KEYCODE_R 0x72
#define KEYCODE_Y 0x79
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65



class ManualCalibration
{
    private:
        double walk_vel_;
        double run_vel_;
        double yaw_rate_;
        double yaw_rate_run_;
        double tx, ty, tz;
        double yaw, pitch, roll;
        
        geometry_msgs::Twist cmdvel_;
        ros::NodeHandle n_;
        ros::Publisher pub_;

    public:
        ManualCalibration()
        {
            
            ros::NodeHandle n_private("~");
            n_private.param("walk_vel", walk_vel_, 0.5);
            n_private.param("run_vel", run_vel_, 1.0);
            n_private.param("yaw_rate", yaw_rate_, 1.0);
            n_private.param("yaw_rate_run", yaw_rate_run_, 1.5);
            
               
            tx=-0.3483; ty=-0.01316162; tz=0.643236;
            tf::Quaternion quat(0.228405, 0.326475, -0.502927, 0.767014);
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);   
        }
        
        ~ManualCalibration() { }
        void keyboardLoop();
        
        void stop()
        {
        }
};

ManualCalibration* tbk;
int kfd = 0;
struct termios cooked, raw;
bool done;

int main(int argc, char** argv)
{
    ros::init(argc,argv,"manual_calib", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    ManualCalibration tbk;
    
    boost::thread t = boost::thread(boost::bind(&ManualCalibration::keyboardLoop,&tbk));
    
    ros::spin();
    
    t.interrupt();
    t.join();
    tbk.stop();
    tcsetattr(kfd, TCSANOW, &cooked);
    
    return(0);
}

void ManualCalibration::keyboardLoop()
{
    char c;
    double max_tv = walk_vel_;
    double max_rv = yaw_rate_;
    bool dirty = false;

    
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
   
    
    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;
    
    for(;;)
    {
        boost::this_thread::interruption_point();
        
        // get the next event from the keyboard
        int num;
        
        if ((num = poll(&ufd, 1, 250)) < 0)
        {
            perror("poll():");
            return;
        }
        else if(num > 0)
        {
            if(read(kfd, &c, 1) < 0)
            {
                perror("read():");
                return;
            }
        }
        else
        {
            if (dirty == true)
            {
                stop();
                dirty = false;
            }
            
            continue;
        }
        double scale=.5;
        
        switch(c)
        {
            case KEYCODE_W:
                roll+=scale*0.01;
	              
                dirty = true;
                break;
            case KEYCODE_S:
                roll-=scale*0.01;
                dirty = true;
                break;
            case KEYCODE_A:
                pitch-=scale*0.01;
                dirty = true;
                break;
            case KEYCODE_D:
                pitch+=scale*0.01;
                dirty = true;
                break;
                
            case KEYCODE_Q:
                yaw-=scale*0.01;
                dirty = true;
                break;
            case KEYCODE_E:
                yaw+=scale*0.01;
                dirty = true;
                break;
                
            case KEYCODE_R:
                tz-=scale*0.005;
                dirty = true;
                break;
	              
             case KEYCODE_Y:
                tz+=scale*0.005;
                dirty = true;
                break;
             case KEYCODE_F:
                tx-=scale*0.005;
                dirty = true;
                break;
                
              case KEYCODE_H:
                tx+=scale*0.005;
                dirty = true;
                break;
                
              case KEYCODE_T:
                ty+=scale*0.005;
                dirty = true;
                break;
                
              case KEYCODE_G:
                ty-=scale*0.005;
                dirty = true;
                break;
            default:
                dirty = true;
        }
        
      static tf::TransformBroadcaster br;
      tf::Transform transform;
      transform.setOrigin( tf::Vector3(tx, ty, tz) );
      tf::Quaternion q;
      q.setRPY(roll,pitch,yaw);
      transform.setRotation(q);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "arm_base", "kinect_link"));
     // std::cout<<"translation: "<<tx<<" "<<ty<<" "<<tz<<std::endl;
     // std::cout<<"orientation: "<<q.x<<" "<<q.y<<" "<<q.z<<" "<<q.w<<std::endl;
      
      ros::Rate lr(30);
      lr.sleep();
    }
}

