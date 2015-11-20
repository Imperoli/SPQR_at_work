/*
 * Main node for hri_pnp
 */

#include "RockinPNPAS.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rockin_pnp");

    ros::NodeHandle node;
    
    RockinPNPActionServer rockin_pnp(node);
    rockin_pnp.start();
    ros::spin();

    return 0;
}

