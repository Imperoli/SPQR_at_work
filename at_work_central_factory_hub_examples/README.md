RoCKIn@Work Central Factory Hub (CFH) Examples
==============================================

This repository contains very simple examples of how a robot would communicate with the RoCKIn Central Factory Hub.

building with ROS catkin_make
-----------------------------

This project works out of the box

1. Clone it into catkin_workspace/src
2. Get the git submodules. See README
3. run catkin_make

Note: This package contains at_work_central_factory_hub_comm which will conflict if it's already in the catkin workspace.
A quick fix is to add CATKIN_IGNORE files, in one of the conflicting locations. 

    cd [catkin_workspace]
    touch ./src/at_work_central_factory_hub_comm/at_work_central_factory_hub_comm/CATKIN_IGNORE
    touch ./src/at_work_central_factory_hub_comm/protobuf_comm/CATKIN_IGNORE
    touch ./src/at_work_central_factory_hub_comm/rockin_msgs/CATKIN_IGNORE

about git submodules
--------------------

!!Important!! This project uses git submodules, and those submodules contain git submodules. 
Using git 1.6.5 or later this is easy to manage with two commands:

    git clone [URL for at_work_central_factory_hub_examples]
    git submodule update --init --recursive

or with more recent versions of git:

    git clone --recursive-submodule [URL for at_work_central_factory_hub_examples]

Older versions of git need to

    git clone [URL for at_work_central_factory_hub_examples]
    cd at_work_central_factory_hub_examples
    git submodule init
    git submodule update
    cd at_work_central_factory_hub_comm
    git submodule init
    git submodule update
