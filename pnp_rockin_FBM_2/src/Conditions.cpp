#include <boost/algorithm/string.hpp>

#include "RockinPNPAS.h"
#include "topics.h"



using namespace std;


int RockinPNPActionServer::evalCondition(string cond) {

    // when this function returns -1, the condition is evaluated from
    // the events published to PNPConditionEvent
    return -1;
}



