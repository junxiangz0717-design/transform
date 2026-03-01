//模拟半双工通信后的超时器更新

#pragma once

#include "data/decision_data.h"
#include "data/target.h"

void updata_timer()
{
    DD.autoaim_connect_timer.reset(); 
    DD.detector_connect_timer.reset();
        DD.target_timer.reset();
    if (DD.hp_sentry != 0)
        {
            DD.hp_0_timer.reset();
        }

}


