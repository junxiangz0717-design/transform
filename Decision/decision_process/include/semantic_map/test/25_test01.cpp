#include <iostream>
#include <vector>
#include "semantic_map.h"

using namespace std;

int main(int argc, char **argv)
{

    
    area_manager.init();

    vector<PointPtr> targets_point;
    auto target_point_1 = make_shared<ref_Point>(13.0, 3.1);
    auto target_point_2 = make_shared<ref_Point>(15.7,12.6);

    bool a = target_point_1->is_in(area_manager["堡垒"]);
    bool b = target_point_2->is_in(area_manager["堡垒"]);
    cout << a << endl;
    cout << b << endl;



    return 0;
}