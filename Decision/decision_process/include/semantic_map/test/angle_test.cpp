#include <cmath>
#include <iostream>
#include "semantic_map.h"

using namespace std;


// 弧度转360°角度制
double rad_to_360deg(double rad)
{
    while (rad > 2 * M_PI)
        rad -= 2 * M_PI;
    while (rad < 0)
        rad += 2 * M_PI;
    return rad / M_PI * 180;
}

double angle_from_to(double rad_from, double rad_to)
{
    // 映射扇形范围到单位圆上
    auto from = make_pair(cos(rad_from), sin(rad_from));
    auto to = make_pair(cos(rad_to), sin(rad_to));
    auto cos_angle= acos(from.first * to.first + from.second * to.second); // 向量点乘
    auto sin_angle= asin(from.first * to.second - from.second * to.first); // 向量叉乘
    // 利用角的cos和sin值求出其角度制0-360°范围的角度
    if (cos_angle >= M_PI / 2)
        return rad_to_360deg(sin_angle);
    else
        return rad_to_360deg(M_PI - sin_angle);
}

int main()
{
//    double rad_from = -25;
//    double rad_to = 25;
//    for (int i = 0; i < 361; ++i) {
//        double cur_yaw;
//        if (i <= 180)
//            cur_yaw = double(i);
//        else
//            cur_yaw = -double(360 - i);
//        double angle_from = angle_from_to(cur_yaw/180*M_PI, rad_from/180*M_PI);
//        double angle_to = angle_from_to(cur_yaw/180*M_PI, rad_to/180*M_PI);
//        cout << cur_yaw << " " << angle_from << " " << angle_to << " " << angle_to-angle_from << endl;
//    }
//    cout << angle_from_to(90.0/180*M_PI, -30.0/180*M_PI) << endl;
//    cout << angle_from_to(90.0/180*M_PI, 30.0/180*M_PI) << endl;
    
    auto[a, b] = AreaParser::toAreaType("Polygon(13.2,12.67;16.1,9.97;18.47,13.8;14.78,13.82)")->CCW_sector_from(const_Point(0,0));
    cout << a << " " << b << endl;
}