#include <iostream>
#include "semantic_map.h"

using namespace std;

int main(int argc, char **argv)
{
//    auto c = make_shared<Circle>(1, 2, 3);
//    // Rectangle r(1, 2, 3, 4);
//    auto r = make_shared<Rectangle>(1, 2, 3, 4);
//    cout << c->NR_str() << endl;
//    cout << r->NR_str() << endl;
//    cout << (c + r)->NR_str() << endl;
//    cout << c->NR_str() << endl;
//    cout << r->NR_str() << endl;
//    cout << (c + r)->NR_str() << endl;
//    AreaParser::toAreaType(c->NR_str()+"_CS")->NR_print();
//    AreaParser::toAreaType(r->NR_str()+"_CS")->NR_print();
//    AreaParser::toAreaType(c->NR_str()+"_both")->NR_print();
//    AreaParser::toAreaType(r->NR_str()+"_both")->NR_print();
//    AreaParser::toAreaType("MixArea(Circle(1.00, 2.00, 3.00) + Circle(27.00, 13.00, 3.00) + MixArea(Circle(1.00, 2.00, 3.00) + Circle(27.00, 13.00, 3.00) ))")->NR_print();
//    AreaParser::toAreaType("Circle(1.00, 2.00, 3.00)")->NR_print();
//    AreaParser::toAreaType("(B3:=Circle(1.00, 2.00, 3.00))")->NR_print(); // 创建并注册区域（带括号）
//    AreaParser::toAreaType("B4:=Circle(1.00, 2.00, 3.00)")->NR_print(); // 创建并注册区域（不带括号）
//    AreaParser::toAreaType("A4:=(B5:=Circle(1.00, 2.00, 3.00))")->NR_print(); // 多个名称对应同一个区域（区域内部注册名称变量为最新注册名称A4）
//    AreaParser::toAreaType("A6:=MixArea(Rectangle(1, 1, 1, 1)+(B5:=Circle(1.00, 2.00, 3.00)))")->NR_print();
//    AreaParser::toAreaType("A7:=Circle(1.00, 2.00, 3.00)")->NR_print();
//    AreaParser::toAreaType("A8:=Rectangle(1, 1, 1, 1)")->NR_print();
//    AreaParser::toAreaType("A9:=MixArea(A7+A8)")->NR_print();
//    AreaManager::printAll();
//    AreaManager::getElementByName("B3")->NR_print();
//    AreaParser::toAreaType("B3_CS")->NR_print(); // 只临时创建中心对称对象，不注册
//    AreaParser::toAreaType("B3_CS:=B3_CS")->NR_print(); // 创建并注册中心对称对象
//    AreaParser::toAreaType("B3_CS:=Circle(1,3,5)")->NR_print(); // 已注册的区域不会被:=修改
//    AreaParser::toAreaType("B3_CS:=Circle(2, 4, 6)")->NR_print();
//    AreaManager::printAll();
//    auto A9 = AreaManager::getElementByName("A6");
//    auto A8 = AreaManager::getElementByName("B5");
//    cout << (A9->NR_str()) << endl;
//    AreaParser::toAreaType("B5:=(B6=(B5:=Circle(3,2,1)))")->NR_print();
//    AreaParser::toAreaType("B6=Circle(3,1,1)")->NR_print();
//    AreaParser::toAreaType("梯高:=环高=MixArea((梯高:=Circle(3,2,1))+circle(1,2,4))")->NR_print(); // 中文
//    AreaParser::toAreaType("梯高:=环高=梯高:=Circle(3,2,1)")->NR_print(); // 中文
//    AreaParser::toAreaType("环高=Circle(3,1,1)")->NR_print(); // 中文
//    AreaParser::toAreaType("A:=B=MixArea((A:=Circle(3,2,1))+circle(1,2,4))")->NR_print(); //
//    AreaManager::getElementByName("A")->NR_print();
//    AreaParser::toAreaType("A:=B=A:=Circle(3,2,1)")->NR_print(); //
//    AreaParser::toAreaType("B=Circle(3,1,1)")->NR_print(); //
//    AreaManager::printAll();
//    AreaParser::toAreaType("B5=Circle(1,2,4)")->NR_print();
//    AreaParser::toAreaType("B6")->NR_print();
//    A8->NR_print();
//
//    double a = 1;
//    double b = 2;
//    ref_Point p1(a, b, "测试点");
//    a = 3;
//    b = 4;
//    p1.NR_print();
//    area_parser["A1:=Circle(1,2,3)"]->NR_print();
//    area_parser["A1"]->NR_print();
//    area_parser["MixArea(m(c(1,2,3)))"]->NR_print();
//    AreaManager::init();
//    total_map_ptr->NR_print();
//
//    cout << const_Point(0.5, 0.5).is_in(area_parser["p(0,0; 0,1; 1,1; 0,1)"]) << endl;
//    cout << setprecision(8) << const_Point(1, 0).is_in(area_parser["c(0,0,1)"]) << endl;
//
//    // 测试ref_Point的引用能力
//    vector<double> test_vec{1, 2, 3};
//    auto p3 = make_shared<ref_Point>(test_vec[2], test_vec[0], "测试点");
////    p1.NR_print();
////    test_vec.at(0) = 4;
////    p1.NR_print();
//    auto p4 = make_shared<ref_Point>(test_vec[1], test_vec[0], "测试点");
//    auto p5 = make_shared<ref_Point>(test_vec[0], test_vec[2], "测试点");
//    auto Poly = Polygon(vector<PointPtr>{p3, p4, p5});
////    p.NR_print();
//    test_vec.at(0) = 5;
//    test_vec.at(1) = 6;
//    test_vec.at(2) = 7;
//    Poly.S_print();
//    Poly.SR_print();
//    Poly.N_print();
//    Poly.NR_print();
//    Poly.D_print();
//    Poly.DR_print();
//    AreaManager::init();
//    AreaManager::printAll();
    
//    auto a = make_shared<const_Point>();
//    auto b = make_shared<const_Point>();
//    auto c = make_shared<const_Point>();
//    auto d = make_shared<Circle>(1, 2, 3);
//    AreaManager::registerElement("1", d);
//    cout << PointManager::isRegistered("1") << endl;
//    PointManager::registerElement("1", a);
//    PointManager::registerElement("2", b);
//    cout << PointManager::isRegistered("1") << endl;
//    PointManager::registerElement("3", c);
//    PointManager::printAll();
    // 混合区域判断
//    auto P = AreaParser::toAreaType("多边形:=P(0,0; 0,1; 1,1; 1,0)");
//    auto mix = AreaParser::toAreaType("混合:=m(多边形-c(0,0,1))");
//    for (int i = 1; i < 100; ++i) {
//        for (int j = 1; j < 100; ++j) {
//            auto p = make_shared<const_Point>(double(i)/100, double(j)/100);
//            cout << p->N_str() << (p->is_in(mix)? "在": "不在") << mix->S_str() << "区域内, dis_2=" << (p->get_x()*p->get_x()) + (p->get_y()*p->get_y()) << endl;
//            assert(p->is_in(P));
//        }
//    }
//    auto point = make_shared<const_Point>(0.99, 0.99);
//    cout << point->is_in(P) << endl;
//    area_parser["A1:=Circle(1,2,3)"]->NR_print();
//    area_parser["A1^cs"]->NR_print();
//    cout << const_Point(0.99999999999999999, 0.5).is_in(area_parser["poly(-1,-1;-1,1;1,1;1,-1)"]) << endl; // 顶点及边点是否在多边形内
    auto [from_0, to_0] = rad2deg(area_parser["Rectangle(0,0,2,2)"]->CCW_sector_from(const_Point(-1, -1)));
    cout << "from: " << from_0 << " to: " << to_0 << endl;
    auto [from_1, to_1] = rad2deg(area_parser["poly(-1,-1;-1,1;1,1;1,-1)"]->CCW_sector_from(const_Point(-1, -1)));
    cout << "from: " << from_1 << " to: " << to_1 << endl;
    auto [from_2, to_2] = rad2deg(area_parser["Circle(0,0,1)"]->CCW_sector_from(const_Point(0, 1)));
    cout << "from: " << from_2 << " to: " << to_2 << endl;
    auto [from_3, to_3] = rad2deg(area_parser["poly(13.2,12.67;16.1,9.97;18.47,13.8;14.78,13.82)"]->CCW_sector_from(const_Point(-1, -1)));
    cout << "from: " << from_3 << " to: " << to_3 << endl;
}