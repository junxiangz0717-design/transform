//
// Created by zh on 24-7-25.
// Update  by tzz
// semantic_map.h

#pragma once

#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <algorithm>
#include <utility>
#include <vector>
#include <map>
#include <unordered_set>
#include <memory>
#include <cassert>
#include <cmath>

using namespace std;

/** 预声明管理类 **/
class AreaManager;
class PointManager;

/** 预声明点类型 **/
class Point;
class const_Point;
class ref_Point;

/** 定义点共享指针别名 **/
using PointPtr          = shared_ptr<Point>;
using const_PointPtr    = shared_ptr<const_Point>;
using ref_PointPtr      = shared_ptr<ref_Point>;

/** 预声明区域类型 **/
class Area;
class Circle;
class Polygon;
class Rectangle;
class MixArea;

/** 定义区域共享指针别名 **/
using AreaPtr       = shared_ptr<Area>;
using CirclePtr     = shared_ptr<Circle>;
using PolygonPtr    = shared_ptr<Polygon>;
using RectanglePtr  = shared_ptr<Rectangle>;
using MixAreaPtr    = shared_ptr<MixArea>;


/** 编译时期确定地图中心点坐标，用于作中心对称 **/
//   static constexpr const double map_center_x = 14.0;
//   static constexpr const double map_center_y = 7.5;
/** 运行时动态确定地图中心点坐标，用于作中心对称 **/
extern double map_center_x;
extern double map_center_y;
/** 运行时动态确定全地图区域，用于区域取反操作 **/
extern AreaPtr total_map_ptr;


enum class PrintType: int
{
    Simple_Without_Registered_Names,
    Simple_With_Registered_Names,
    Normal_Without_Registered_Names,
    Normal_With_Registered_Names,
    Detail_Without_Registered_Names,
    Detail_With_Registered_Names,
};


template<typename T>
class Manager
{
protected:
    static map<string, shared_ptr<T>> registered_map;
    
public:
    Manager() = default;
    virtual ~Manager() = default;
    
    /**
     * @brief 注册区域，使用名称和区域指针
     */
    static void registerElement(const string& name, const shared_ptr<T>& T_ptr)
    {
        // 存在则更新
        if (isRegistered(name))
        {
            // 旧指针删除关联注册名称
            auto ord_T_ptr = registered_map.at(name);
            auto it = ord_T_ptr->registered_names.find(name);
            if (it != ord_T_ptr->registered_names.end())
                ord_T_ptr->registered_names.erase(it);
            // 更新注册区域
            registered_map.at(name) = T_ptr;
        }
            // 不存在则创建注册区域
        else
            registered_map.insert({name, T_ptr});
        // 新指针添加关联注册名称
        T_ptr->registered_names.insert(name);
    }
    
    /**
     * @brief 用注册名称获取对应区域
     * @param area_name 注册时使用的名称
     * @return 返回区域的共享指针
     */
    static shared_ptr<T>& getElementByName(const string& name)
    {
        // 确保需要获取的区域存在
        assert(registered_map.find(name) != registered_map.end());
        return registered_map.at(name);
    }
    
    /**
     * @brief 对象实例通过区域名称获取区域指针
     * @param area_name 区域名称
     * @return 区域指针
     */
    inline shared_ptr<T>& operator[](const string& name){ return getElementByName(name); }
    
    /**
     * @brief 打印已注册的所有元素
     * @param print_type 打印类型，使用默认注册名称
     */
    static void printAll(PrintType print_type = PrintType::Normal_With_Registered_Names)
    {
        stringstream ss;
        for (const auto& [name, T_ptr] : registered_map)
            ss << name + ": " + T_ptr->to_str(print_type) << endl;
        cout << ss.str();
    }

    /**
     * @brief 判断区域名称是否已经注册
     * @param name 注册名称
     * @return
     */
    static inline bool isRegistered(const string& name){ return registered_map.find(name) != registered_map.end(); }
};
// 模版类静态成员实例化
template<typename T>
map<string, shared_ptr<T>> Manager<T>::registered_map = map<string, shared_ptr<T>>();


/** 管理注册与文本打印的基类 **/
class Base
{
protected:
    friend class Manager<Area>;
    friend class Manager<Point>;
    unordered_set<string> registered_names{}; // 注册的名称集合
public:
    /** @brief 获取已注册名称前缀字符串 **/
    string get_registered_names() const
    {
        stringstream ss;
        ss << "[";
        for (auto& name : registered_names)
            ss << name << "|";
        return ss.str().substr(0, ss.str().size() - 1) + "]$";
    }
    /** @brief 获取打印字符串 **/
    virtual string to_str(PrintType print_type) const = 0;
    inline string S_str()    const { return to_str(PrintType::Simple_Without_Registered_Names); }
    inline string SR_str()   const { return to_str(PrintType::Simple_With_Registered_Names);    }
    inline string N_str()    const { return to_str(PrintType::Normal_Without_Registered_Names); }
    inline string NR_str()   const { return to_str(PrintType::Normal_With_Registered_Names);    }
    inline string D_str()    const { return to_str(PrintType::Detail_Without_Registered_Names); }
    inline string DR_str()   const { return to_str(PrintType::Detail_With_Registered_Names);    }
    inline void   S_print()  const { cout << S_str()     << endl;}
    inline void   SR_print() const { cout << SR_str()    << endl;}
    inline void   N_print()  const { cout << N_str()     << endl;}
    inline void   NR_print() const { cout << NR_str()    << endl;}
    inline void   D_print()  const { cout << D_str()     << endl;}
    inline void   DR_print() const { cout << DR_str()    << endl;}
};


/** 点基类(非区域) **/
// 继承enable_shared_from_this<Point>, 使得Point对象可以安全地获取指向自身的共享指针
class Point: public Base, public std::enable_shared_from_this<Point>
{
protected:
    friend class PointManager;
public:
    Point() = default;
    Point(const Point& point) = default;
    ~Point() = default;
    
    /**
     * @brief 获取所在的注册区域名称
     * @return 返回的数组包含所有所在注册区域名称
     */
    vector<string> get_inside_registered_areas() const;
    
    /** @brief 获取x坐标 **/
    inline virtual double get_x() const = 0;
    /** @brief 获取y坐标 **/
    inline virtual double get_y() const = 0;
    /** @brief 获取x、y坐标对 **/
    inline pair<double, double> get_xy() const { return {get_x(), get_y()}; }
    /** @brief 获取两点之间的距离 **/
    inline double distance(const PointPtr& point_ptr) const { return sqrt(pow(get_x() - point_ptr->get_x(), 2) + pow(get_y() - point_ptr->get_y(), 2)); }
    inline double distance(const Point& point) const { return sqrt(pow(get_x() - point.get_x(), 2) + pow(get_y() - point.get_y(), 2)); }
    inline double distance(double x, double y) const { return sqrt(pow(get_x() - x, 2) + pow(get_y() - y, 2)); }
    
    /** @brief 是否x、y坐标都为.0 **/
    inline bool empty() const { return get_x() == .0 && get_y() == .0; }
    
    /** @brief 是否在区域内 **/
    bool is_in(const AreaPtr& area_ptr) const;
    
    /** @brief 是否在全地图内 **/
    inline bool is_in_total_map() const { return is_in(total_map_ptr); }
    
    /** @brief 获取对区域的逆时针扇形扫描角度 **/
    pair<double, double> CCW_sector_to(const AreaPtr& area_ptr);
    
    /** @brief 获取打印字符串 **/
    string to_str(PrintType print_type) const override = 0;
    
    /** @brief 以拷贝方式创建const_Point **/
    const_Point to_const_Point() const;
    /** @brief 以拷贝方式创建const_PointPtr **/
    inline const_PointPtr to_const_PointPtr() const{ return make_shared<const_Point>(get_x(), get_y()); };
    
    /** @brief 以拷贝方式创建当前点位的中心对称点 **/
    const_Point central_symmetry() const;
    const_Point central_symmetry(const const_Point& point) const;
    const_Point central_symmetry(const const_Point& point, double target_dis) const;
    const_Point central_symmetry(const PointPtr& point_ptr) const;
    const_Point central_symmetry(const PointPtr& point_ptr, double target_dis) const;
    
    /** @brief 以拷贝方式创建当前点位的中心对称点的共享指针 **/
    inline const_PointPtr central_symmetry_ptr() const { return make_shared<const_Point>(2*map_center_x - get_x(), 2*map_center_y - get_y()); };
    inline const_PointPtr central_symmetry_ptr(const PointPtr& point_ptr) const{ return make_shared<const_Point>(2*point_ptr->get_x() - get_x(), 2*point_ptr->get_y() - get_y()); };
};


/** 常量点: 不可变xy坐标 **/
class const_Point: public Point
{
protected:
    string name{};
    string str{};
public:
    double x{};
    double y{};
    
    const_Point() = default;
    const_Point(const const_Point& point) = default;
    const_Point(const double& x, const double& y) : x(x), y(y) { }
    const_Point(double&& x, double&& y) : x(x), y(y) { }
    explicit const_Point(const pair<double, double>& xy) : x(xy.first), y(xy.second) { }
    
    inline double get_x() const override { return x; }
    inline double get_y() const override { return y; }
    
    /**
     * @brief 解析"x,y"格式的文本构造点对象
     * @param text
     */
    explicit const_Point(const string& text)
    {
        size_t pos = text.find(',');
        x = stod(text.substr(0, pos));
        y = stod(text.substr(pos + 1));
    }
    
    string to_str(PrintType print_type) const override
    {
        stringstream ss;
        if ((print_type == PrintType::Simple_With_Registered_Names ||
            print_type == PrintType::Detail_With_Registered_Names  ||
            print_type == PrintType::Normal_With_Registered_Names) && 
            !registered_names.empty())
            ss << get_registered_names();
        if (print_type == PrintType::Detail_Without_Registered_Names ||
            print_type == PrintType::Detail_With_Registered_Names)
            ss << setprecision(2) << fixed << "const_Point" << "(" << get_x() << ", " << get_y() << ")";
        else
            ss << setprecision(2) << fixed << get_x() << "," << get_y();
        
        return ss.str();
    }
};


/** 引用点: 引用外部变量作为动态变化的xy坐标 **/
class ref_Point: public Point
{
private:
    string str{};
    string name{};
protected:
    const volatile double& x;
    const volatile double& y;
public:
    ref_Point(const double& x, const double& y, string  name = ""): name(std::move(name)), x(x), y(y) { }
    string to_str(PrintType print_type) const override
    {
        stringstream ss;
        if ((print_type == PrintType::Simple_With_Registered_Names ||
            print_type == PrintType::Detail_With_Registered_Names  ||
            print_type == PrintType::Normal_With_Registered_Names) && 
            !registered_names.empty())
            ss << get_registered_names();
        if (print_type == PrintType::Detail_Without_Registered_Names ||
            print_type == PrintType::Detail_With_Registered_Names)
            ss << setprecision(2) << fixed << "ref_Point" << "(" << get_x() << ", " << get_y() << ")";
        else
            ss << setprecision(2) << fixed << get_x() << "," << get_y();
        
        return ss.str();
    }
    
    double get_x() const override { return x; }
    double get_y() const override { return y; }
};


/** 定义点共享指针运算符 **/
// 相等
inline bool operator== (const PointPtr& point_ptr_1, const PointPtr& point_ptr_2)
{ return point_ptr_1->get_x() == point_ptr_2->get_x() && point_ptr_1->get_y() == point_ptr_2->get_y(); }
inline bool operator== (const PointPtr& point_ptr, const const_Point& point)
{ return point_ptr->get_x() == point.get_x() && point_ptr->get_y() == point.get_y(); }
inline bool operator== (const const_Point& point, const PointPtr& point_ptr)
{ return point.get_x() == point_ptr->get_x() && point.get_y() == point_ptr->get_y(); }
inline bool operator== (const const_Point& point_1, const const_Point& point_2)
{ return point_1.get_x() == point_2.get_x() && point_1.get_y() == point_2.get_y(); }
// 不相等
inline bool operator!= (const PointPtr& point_ptr_1, const PointPtr& point_ptr_2)
{ return !(point_ptr_1 == point_ptr_2); }
inline bool operator!= (const PointPtr& point_ptr, const const_Point& point)
{ return !(point_ptr == point); }
inline bool operator!= (const const_Point& point, const PointPtr& point_ptr)
{ return !(point == point_ptr); }
inline bool operator!= (const const_Point& point_1, const const_Point& point_2)
{ return !(point_1 == point_2); }
// 作差求向量
inline pair<double, double> operator- (const PointPtr& point_ptr_1, const PointPtr& point_ptr_2)
{ return make_pair(point_ptr_1->get_x() - point_ptr_2->get_x(), point_ptr_1->get_y() - point_ptr_2->get_y()); }
inline pair<double, double> operator- (const const_Point& point_1, const PointPtr& point_ptr_2)
{ return make_pair(point_1.get_x() - point_ptr_2->get_x(), point_1.get_y() - point_ptr_2->get_y()); }
inline pair<double, double> operator- (const PointPtr& point_ptr_1, const const_Point& point_2)
{ return make_pair(point_ptr_1->get_x() - point_2.get_x(), point_ptr_1->get_y() - point_2.get_y()); }
inline pair<double, double> operator- (const const_Point& point_1, const const_Point& point_2)
{ return make_pair(point_1.get_x() - point_2.get_x(), point_1.get_y() - point_2.get_y()); }
// 叉乘
inline double operator* (const pair<double, double>& vec_1, const pair<double, double>& vec_2)
{ return vec_1.first * vec_2.second - vec_1.second * vec_2.first; }
// 求角度
inline double operator/ (const PointPtr& to_point, const PointPtr& from_point)
{ return atan2(to_point->get_y() - from_point->get_y(), to_point->get_x() - from_point->get_x()); }
inline double operator/ (const const_Point& to_point, const const_Point& from_point)
{ return atan2(to_point.get_y() - from_point.get_y(), to_point.get_x() - from_point.get_x()); }
inline double operator/ (const PointPtr& to_point, const const_Point& from_point)
{ return atan2(to_point->get_y() - from_point.get_y(), to_point->get_x() - from_point.get_x()); }
inline double operator/ (const const_Point& to_point, const PointPtr& from_point)
{ return atan2(to_point.get_y() - from_point->get_y(), to_point.get_x() - from_point->get_x()); }
//inline PointPtr operator"" _point(const char* str, size_t len);


// 弧度规则化
inline double rad_rule(double ori_angle)
{
    while (ori_angle > M_PI) ori_angle -= 2 * M_PI;
    while (ori_angle < -M_PI) ori_angle += 2 * M_PI;
    return ori_angle;
}
// 弧度制转角度
inline double rad2deg(double ori_angle)
{ return ori_angle * 180 / M_PI; }
inline pair<double, double> rad2deg(pair<double, double> ori_angle_pair)
{ return {rad2deg(ori_angle_pair.first), rad2deg(ori_angle_pair.second)}; }


/** 区域基类，所有区域类型继承于此 **/
// 继承enable_shared_from_this<Area>, 使得Area对象可以安全地获取指向自身的共享指针
class Area: public Base, public std::enable_shared_from_this<Area>
{
protected:
    friend class AreaManger;
    string str{};
    string register_name{};
//    /**
//     * @brief 初始化区域信息字符串，防止每次打印都调用to_str()
//     * @param with_registered_name 是否使用[registered_names]$前缀，默认不使用
//     */
//    inline void init_str( bool with_registered_name = false){ str = to_str(with_registered_name); }
public:
    static const vector<string> prefix;
public:
    Area() = default;
    /**
     * @brief 实际完成除注册名称之外的区域信息字符串转换的方法
     * @param with_registered_name 是否使用[registered_names]$前缀
     * @return 返回区域信息字符串
     */
    string to_str(PrintType print_type) const override = 0;

//    /**
//     * @brief 获取区域对象内部储存的信息字符串
//     * @param with_registered_name 返回的字符串是否添加[registered_names]$前缀，默认添加
//     * @return 返回区域信息字符串
//     */
//    inline string get_str(bool with_registered_name = true) const { return ((!with_registered_name || registered_names.empty())? "" : get_registered_names()) + str; }
//    /**
//     * @brief 打印区域对象内部储存的信息字符串
//     * @param with_registered_name 打印字符串是否添加[registered_names]$前缀，默认添加
//     */
//    inline void print_str(bool with_registered_name = true) const { cout << get_str(with_registered_name) << endl; }
    
    /**
     * @brief 获取本区域的中心对称区域
     * @return 返回一对中心对称区域共享指针（分别为不包含自身的单中心对称和包括自身的双中心对称）
     */
    virtual pair<AreaPtr, MixAreaPtr> central_symmetry() = 0;
    
    /**
     * @brief 获取本区域对全地图的取反区域
     * @return 返回取反区域共享指针
     */
    AreaPtr invert();
    
    /**
     * @brief 判断点是否在区域内
     * @param x
     * @param y
     * @return 返回true表示点在区域内，否则false
     */
    virtual bool is_include(double x, double y) const = 0;
    inline bool is_include(const PointPtr& point) const{ return is_include(point->get_x(), point->get_y()); }
    inline bool is_include(const const_PointPtr& point) const{ return is_include(point->x, point->y); }
    inline bool is_include(const ref_PointPtr& point) const{ return is_include(point->get_x(), point->get_y()); }
    
    virtual pair<double, double> CCW_sector_from(const PointPtr& point) const = 0;
    inline pair<double, double> CCW_sector_from(const_Point& point) const{ return CCW_sector_from(point.shared_from_this()); }
    inline pair<double, double> CCW_sector_from(const_Point&& point) const{ return CCW_sector_from(make_shared<const_Point>(point.get_xy())); }
};


/** 圆形区域(由圆心+半径构成) **/
class Circle: public Area
{
private:
    PointPtr center;
    double radius;
public:
    static const vector<string> prefix;
    static const vector<string> xml_area_type;
public:
    Circle(const PointPtr& center, const double& radius)
    : center(center->shared_from_this()), radius(radius)
    { assert(radius>0); }
    
    Circle(const double& x, const double& y, const double& radius)
    : center(make_shared<const_Point>(x, y)), radius(radius)
    { assert(radius>0); }
    
    explicit Circle(const vector<double>& data)
    : center(make_shared<const_Point>(data[0], data[1])), radius(data[2])
    { assert(radius>.0); assert(data.size() == 3); }
    
    explicit Circle(const pair<PointPtr, vector<double>>& data)
    : center(data.first->shared_from_this()), radius(data.second[0])
    { assert(radius>.0); assert(data.second.size() == 1); }
    
    string to_str(PrintType print_type) const override
    {
        stringstream ss;
        if ((print_type == PrintType::Simple_With_Registered_Names ||
            print_type == PrintType::Detail_With_Registered_Names  ||
            print_type == PrintType::Normal_With_Registered_Names) && 
            !registered_names.empty())
            ss << get_registered_names();
        if (print_type == PrintType::Simple_Without_Registered_Names ||
            print_type == PrintType::Simple_With_Registered_Names)
            ss << setprecision(2) << fixed << "Circ(" << center->to_str(print_type) << "," << radius << ")";
        else
            ss << setprecision(2) << fixed << "Circle(" << center->to_str(print_type) << ", " << radius << ")";
        return ss.str();
    }
    pair<AreaPtr, MixAreaPtr> central_symmetry() override;
    inline bool is_include(double x, double y) const override
    { return (x - center->get_x()) * (x - center->get_x()) + (y - center->get_y()) * (y - center->get_y()) < radius * radius; }
    
    pair<double, double> CCW_sector_from(const PointPtr &point) const override;
};


/** 多边形区域(由顶点构成) **/
class Polygon: public Area
{
private:
    bool is_convex = false; // 是否为凸多边形
    vector<PointPtr> points{};
    size_t vertex_num{}; // 顶点数量
public:
    static const vector<string> prefix;
    static const vector<string> xml_area_type;
public:
    explicit Polygon(const vector<PointPtr>& points)
    : points(points), vertex_num(points.size())
    { assert(vertex_num > 2); is_convex = convex_judge(); }
    explicit Polygon(const vector<PointPtr>&& points)
    : points(points), vertex_num(points.size())
    { assert(vertex_num > 2); is_convex = convex_judge(); }
    Polygon(const initializer_list<PointPtr>& points)
    : points(points), vertex_num(points.size())
    { assert(vertex_num > 2); is_convex = convex_judge(); }
    
    string to_str(PrintType print_type) const override
    {
        stringstream ss;
        if ((print_type == PrintType::Simple_With_Registered_Names ||
            print_type == PrintType::Detail_With_Registered_Names  ||
            print_type == PrintType::Normal_With_Registered_Names) && 
            !registered_names.empty())
            ss << get_registered_names();
        if (print_type == PrintType::Simple_Without_Registered_Names ||
            print_type == PrintType::Simple_With_Registered_Names)
        {
            ss << setprecision(2) << fixed << "Poly(";
            for (auto& point : points)
                ss << point->to_str(print_type) << ";";
            return ss.str().substr(0, ss.str().size() - 1) + ")";
        }
        else
        {
            ss << setprecision(2) << fixed << "Polygon(";
            for (auto& point : points)
                ss << point->to_str(print_type) << "; ";
            return ss.str().substr(0, ss.str().size() - 2) + ")";
        }
    }
    pair<AreaPtr, MixAreaPtr> central_symmetry() override;
    bool is_include(double x, double y) const override;
    bool convex_judge() const;
    pair<double, double> CCW_sector_from(const PointPtr &point) const override;
};


/** 矩形区域(由中心点+长宽构成) **/
class Rectangle: public Area
{
private:
    PointPtr center;
    mutable vector<const_Point> vertexes;
    double width;
    double height;
    double half_width;
    double half_height;
public:
    static const vector<string> prefix;
    static const vector<string> xml_area_type;
public:
    Rectangle(const PointPtr& center, const double& width, const double& height)
    : center(center->shared_from_this()), width(width), height(height), half_width(width/2), half_height(height/2)
    { assert(width>0 && height>0); }
    
    Rectangle(const double& x, const double& y, const double& width, const double& height)
    : center(make_shared<const_Point>(x, y)), width(width), height(height), half_width(width/2), half_height(height/2)
    { assert(width>0 && height>0); }
    
    explicit Rectangle(const vector<double>& data)
    : center(make_shared<const_Point>(data[0], data[1])), width(data[2]), height(data[3]), half_width(width/2), half_height(height/2)
    { assert(width>0 && height>0); assert(data.size() == 4); }
    
    explicit Rectangle(const pair<PointPtr, vector<double>>& data)
    : center(data.first->shared_from_this()), width(data.second[0]), height(data.second[1]), half_width(width/2), half_height(height/2)
    { assert(width>0 && height>0); assert(data.second.size() == 2); }
    
    string to_str(PrintType print_type) const override
    {
        stringstream ss;
        if ((print_type == PrintType::Simple_With_Registered_Names ||
            print_type == PrintType::Detail_With_Registered_Names  ||
            print_type == PrintType::Normal_With_Registered_Names) && 
            !registered_names.empty())
            ss << get_registered_names();
        if (print_type == PrintType::Simple_Without_Registered_Names ||
            print_type == PrintType::Simple_With_Registered_Names)
            ss << setprecision(2) << fixed << "Rect(" << center->to_str(print_type) << "," << width << "," << height << ")";
        else
            ss << setprecision(2) << fixed << "Rectangle(" << center->to_str(print_type) << ", " << width << ", " << height << ")";
        return ss.str();
    }
    pair<AreaPtr, MixAreaPtr> central_symmetry() override;
    inline bool is_include(double x, double y) const override
    { return x > center->get_x() - half_width && x < center->get_x() + half_width && y > center->get_y() - half_height && y < center->get_y() + half_height; }
    inline void to_polygon() const
    {
        if (vertexes.empty())
        {
            vertexes.resize(4);
            vertexes.emplace_back(center->get_x() + half_width, center->get_y() + half_height);
            vertexes.emplace_back(center->get_x() - half_width, center->get_y() + half_height);
            vertexes.emplace_back(center->get_x() - half_width, center->get_y() - half_height);
            vertexes.emplace_back(center->get_x() + half_width, center->get_y() - half_height);
        }
    }
    pair<double, double> CCW_sector_from(const PointPtr &point) const override;;
};


/** 混合区域(可由多个不同类型区域构成) **/
class MixArea: public Area
{
private:
    vector<pair<bool, AreaPtr>> areas{};
public:
    static const vector<string> prefix;
    static const vector<string> xml_area_type;
public:
    MixArea() = default;
    
    explicit MixArea(const vector<pair<bool, AreaPtr>>& areas) : areas(areas) {  }
    explicit MixArea(const vector<pair<bool, AreaPtr>>&& areas) : areas(areas) {   }
    MixArea(MixArea&& mix_area) noexcept: areas(std::move(mix_area.areas)) {   }
    explicit MixArea(const AreaPtr& area_ptr, bool is_add = true) : areas({{is_add, area_ptr}}) {  }
    
    /**
     * @brief MixArea特殊运算符
     */
    // todo: 考虑对MixArea单独使用vector拼接
    void operator+=(const AreaPtr& area_ptr);
    void operator-=(const AreaPtr& area_ptr);
    MixAreaPtr operator+(const AreaPtr& area_ptr);
    MixAreaPtr operator-(const AreaPtr& area_ptr);
    
    string to_str(PrintType print_type) const override
    {
        stringstream ss;
        if ((print_type == PrintType::Simple_With_Registered_Names ||
            print_type == PrintType::Detail_With_Registered_Names  ||
            print_type == PrintType::Normal_With_Registered_Names) && 
            !registered_names.empty())
            ss << get_registered_names();
        bool is_first = true;
        if (print_type == PrintType::Simple_Without_Registered_Names ||
            print_type == PrintType::Simple_With_Registered_Names)
        {
            ss << setprecision(2) << fixed << "Mix(";
            for (const auto& [is_add, area]: areas)
            {
                ss << (is_first? "": (is_add? "+": "-")) + area->to_str(print_type);
                is_first = false;
            }
        }
        else
        {
            ss << setprecision(2) << fixed << "Mix(";
            for (const auto& [is_add, area]: areas)
            {
                ss << (is_first? "": (is_add? " + ": " - ")) + area->to_str(print_type);
                is_first = false;
            }
        }
        ss << ")";
        return ss.str();
    }
    
    pair<AreaPtr, MixAreaPtr> central_symmetry() override;
    
    bool is_include(double x, double y) const override;
    
    pair<double, double> CCW_sector_from(const PointPtr &point) const override;;
};

/** 定义区域共享指针运算符 **/
inline MixAreaPtr operator+(const AreaPtr& area_ptr1, const AreaPtr& area_ptr2)
{ return make_shared<MixArea>(vector<pair<bool,AreaPtr>>{make_pair(true, area_ptr1), make_pair(true, area_ptr2)}); }
inline MixAreaPtr operator-(const AreaPtr& area_ptr1, const AreaPtr& area_ptr2)
{ return make_shared<MixArea>(vector<pair<bool,AreaPtr>>{make_pair(true, area_ptr1), make_pair(false, area_ptr2)}); }
inline void operator+=(const MixAreaPtr& mix_area, const AreaPtr& area){ *mix_area += area; }
inline void operator-=(const MixAreaPtr& mix_area, const AreaPtr& area){ *mix_area -= area; }


/** 点管理类，用于管理区域对象的注册 **/
class PointManager: public Manager<Point>{};
// 创建内联对象，方便使用[]运算符
inline PointManager point_manager;


/** 区域管理类，用于管理区域对象的注册，以及从xml初始化注册 **/
class AreaManager: public Manager<Area>
{
public:
    /**
     * @brief 解析../xml/semantic.xml文件，初始化注册区域名称
     */
    static void init();
    
//    /**
//     * @brief 注册区域，使用区域类型文本形式，自动解析
//     * @param area_name
//     * @param area_text
//     */
//    static void registerElement(const string& area_name, const string& area_text);
    
    /**
     * @brief 点所在的已注册区域
     * @return
     */
    static vector<string> get_inside_registered_areas(double x, double y){ return {}; }
    
    /**
     * @brief 判断区域名称是否已经注册
     * @param area_name 注册名称
     * @return
     */
    static bool isRegistered(const string& area_name) { return Manager<Area>::isRegistered(area_name); }
};
// 创建内联对象，方便使用[]运算符
inline AreaManager area_manager;


/**
 * @brief 中心对称操作枚举类型
 */
enum class CentralSymmetryType
{
    // 不使用中心对称(即自身)
    SELF = 0,
    // 使用中心对称后的区域
    ONCE,
    // 使用中心对称后的区域与原区域的并集
    BOTH
};


/** 区域文本解析类，用于解析文本得到区域对象 **/
class AreaParser
{
public:
    static const AreaPtr& Total_Map;          // 全地图区域，用于!运算取反
    static const vector<string> true_text;    // 表示真值的文本
    static const vector<string> cs_text;      // 表示单中心对称的文本
    static const vector<string> cs_both_text; // 表示双中心对称的文本
public:
    
    /**
     * @brief 判断文本是否代表真值
     * @param text 待判断的文本
     * @return 返回true则代表文本为真值
     */
    inline static bool is_true(const string& text)
    { return std::any_of(true_text.begin(), true_text.end(), [&](const string& str){ return text == str; }); }
    
    /**
     * @brief 判断文本是否代表合法的区域类型
     * @tparam AreaType 区域类型
     * @param area_type 待校验的区域类型文本
     * @return 返回true则代表区域类型文本合法
     */
    template<typename AreaType>
    inline static bool is_area_type(const string& area_type)
    { return std::any_of(AreaType::xml_area_type.begin(), AreaType::xml_area_type.end(), [&](const string& str){ return str == area_type; }); }
    
    
    /**
     * @brief 去除字符串中的空格
     * @param text 待去除空格的字符串
     */
    inline static void delete_white_space(string& text)
    { text.erase(std::remove(text.begin(), text.end(), ' '), text.end()); };
    /**
     * @brief 去除字符串中的空格
     * @param text 待去除空格的C风格字符串
     */
    inline static string delete_white_space(const char* text)
    {
        string str(text);
        str.erase(std::remove(str.begin(), str.end(), ' '), str.end());
        return str;
    };
    
    /**
     * @brief 分AreaType解析文本
     * @tparam AreaType
     * @param text
     * @return
     */
    template<typename AreaType>
    static bool process_prefix(string& text)
    {
        for (const auto& valid_prefix: AreaType::prefix)
        {
            size_t pos = text.find(valid_prefix);
            if (pos != string::npos)
            {
                size_t start_pos = text.find_first_of('(');
                size_t end_pos = text.find_last_of(')');
                text = text.substr(start_pos+1, end_pos - start_pos - 1);
                return true;
            }
        }
        return false;
    }
    
    /**
     * @brief 通过'!'开头判断是否需要取反
     * @param text
     * @return
     */
    static bool process_exclamation(string& text);
    
    /**
     * @brief 解析":="声明语句，注意非单独使用时需要在声明语句外层加()
     * @note 此语法只会在未注册区域时进行注册，不会修改已注册的区域，若:=的左操作数已注册，\n
     *       则解析右操作数后仍返回未修改的左操作数注册区域本身
     * @param text
     */
    static void process_declaration(string& text);
    
    /**
     * @brief 解析"="赋值语句，注意非单独使用时需要在赋值语句外层加()
     * @note 此语法在未注册区域时进行注册，已注册区域会重新赋值
     * @param text
     */
    static void process_assignment(string& text);
    
    /**
     * @brief 通过'^'后缀解析中心对称类型
     * @param text
     * @param is_registered
     * @return
     */
    static CentralSymmetryType process_suffix(string& text, bool is_registered=false);
    
    /**
     * @brief 字符串以','分隔得到double数组
     * @param raw_text
     * @return 返回double数组
     */
    static vector<double> split_by_comma(const string& raw_text);
    
    /**
     * @brief 字符串以','分隔得到Point对象(首个','前构造)和double数组
     * @param raw_text
     * @return 返回Point对象和double数组的pair
     */
    static pair<PointPtr, vector<double>> split_point_by_comma(const string& raw_text);
    
    /**
     * @brief 字符串以';'分隔得到Point数组
     * @param raw_text
     * @return 返回Point数组
     */
    static vector<PointPtr> split_by_semicolon(const string& raw_text);
    
    /**
     * @brief 字符串以'+'、'-'分隔得到构造MixArea的pair数组
     * @param raw_text
     * @return 返回pair数组
     */
    static vector<pair<bool, AreaPtr>> split_by_add_or_sub(const string& raw_text);
    
    
    /**
     * @brief 文本解析总函数
     * @param raw_text 原始文本
     * @return 返回新创建或已注册区域的共享指针
     */
    static AreaPtr toAreaType(const string& raw_text);
    
    /**
     * @brief 判断点名称是否已注册
     * @param raw_text 目标点注册名称
     * @return 返回true表示已注册，false表示未注册
     */
    static inline bool isValidPointName(const string& raw_text){ return PointManager::isRegistered(raw_text); }
    
    /**
     * @brief 判断区域名称是否已注册
     * @param raw_text 目标区域注册名称
     * @return 返回true表示已注册，false表示未注册
     */
    static inline bool isValidAreaName(const string& raw_text){ return AreaManager::isRegistered(raw_text); }
    
    /**
     * @brief 供区域文本解析器实例化的对象使用的[]操作符，方便解析文本
     * @param raw_text 待解析文本
     * @return 返回区域共享指针
     */
    inline AreaPtr operator[](const string& raw_text){ return toAreaType(raw_text); };
};
// 区域文本解析器内联对象，可使用[]操作符便捷解析文本
inline AreaParser area_parser;
