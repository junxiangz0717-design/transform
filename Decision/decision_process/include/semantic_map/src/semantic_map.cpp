//
// Created by zh on 24-7-25.
// Update  by tzz
// semantic_map.cpp

#include "semantic_map.h"

#include "decision_package_path.h"
#include "tinyxml2.h" 


/** 默认地图中心坐标，用于中心对称 **/
double map_center_x = 14;
double map_center_y = 7.5;
/** 默认全地图区域，用于区域取反 **/
AreaPtr total_map_ptr = make_shared<Rectangle>(14, 7.5, 28, 15);

/** Point: Base **/
bool Point::is_in(const AreaPtr& area_ptr) const { return area_ptr->is_include(get_x(), get_y()); }

const_Point Point::to_const_Point() const { return {get_x(), get_y()}; }

vector<string> Point::get_inside_registered_areas() const { return std::move(AreaManager::get_inside_registered_areas(get_x(), get_y())); }

const_Point Point::central_symmetry() const { return {2*map_center_x - get_x(), 2*map_center_y - get_y()}; }
const_Point Point::central_symmetry(const const_Point& point) const{ return {2*point.get_x() - get_x(), 2*point.get_y() - get_y()}; }
const_Point Point::central_symmetry(const const_Point& point, double target_dis) const
{
    const double radio = target_dis / (this->distance(point) + target_dis);
    return {(point.get_x() - get_x()*radio) / (1-radio) ,(point.get_y() - get_y()*radio) / (1-radio)}; }
const_Point Point::central_symmetry(const PointPtr& point_ptr) const{ return {2*point_ptr->get_x() - get_x(), 2*point_ptr->get_y() - get_y()}; }
const_Point Point::central_symmetry(const PointPtr& point_ptr, double target_dis) const
{
    const double radio = target_dis / (this->distance(point_ptr) + target_dis);
    return {(point_ptr->get_x() - get_x()*radio) / (1-radio) ,(point_ptr->get_y() - get_y()*radio) / (1-radio)};
}

pair<double, double> Point::CCW_sector_to(const AreaPtr& area_ptr) { return area_ptr->CCW_sector_from(shared_from_this()); }

/** Area: Base **/
AreaPtr Area::invert() { return total_map_ptr - shared_from_this(); }

const vector<string> Area::prefix = {"Area(", "area(", "AREA("};


/** Circle: Area **/

pair<AreaPtr, MixAreaPtr> Circle::central_symmetry()
{
    // 以(14, 7.5)为中心作中心对称
    PointPtr new_center = make_shared<const_Point>(2*map_center_x - center->get_x(), 2*map_center_y - center->get_y());
    CirclePtr circle_CS = make_shared<Circle>(new_center, radius);
    MixAreaPtr circle_both = shared_from_this() + circle_CS;
    return make_pair(circle_CS, circle_both);
}

pair<double, double> Circle::CCW_sector_from(const PointPtr& point) const {
    if (Area::is_include(point))
        return {.0, .0};
    double center_forward = center/point;
    double range_angle = asin(radius/center->distance(point));
    return {rad_rule(center_forward - range_angle), rad_rule(center_forward + range_angle)};
}

const vector<string> Circle::prefix = {"Circle(", "circle(", "CIRCLE(", "Circ(", "circ(", "CIRC(", "C(", "c("};
const vector<string> Circle::xml_area_type = {"Circle", "circle", "CIRCLE", "Circ", "circ", "CIRC", "C", "c"};


/** Polygon: Area **/
pair<AreaPtr, MixAreaPtr> Polygon::central_symmetry()
{
    vector<PointPtr> points_CS;
    // 提前为points_CS分配空间
    points_CS.reserve(this->points.size());
    for (const auto& vertex: this->points){ points_CS.emplace_back(make_shared<const_Point>(2*map_center_x - vertex->get_x(), 2*map_center_y - vertex->get_y())); }
    shared_ptr<Polygon> polygon_CS = make_shared<Polygon>(points_CS);
    MixAreaPtr polygon_both = shared_from_this() + polygon_CS;
    return make_pair(polygon_CS, polygon_both);
}

bool Polygon::is_include(double x, double y) const {
    
    size_t n = points.size();
    bool flag = false;
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        if (x == points[i]->get_x() && y == points[i]->get_y())
            return false;
        if (((points[i]->get_y() > y) != (points[j]->get_y() > y))
            &&
            (x < (points[j]->get_x() - points[i]->get_x()) * (y - points[i]->get_y()) / (points[j]->get_y() - points[i]->get_y()) + points[i]->get_x()))
            flag = !flag;
    }
    return flag;
}

// 判断逆时针点序的多边形是否为凸多边形
bool Polygon::convex_judge() const {
    vector<pair<double, double>> vec;
    for (size_t i = 0, j = vertex_num-1; i < vertex_num; j=i++)
    {
        vec.emplace_back(points[i]-points[j]); // 计算两点作差的向量
        if (i > 0 // 当已经计算出两个向量时开始判断
            &&
            vec[i-1] * vec[i] < 0 // 以两邻边向量转角是否大于180度作为凹多边形判据
            )
            return false;
    }
    cout << "Polygon is convex" << endl;
    return true;
}

pair<double, double> Polygon::CCW_sector_from(const PointPtr& point) const {
    if (Area::is_include(point))
        return {0, 0};
    if (is_convex)
    {
        vector<double> positive_angles{};
        vector<double> negative_angles{};
        for (const auto& vertex_point : points)
        {
            if (double angle = vertex_point/point; angle > 0 )
                positive_angles.emplace_back(angle);
            else if (angle < 0)
                negative_angles.emplace_back(angle);
            else if (vertex_point != point) // 角度等于0时，若不与顶点重合则放入
                positive_angles.emplace_back(.0);
        }
        sort(positive_angles.begin(), positive_angles.end());
        sort(negative_angles.begin(), negative_angles.end());
        // 同号
        if (positive_angles.empty())
            return {negative_angles.front(), negative_angles.back()};
        if (negative_angles.empty())
            return {positive_angles.front(), positive_angles.back()};
        // 异号
        if ((positive_angles.back()-negative_angles.front()) <= M_PI)
            return {negative_angles.front(), positive_angles.back()};
        else
            return {positive_angles.front(), negative_angles.back()};
    }
        // 凹多边形暂时不处理
    else return {.0, .0};
}

const vector<string> Polygon::prefix = {"Polygon(", "polygon(", "POLYGON(", "Poly(", "poly(", "POLY(", "P(", "p("};
const vector<string> Polygon::xml_area_type = {"Polygon", "polygon", "POLYGON", "Poly", "poly", "POLY", "P", "p"};


/** Rectangle: Area **/
pair<AreaPtr, MixAreaPtr> Rectangle::central_symmetry()
{
    // 以(14, 7.5)为中心作中心对称
    PointPtr new_center = make_shared<const_Point>(2*map_center_x - center->get_x(), 2*map_center_y - center->get_y());
    RectanglePtr rectangle_CS = make_shared<Rectangle>(new_center, width, height);
    MixAreaPtr rectangle_both = shared_from_this() + rectangle_CS;
    return make_pair(rectangle_CS, rectangle_both);
}

pair<double, double> Rectangle::CCW_sector_from(const PointPtr& point) const {
    if (Area::is_include(point))
        return {.0, .0};
    to_polygon();
    vector<double> positive_angles{};
    vector<double> negative_angles{};
    for (const auto& vertex_point : vertexes)
    {
        if (double angle = vertex_point/point; angle > 0 )
            positive_angles.emplace_back(angle);
        else if (angle < 0)
            negative_angles.emplace_back(angle);
        else if (vertex_point != point) // 角度等于0时，若不与顶点重合则放入
            positive_angles.emplace_back(.0);
    }
    sort(positive_angles.begin(), positive_angles.end());
    sort(negative_angles.begin(), negative_angles.end());
    // 同号
    if (positive_angles.empty())
        return {negative_angles.front(), negative_angles.back()};
    if (negative_angles.empty())
        return {positive_angles.front(), positive_angles.back()};
    // 异号
    if ((positive_angles.back()-negative_angles.front()) <= M_PI)
        return {negative_angles.front(), positive_angles.back()};
    else
        return {positive_angles.front(), negative_angles.back()};
}

const vector<string> Rectangle::prefix = {"Rectangle(", "rectangle(", "RECTANGLE(", "Rect(", "rect(", "R(", "r("};
const vector<string> Rectangle::xml_area_type = {"Rectangle", "rectangle", "RECTANGLE", "Rect", "rect", "R", "r"};


/** MixArea: Area **/
pair<AreaPtr, MixAreaPtr> MixArea::central_symmetry()
{
    MixAreaPtr new_mixarea = make_shared<MixArea>();
    for (const auto& [is_add, area]: this->areas)
    {
        if (is_add)
            *new_mixarea += area->central_symmetry().first;
        else
            *new_mixarea -= area->central_symmetry().first;
    }
    MixAreaPtr mixarea_both = shared_from_this() + new_mixarea;
    return make_pair(new_mixarea, mixarea_both);
}

void MixArea::operator+=(const AreaPtr& area_ptr){
    this->areas.emplace_back(true, area_ptr);
}
void MixArea::operator-=(const AreaPtr& area_ptr){
    this->areas.emplace_back(false, area_ptr);
}
MixAreaPtr MixArea::operator+(const AreaPtr& area_ptr) {
    *this += area_ptr;
    return std::dynamic_pointer_cast<MixArea>(this->shared_from_this());
}
MixAreaPtr MixArea::operator-(const AreaPtr& area_ptr) {
    *this -= area_ptr;
    return  std::dynamic_pointer_cast<MixArea>(this->shared_from_this());
}

bool MixArea::is_include(double x, double y) const {
    bool is_in = false;
    for (const auto& [is_add, area_ptr]: areas)
        if (area_ptr->is_include(x, y)) is_in = is_add;
    return is_in;
}

pair<double, double> MixArea::CCW_sector_from(const PointPtr& point) const { return {.0, .0}; }

const vector<string> MixArea::prefix = {"MixArea(", "mixarea(", "MIXAREA(", "Mix(", "mix(", "MIX(", "M(", "m("};
const vector<string> MixArea::xml_area_type = {"mix", "Mix", "MIX", "MixArea", "mixarea", "MIXAREA", "mix_area", "M", "m"};


extern template std::map<string, std::shared_ptr<Area>> Manager<Area>::registered_map;
extern template std::map<string, std::shared_ptr<Point>> Manager<Point>::registered_map;
/** PointManager: Manager<Point> **/
//template<>
//map<string, PointPtr> Manager<Point>::map = map<string, PointPtr>();


/** AreaManager: Manager<Area> **/
//template <>
//map<string, AreaPtr> Manager<Area>::map = map<string, AreaPtr>();

//导航也在用这个包，最好不要动源码，值修改文件路径
void AreaManager::init()
{
    using namespace tinyxml2;
    
    XMLDocument doc;
    // 加载xml并输出加载状态
    auto status = doc.LoadFile((DECISION_PACKAGE_PATH_SEMNATIC_MAP_XML+"/26_RMUL.xml").c_str());//
    cout << DECISION_PACKAGE_PATH_SEMNATIC_MAP_XML<<"/26_RMUL.xml加载状态：" << XMLDocument::ErrorIDToName(status) << endl;
    if (status != XML_SUCCESS) return;
    
    // 获取根节点指定加载的地图名称
    XMLElement* root = doc.RootElement();
    const string map_name = root->Attribute("load");
    // 加载指定语义地图集
    XMLElement* map = root->FirstChildElement(map_name.c_str());
    if (map == nullptr)
    {
        cerr << "semantic.xml中未定义决策语义地图集:" << map_name << "!" << endl;
        return;
    }
    else
    {
        // 加载全地图集
        if (auto str = map->Attribute("Total_Map"); str != nullptr)
            total_map_ptr = AreaParser::toAreaType(str);
        else total_map_ptr = make_shared<Rectangle>(14.0, 7.5, 28.0, 14.0 );
        
        registerElement("Total_Map", total_map_ptr);
        // 如指定地图中心点，则重载
        if (auto str = map->Attribute("Center"); str != nullptr)
        {
            auto center = const_Point(str);
            map_center_x = center.get_x();
            map_center_y = center.get_y();
        }
    }
    auto Areas_Element = map->FirstChildElement("Areas");
    assert(Areas_Element != nullptr);
    for (auto child_area = Areas_Element->FirstChildElement(); child_area; child_area = child_area->NextSiblingElement()) {
        const string area_name = child_area->Name(); // 获取标签ID，作为待注册区域名称
        auto area_type_ptr = child_area->Attribute("type");
        AreaPtr origin_area;
        AreaPtr after_cs_area;
        // 按构造函数格式注册区域
        if (area_type_ptr != nullptr)
        {
            string area_type = area_type_ptr;
            // 圆区域
            if (AreaParser::is_area_type<Circle>(area_type))
            {
                auto center = child_area->Attribute("center");
                auto radius  = child_area->Attribute("radius");
                assert(center != nullptr && radius != nullptr);
                origin_area = make_shared<Circle>(make_shared<const_Point>(AreaParser::delete_white_space(center)), stod(radius));
            }
            // 矩形区域
            else if (AreaParser::is_area_type<Rectangle>(area_type))
            {
                auto center = child_area->Attribute("center");
                auto width  = child_area->Attribute("width");
                auto height = child_area->Attribute("height");
                assert(center != nullptr && width != nullptr && height != nullptr);
                origin_area = make_shared<Rectangle>(make_shared<const_Point>(AreaParser::delete_white_space(center)), stod(width), stod(height));
            }
            // 多边形区域
            else if (AreaParser::is_area_type<Polygon>(area_type))
            {
                auto points = child_area->Attribute("points");
                assert(points != nullptr);
                origin_area = make_shared<Polygon>(AreaParser::split_by_semicolon(AreaParser::delete_white_space(points)));
            }
            // 混合区域
            else if (AreaParser::is_area_type<MixArea>(area_type))
            {
                auto areas = child_area->Attribute("areas");
                assert(areas != nullptr);
                string areas_str = "MixArea(" + string(areas) + ")";
                origin_area = AreaParser::toAreaType(areas_str);
            }
            else
                cerr << "未定义的区域类型:" << area_type << endl;
        }
        // 用表达式注册区域
        else
        {
            auto code = child_area->Attribute("code");
            assert(code != nullptr);
            origin_area = AreaParser::toAreaType(code);
        }
        
        // 获取单中心对称区域
        if (any_of(AreaParser::cs_text.begin(), AreaParser::cs_text.end(),
                   [&](const string& str) {
            return child_area->Attribute(str.c_str()) != nullptr && AreaParser::is_true(child_area->Attribute(str.c_str()));
        }))
            after_cs_area = origin_area->central_symmetry().first;
        // 获取双中心对称区域
        else if (any_of(AreaParser::cs_both_text.begin(), AreaParser::cs_both_text.end(),
                        [&](const string& str) {
            return child_area->Attribute(str.c_str()) != nullptr && AreaParser::is_true(child_area->Attribute(str.c_str()));
        }))
            after_cs_area = origin_area->central_symmetry().second;
        // 不中心对称
        else
            after_cs_area = origin_area;
        
        // 取反
        if (auto invert = child_area->Attribute("invert"); invert != nullptr && AreaParser::is_true(invert))
            after_cs_area = after_cs_area->invert();
        
        registerElement(area_name, after_cs_area);
    }
    
    // 打印注册区域信息
    printAll();
}

//void AreaManager::registerElement(const string& area_name, const string& area_text) {
//    // 解析文本
//    auto area_ptr = AreaParser::toAreaType(area_text);
//    Manager<Area>::registerElement(area_name, area_ptr);
//}


/** AreaParser **/
const AreaPtr& AreaParser::Total_Map = total_map_ptr;
const vector<string> AreaParser::true_text = { "true", "True", "TRUE",
                                               "on",    "On" ,  "ON",
                                               "yes",   "Yes", "YES",
                                               "y",     "Y",    "1"};
const vector<string> AreaParser::cs_text = { "cs", "CS", "CentralSymmetry", "centralsymmetry", "central_symmetry" };
const vector<string> AreaParser::cs_both_text = {"both","Both" , "BOTH", "both_cs", "Both_CS",  "cs_both", "CS_Both", "CentralSymmetryBoth", "centralsymmetryboth", "central_symmetry_both" };

bool AreaParser::process_exclamation(string& text) {
    // 判断是否需要取反（以'!'开头）
    if (text.find_first_of('!') == 0)
    {
        // 不允许连用'!'取反
        assert(text.find("!!") == string::npos);
        text = text.substr(1);
        return true;
    }
    else
        return false;
}

void AreaParser::process_declaration(string& text) {
    // 判断是否需要声明区域
    size_t declaration_pos = text.find(":=");
    if (declaration_pos == string::npos)
        return;
    
    string id;                  // 声明语句中注册名称
    string declaration_text;    // 声明语句中区域定义文本
    size_t id_start_pos = 0;        // 声明语句中注册名称的起始位置
    auto declaration_iter = text.begin() + declaration_pos + 2; // 从':='后一位开始遍历
    int left_parenthesis = 0;
    int right_parenthesis = 0;
    for (; declaration_iter < text.end(); declaration_iter++)
    {
        switch (*declaration_iter)
        {
            case '(':
                left_parenthesis++;
                continue;
            case ')':
                right_parenthesis++;
                
                // 在AreaType(())中嵌套使用时
                if (left_parenthesis == right_parenthesis - 1)
                {
                    // 通过获取声明名称起始位置
                    id_start_pos = text.find_last_of('(', declaration_pos);
                    // 截取名称、声明
                    id = text.substr(id_start_pos + 1, declaration_pos - id_start_pos -1);
                    declaration_text = text.substr(declaration_pos + 2, (declaration_iter - text.begin()) - declaration_pos - 2);
                    
                    // tips: 先解析再判断是否已经注册，因为解析文本中可能先包含同注册名称的声明语句如"B5:=((B5:=B4+A3)+B4)"
                    if (auto area_ptr = AreaParser::toAreaType(declaration_text); !AreaManager::isRegistered(id))
                        // 注册未注册区域
                        AreaManager::registerElement(id, area_ptr);
                    
                    // 将声明语句部分替换为注册名称
                    text.replace(id_start_pos, (declaration_iter - text.begin()) - id_start_pos + 1, id);
                    return;
                }
                continue;
            default:
                continue;
        }
    }
    
    // 单独使用时
    id = text.substr(id_start_pos, declaration_pos - id_start_pos);
    declaration_text = text.substr(declaration_pos + 2);
    
    // tips: 先解析再判断是否已经注册，因为解析文本中可能先包含同注册名称的声明语句如"B5:=((B5:=B4+A3)+B4)"
    if (auto area_ptr = AreaParser::toAreaType(declaration_text); !AreaManager::isRegistered(id))
        // 注册未注册区域
        AreaManager::registerElement(id, area_ptr);
    
    // 将声明语句部分替换为注册名称
    text.replace(id_start_pos, (declaration_iter - text.begin()) - id_start_pos, id);
}

void AreaParser::process_assignment(string& text)
{
    // 判断是否需要赋值区域
    size_t assignment_pos = text.find('=');
    if (assignment_pos == string::npos)
        return;
    
    string id;                  // 赋值语句中注册名称
    string assignment_text;     // 赋值语句中区域定义文本
    size_t id_start_pos = 0;        // 赋值语句中注册名称的起始位置
    auto assignment_iter = text.begin() + assignment_pos + 1; // 从'='后一位开始遍历
    int left_parenthesis = 0;
    int right_parenthesis = 0;
    for (; assignment_iter < text.end(); assignment_iter++)
    {
        switch (*assignment_iter)
        {
            case '(':
                left_parenthesis++;
                continue;
            case ')':
                right_parenthesis++;
                
                // 在AreaType(())中嵌套使用时
                if (left_parenthesis == right_parenthesis - 1)
                {
                    // 通过获取声明名称起始位置
                    id_start_pos = text.find_last_of('(', assignment_pos);
                    // 截取名称、声明
                    id = text.substr(id_start_pos + 1, assignment_pos - id_start_pos - 1);
                    assignment_text = text.substr(assignment_pos + 1, (assignment_iter - text.begin()) - assignment_pos - 1);
                    // 赋值注册区域（不论是否已注册）
                    AreaManager::registerElement(id, AreaParser::toAreaType(assignment_text));
                    // 将赋值语句部分替换为注册名称
                    text.replace(id_start_pos, (assignment_iter - text.begin()) - id_start_pos + 1, id);
                    return;
                }
                continue;
            default:
                continue;
        }
    }
    
    // 单独使用时
    id = text.substr(id_start_pos, assignment_pos - id_start_pos);
    assignment_text = text.substr(assignment_pos + 1);
    // 赋值注册区域（不论是否已注册）
    AreaManager::registerElement(id, AreaParser::toAreaType(assignment_text));
    // 将赋值语句部分替换为注册名称
    text.replace(id_start_pos, (assignment_iter - text.begin()) - id_start_pos, id);
}

CentralSymmetryType AreaParser::process_suffix(string& text, bool is_registered) {
    string text_without_suffix;
    string suffix;
    
    size_t last_suffix_pos = text.find_last_of('^');
    // 无后缀则直接返回SELF
    if (last_suffix_pos == string::npos)
        return CentralSymmetryType::SELF;
    
    if (!is_registered)
    {
        // 断言最后的'^'一定在最后的')'后一位
        size_t end_pos = text.find_last_of(')');
        assert(last_suffix_pos == end_pos + 1);
    }
    
    // 截取字符串最后一个'^'后的部分作为后缀
    suffix = text.substr(last_suffix_pos + 1);
    // 截取字符串最后一个'^'前的部分作为区域类型文本
    text_without_suffix = text.substr(0, last_suffix_pos);
    // 判断区域名称是否已经注册，并移除后缀
    if(!is_registered || AreaManager::isRegistered(text_without_suffix))
        text = text_without_suffix;
    
    if (suffix == "CS" || suffix == "cs" || suffix == "CentralSymmetry" || suffix == "centralsymmetry")
        return CentralSymmetryType::ONCE;
    else if (suffix == "both" || suffix == "Both" || suffix == "BOTH")
        return CentralSymmetryType::BOTH;
    else
        throw std::runtime_error(text + "带有不支持的区域类型后缀:^" + suffix);
}

pair<PointPtr, vector<double>> AreaParser::split_point_by_comma(const string& raw_text) {
    vector<double> result{};
    stringstream ss(raw_text);
    string text;
    // 断言待解析点名已经注册
    getline(ss, text, ',');
    assert(isValidPointName(text));
    auto center_point_ptr = PointManager::getElementByName(text);
    while (getline(ss, text, ','))
        result.emplace_back(stod(text));
    return {center_point_ptr, result};
}

vector<double> AreaParser::split_by_comma(const string& raw_text) {
    vector<double> result{};
    stringstream ss(raw_text);
    string text;
    while (getline(ss, text, ','))
        result.emplace_back(stod(text));
    return result;
}

vector<PointPtr> AreaParser::split_by_semicolon(const string& raw_text) {
    vector<PointPtr> result;
    stringstream ss(raw_text);
    string text;
    while (getline(ss, text, ';'))
    {
        // 如果是已注册点名则直接加入
        if (isValidPointName(text))
            result.emplace_back(PointManager::getElementByName(text));
        // 否则创建临时点并加入
        result.emplace_back(make_shared<const_Point>(text));
    }
    return result;
}

vector<pair<bool, AreaPtr>> AreaParser::split_by_add_or_sub(const string& raw_text) {
    vector<pair<bool, AreaPtr>> result;
    stringstream ss(raw_text);
    string start_with_add_text;
    string atomic_text;
    
    if (raw_text.find('(') != string::npos || raw_text.find(')') != string::npos)
    {
        // 有意义的原子字符串开始位置
        size_t start_pos = 0;
        // 有意义的'+'、'-'运算符匹配位置（两运算符之间左右小括号数量匹配）
        size_t match_pos = 0;
        int left_parentheses = 0;
        int right_parentheses = 0;
        bool is_add = true;
        for (auto i: raw_text) {
            switch (i) {
                case '(':
                    left_parentheses++;
                    break;
                case ')':
                    // 增加右括号计数并判断是否和左括号数量匹配
                    right_parentheses++;
                    break;
                case '+':
                case '-':
                    // 判断是否为'+'、'-'运算符
                    if (left_parentheses == right_parentheses)
                    {
                        result.emplace_back(is_add, toAreaType(raw_text.substr(start_pos, match_pos - start_pos)));
                        is_add = i == '+';
                        // 断言'+'、'-'运算符不连续出现
                        assert(raw_text[start_pos] != '+' && raw_text[start_pos] != '-');
                        // 设置下一个原子字符串的起点
                        start_pos = match_pos + 1;
                    }
                    break;
                default:
                    break;
            }
            match_pos += 1;
        }
        // 最后一个原子字符串
        result.emplace_back(is_add, toAreaType(raw_text.substr(start_pos, match_pos - start_pos)));
    }
    /** 以下为旧代码，已废弃，只考虑l'+''-'运算符的匹配，不能处理嵌套MixArea(相邻两个加减运算符在原子字符串MixArea(...)的括号内) **/
    else
    {
        // 获取'+'分隔的字符串
        while (getline(ss, start_with_add_text, '+'))
        {
            // 首个原子字符串默认为'+'，以及每次循环开头的原子字符串必为'+'（因为上一次getline遇到'+'才停止）
            bool is_add = true;
            stringstream ss_sub{start_with_add_text};
            // 用'-'二次分隔字符串，由于已经不含有'+'，所以得到原子字符串
            while (getline(ss_sub, atomic_text, '-'))
            {
                // '-'号分隔得到的原子字符串首个必为'+'号，其后必为'-'号
                result.emplace_back(is_add, toAreaType(atomic_text));
                is_add = false;
            }
        }
    }
    return result;
}

AreaPtr AreaParser::toAreaType(const string& raw_text) {
    AreaPtr origin_area;
    AreaPtr after_CS_area;
    AreaPtr result;
    
    assert(!raw_text.empty());
    string text{raw_text};
    // 去除空格
    delete_white_space(text);
    assert(!text.empty());
    
    process_declaration(text);  // 声明部分注册与替换
    process_assignment(text);   // 赋值部分注册、重新赋值与替换
    
    bool is_invert = process_exclamation(text); // 是否用'!'取反
    CentralSymmetryType CS_type; // 中心对称类型
    
    /** 已注册的区域处理 **/
    if (isValidAreaName(text))
        return AreaManager::getElementByName(text);
        // 判断是否为注册区域+后缀
    else
    {
        CS_type = process_suffix(text, true);
        if (isValidAreaName(text))
        {
            origin_area = AreaManager::getElementByName(text);
            // 中心对称处理
            switch (CS_type)
            {
                case CentralSymmetryType::ONCE:
                    after_CS_area = origin_area->central_symmetry().first;
                    break;
                case CentralSymmetryType::BOTH:
                    after_CS_area = origin_area->central_symmetry().second;
                    break;
                default:
                    throw runtime_error(text + "带有无法识别的区域类型后缀");
            }
            // 取反处理
            return is_invert ? make_shared<MixArea>(Total_Map - after_CS_area) : after_CS_area;
        }
    }
    
    /** 未注册的区域处理 **/
    int add_count = 0;          // '+'的计数器
    int sub_count = 0;          // '-'的计数器
    int left_count = 0;         // '('的计数器
    int right_count = 0;        // ')'的计数器
    int hat_count = 0;          // '^'的计数器
    int exclamation_count = 0;  // '!'的计数器
    int comma_count = 0;        // ','的计数器

    for (char c : text) {
        if (c == '(') ++left_count;
        else if (c == ')') ++right_count;
        else if (c == '+') ++add_count;
        else if (c == '-') ++sub_count;
        else if (c == '^') ++hat_count;
        else if (c == '!') ++exclamation_count;
        else if (c == ',') ++comma_count;
    }
    // 断言'('和')'的数量相等，且不为0(非注册区域必须被AreaType()包括)
    assert((left_count == right_count) && left_count);
    // 下面的断言废弃：因为会阻止"MixArea(m(c(1,2,3)))"这种写法
//    // 断言若未使用+/-运算符，则'('和')'的数量最多为1
//    assert(add_count || sub_count || left_count <= 1);
    
    // 获取中心对称类型并移除^后缀
    CS_type = process_suffix(text);
    
    // MixArea类型处理
    if (process_prefix<MixArea>(text))
        origin_area = make_shared<MixArea>(split_by_add_or_sub(text));
    // 其他区域类型处理
    else
    {
        // 一些关于非MixArea类型的断言
        // 断言非MixArea类型未使用+\-运算符，已移除，会误判："poly(-1,-1;-1,1;1,1;1,-1)"
        // assert(((add_count || sub_count) == false));
        // 断言'!'前缀只能出现在开头且最多出现一次
        assert(exclamation_count == 0 || (exclamation_count == 1 && text.find('!') == 0));
        // 断言'^'后缀最多出现一次
        assert(hat_count <= 1);
        
        if (process_prefix<Circle>(text)) {
            // 断言圆形区域的构造函数的参数个数为2或3
            assert(comma_count < 3 && comma_count > 0);
            // 1个','表示使用使用点作为圆心和半径2个参数初始化Circle
            if (comma_count == 1)
                origin_area = make_shared<Circle>(split_point_by_comma(text));
            // 2个','表示使用圆心x、y坐标和半径3个参数初始化Circle
            else
                origin_area = make_shared<Circle>(split_by_comma(text));
        }
        else if (process_prefix<Rectangle>(text)) {
            // 断言矩形区域的构造函数的参数个数为3或4
            assert(comma_count < 4 && comma_count > 1);
            // 2个','表示使用点作为矩形中心和长、宽3个参数初始化Rectangle
            if (comma_count == 2)
                origin_area = make_shared<Rectangle>(split_point_by_comma(text));
            // 3个','表示使用矩形中心x、y坐标和长、宽4个参数初始化Rectangle
            else
                origin_area = make_shared<Rectangle>(split_by_comma(text));
        }
        else if (process_prefix<Polygon>(text)) {
            origin_area = make_shared<Polygon>(split_by_semicolon(text));
        }
        else { throw runtime_error("无法识别的区域类型:" + raw_text); }
    }
    // 中心对称处理
    switch (CS_type)
    {
        case CentralSymmetryType::SELF:
            after_CS_area = origin_area;
            break;
        case CentralSymmetryType::ONCE:
            after_CS_area = origin_area->central_symmetry().first;
            break;
        case CentralSymmetryType::BOTH:
            after_CS_area = origin_area->central_symmetry().second;
            break;
    }
    // 取反处理
    result = is_invert ? make_shared<MixArea>(Total_Map - after_CS_area) : after_CS_area;
    
    return result;
}
