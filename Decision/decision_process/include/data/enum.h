#pragma once
//枚举类赋值说明
//   Style style1 = Style::CHARGE; // 使用枚举名称赋值
//   Style style2 = static_cast<Style>(1); // 使用 static_cast 进行显式转换

enum Target
{
    HERO,
    ENGINEER,
    FOOT_3,
    FOOT_4,
    OUTPOST,
    BASE,
    SENTRY,
    NONE,
};

enum AMMO
{
    NO_AMMO,
    LOW_AMMO,
    HIGH_AMMO,
};

enum State
{
    ATTACKING ,     //攻击
    FINDING,        //搜寻
    BACKING,        //回家
    TREATING,       //治疗
    TO_FORT,        //前往堡垒
    IN_FORT,        //在堡垒
    ATTACK_OUTPOST, //攻击前哨
    INSTRUCTION,    //云台手指令中
};

enum Style
{
    CHARGE,   //激进
    NORMAL,   //正常
    DEFENCE, //防守
};


enum Tripod_Spin //云台自旋模式
{  
    PI,                //180度
    TWO_PI,            //360度
};

//方便输出
string State_to_str(State state)
{
    switch (state)
    {
    case ATTACKING:
        return "攻击";
    case FINDING:
        return "搜寻";
    case BACKING:
        return "回家";
    case TREATING:
        return "治疗";
    case TO_FORT:
        return "前往堡垒";
    case IN_FORT:
        return "在堡垒";
    default:
        throw invalid_argument("State_to_str: Undefined State");
    }
}

string Style_to_str(Style style)
{
    switch (style)
    {
    case CHARGE:
        return "激进";
    case NORMAL:
        return "正常";
    case DEFENCE:
        return "守家";
    default:
        throw invalid_argument("Style_to_str: Undefined Style");
    }
}

string AMMO_to_str(AMMO ammo)
{
    switch (ammo)
    {
    case NO_AMMO:
        return "无弹药";
    case LOW_AMMO:
        return "弹药不足";
    case HIGH_AMMO:
        return "弹药充足";
    default:
        throw invalid_argument("AMMO_to_str: Undefined AMMO");
    }
}


string Target_to_str(Target target)
{
    switch (target)
    {
    case HERO:
        return "英雄";
    case ENGINEER:
        return "工程";
    case FOOT_3:
        return "步兵3";
    case FOOT_4:
        return "步兵4";
    case OUTPOST:
        return "前哨";
    case BASE:
        return "基地";
    case SENTRY:
        return "哨兵";
    case NONE:
        return "空";
    default:
        throw invalid_argument("Target_to_str: Undefined Target");
    }
}

string Tripod_Mode_to_str(Tripod_Spin tripodMode)
{
    switch (tripodMode)
    {
    case Tripod_Spin::PI:
        return "180度自转";
    case Tripod_Spin::TWO_PI:
        return "360度自转";
    default:
        throw invalid_argument("Tripod_Mode_to_str: Undefined Tripod Mode");
    }
}
