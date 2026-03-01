#pragma once

#include <vector>
#include <chrono>
#include <cassert>

using namespace std;

class zh_timer
{
private:
    bool stop_flag_ = true; // 关闭超时器标志位
    bool pause_flag_ = true; // 暂停计时器标志位
    chrono::system_clock::time_point start_time_point_ = chrono::system_clock::now();
    chrono::system_clock::time_point last_restart_time_point_ = chrono::system_clock::now();
    chrono::milliseconds total_duration_ = chrono::milliseconds(0);
    int timeout_ = 0;
    
    int children_num = 0;
    vector<zh_timer> children_timer = {};
    
    int idx_in_father = 0;
    zh_timer* p_father = nullptr;
    
public:
    zh_timer() = default;
    zh_timer(int timeout): timeout_(timeout)
    {
        if (timeout > 0)
            reset();
        // 若超时时间小于等于0则认为不开启受击检测
        else stop_flag_ = true;
        // todo:若超时时间小于0则直接超时
    }
    zh_timer(int timeout, vector<zh_timer>&& vec):timeout_(timeout)
    {
        children_timer.clear();
        int idx = 0;
        for (auto&& child: vec)
        {
            child.idx_in_father = idx;
            child.p_father = this;
            children_timer.emplace_back(child);
            idx++;
        }
        this->children_num = idx;
        restart();
    }
    
    inline zh_timer& getChild(int idx){ return children_timer.at(idx); }
    
    inline int getTimeoutSetting()const{ return timeout_; }
    
    inline void stop(){ this->stop_flag_ = true; }
    
    inline void next()
    {
        this->stop();
        
        if (p_father != nullptr)
        {
            // 如果不为父计时器的最后一个子计时器则开启下一相邻计时器
            if (this->idx_in_father < p_father->children_num-1)
                p_father->getChild(idx_in_father+1).restart();
            // 如果为父计时器的最后一个子计时器则停止父计时器
            else p_father->stop();
        }
    }
    
    /**
     * @brief 载入新超时阈值
     * @param timeout
     */
    inline void operator() (int timeout)
    {
        timeout_ = timeout;
        
        if (timeout_ > 0) reset();
        else
        {
            stop_flag_ = true;
            pause_flag_ = true;
        }
    }
    
    inline void operator()(std::initializer_list<int> args_list)
    {
        // 断言初始化表参数数量等于主+子计时器数量
        int args_num = int(args_list.size());
        assert(args_num == this->children_num + 1);
        
        // 重新载入主计时器参数
        auto it = args_list.begin();
        this->operator()(*it);
        
        // 如果有子节点则向子节点载入参数
        if (children_num > 0)
        {
            it++;
            for (int idx = 0; it != args_list.end() ; it++, idx++)
            {
                this->getChild(idx).operator()(*it);
            }
        }
    }
    
    /**
     * @brief 复位计时器
     */
    inline void reset()
    {
        if (timeout_ > 0)
        {
            stop_flag_ = false;
            pause_flag_ = false;
            start_time_point_ = chrono::system_clock::now();
            last_restart_time_point_ = chrono::system_clock::now();
            total_duration_ = chrono::milliseconds(0);
        }
        
        if (children_timer.empty()) return;
        // 若有子计时器则重置它们
        else
        {
            for (auto& child: children_timer)
            {
                child.reset();
            }
        }
    }
    
    /**
     * @brief 超时检测
     * @return 是否超时
     */
    inline bool is_timeout() const
    {
        // 超时器停止则认为不超时
        if (stop_flag_) return false;
        
        // 若有子超时器则对所有子超时器做超时检测
        if (!children_timer.empty())
        {
            for (const auto& child: children_timer)
            {
                if (child.is_timeout())
                    return true;
            }
        }
        
        if (this->is_1_timeout() || this->is_2_timeout())
            return true;
        return false;
    }
    
    /**
     * @brief 从reset开始的连续计时
     */
    inline int MilSecFromReset() const
    { return int(chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now() - start_time_point_).count()); }
    inline double SecFromReset() const
    { return double(MilSecFromReset()) / 1000; }
    
    /**
     * @brief 从reset开始的连续计时
     */
    inline bool is_1_timeout() const
    {
        if (stop_flag_) return false;
        return MilSecFromReset() >= timeout_;
    }
    
    /**
     * @brief 暂停计时器并向断续总计时加入上一次restart()到现在的时间间隔
     */
    inline void pause()
    {
        // 若断续计时器不处于暂停状态则暂停并计入断续计时
        if (!pause_flag_)
        {
            pause_flag_ = true;
            total_duration_ += chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now() - last_restart_time_point_);
        }
        
        if (!children_timer.empty())
        {
            for (auto& child: children_timer)
            {
                child.pause();
            }
        }
    }
    
    /**
     * @brief 重启被pause()暂停的断续累加计时，不影响计时起点
     */
    inline void restart()
    {
        
        pause_flag_ = false;
        last_restart_time_point_ = chrono::system_clock::now();
        
        if (!children_timer.empty())
        {
            for (auto& child: children_timer)
            {
                child.restart();
            }
        }
    }
    
    inline int MilSecTotalOfEverytime() const
    {
        // 已被计入断续总时长的计时
        auto cul_duration = int(total_duration_.count());
        if (pause_flag_)
            return cul_duration;
        
        else
        {
            // 因为restart()后未pause()而未被计入总时长的计时
            auto no_cul_duration = int(chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now() - last_restart_time_point_).count());
            return cul_duration + no_cul_duration;
        }
    }
    /**
     * @brief 受pause()、restart()影响的断续累加计时
     */
    inline bool is_2_timeout() const
    {
        if (stop_flag_) return false;
        return MilSecTotalOfEverytime() >= timeout_;
    }
};