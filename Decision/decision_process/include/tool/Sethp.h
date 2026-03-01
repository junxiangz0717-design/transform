// enemy_health_manager.h
#pragma once
#include <chrono>

class EnemyHealthManager 
{
public:
    // 无敌状态的特殊返回值
    static constexpr int INVINCIBLE_STATE = 1001;
    
    // 更新敌人血量并返回显示值
    int updateHealth(int newHealth) 
    {
        // 记录当前时间
        auto now = std::chrono::steady_clock::now();
                // 记录上次血量
        lastHealth = currentHealth;
        // 更新当前血量
        currentHealth = newHealth;
        // 处理无敌状态逻辑
        if (isInvincibleFlag) 
        {
            // 计算无敌剩余时间
            auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - invincibleStartTime).count();
            
            // 检查无敌时间是否已过
            if (elapsedMs >= invincibleDurationMs) 
            {
                isInvincibleFlag = false;
            }
            // 如果无敌状态下血量增加，解除无敌状态
            else if (newHealth > lastHealth) 
            {
                isInvincibleFlag = false;
            }
        }
    
        
        // 检测复活状态
        if (lastHealth == 0 && currentHealth > 0) 
        {
            if (currentHealth < 100) 
            {
                isInvincibleFlag = true;
                invincibleStartTime = now;
                invincibleDurationMs = 10000;  // 10秒
            } 
            else 
            {
                // 第二种复活方式：>=100，3秒无敌
                isInvincibleFlag = true;
                invincibleStartTime = now;
                invincibleDurationMs = 3000;   // 3秒
            }
        }
        
        // 返回显示血量（无敌时返回特殊值，否则返回真实值）
        return isInvincibleFlag ? INVINCIBLE_STATE : currentHealth;
    }

    int update_sentry_Health(int newHealth) 
    {
        // 记录当前时间
        auto now = std::chrono::steady_clock::now();
        // 记录上次血量
        lastHealth = currentHealth;
        // 更新当前血量
        currentHealth = newHealth;
        // 处理无敌状态逻辑
        if (isInvincibleFlag) 
        {
            // 计算无敌剩余时间
            auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - invincibleStartTime).count();
            
            // 检查无敌时间是否已过
            if (elapsedMs >= invincibleDurationMs) {
                isInvincibleFlag = false;
            }
            // 如果无敌状态下血量增加，解除无敌状态
            else if (newHealth > lastHealth) {
                isInvincibleFlag = false;
            }
        }
        

        
        // 检测复活状态
        if (lastHealth == 0 && currentHealth > 0) {
            if (currentHealth < 100) 
            {
                isInvincibleFlag = true;
                invincibleStartTime = now;
                invincibleDurationMs = 60000;  // 60秒
            } else {
                // 第二种复活方式：>=100，3秒无敌
                isInvincibleFlag = true;
                invincibleStartTime = now;
                invincibleDurationMs = 3000;   // 3秒
            }
        }
        
        // 返回显示血量（无敌时返回特殊值，否则返回真实值）
        return isInvincibleFlag ? INVINCIBLE_STATE : currentHealth;
    }
    
    // 检查敌人是否处于无敌状态
    bool isInvincible()
    {
        return isInvincibleFlag;
    }

private:
    int currentHealth = 0;
    bool isInvincibleFlag = false;
    std::chrono::steady_clock::time_point invincibleStartTime;
    int invincibleDurationMs = 0;  // 无敌持续时间（毫秒）
    int lastHealth = 0;            // 上次更新的血量
};