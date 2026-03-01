#include <chrono>
#include <mutex>
using namespace std;
class EnergyCalculator 
{
private:
    double currentPower;        
    double totalEnergy;           
    std::chrono::time_point<std::chrono::high_resolution_clock> lastUpdateTime;
    mutable std::mutex mutex;     

public:
    EnergyCalculator() : currentPower(0.0), totalEnergy(0.0) 
    {
        reset();
    }

    void updatePower(double power) 
    {
        std::lock_guard<std::mutex> lock(mutex);
        
        auto now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> timeDiff = now - lastUpdateTime;
        double timeInterval = timeDiff.count(); 
    
        double energyIncrement = (currentPower + power) / 2.0 * timeInterval;
        totalEnergy += energyIncrement;

        currentPower = power;
        lastUpdateTime = now;
    }

    double getCurrentPower() const 
    {
        std::lock_guard<std::mutex> lock(mutex);
        return currentPower;
    }

    double getTotalEnergy() const 
    {
        std::lock_guard<std::mutex> lock(mutex);
        return totalEnergy;
    }
    void reset() 
    {
        std::lock_guard<std::mutex> lock(mutex);
        currentPower = 0.0;
        totalEnergy = 0.0;
        lastUpdateTime = std::chrono::high_resolution_clock::now();
    }
};