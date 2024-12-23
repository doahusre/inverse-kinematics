#ifndef MY_A3_SIMULATOR_H
#define MY_A3_SIMULATOR_H

#include "BaseSimulator.h"
#include "BaseSystem.h"
#include "A3System.h"
#include "Hermite.h"

#include <glm/glm.hpp>

class A3Simulator : public BaseSimulator
{
public:
    A3Simulator(const std::string& name, BaseSystem* target);

    int step(double time) override;
    int init(double time) override;
    int command(int argc, myCONST_SPEC char** argv) override;

protected:
    BaseSystem* m_object; 
    Hermite* hermite;    

    double hermite_t;
    double prev_t;

    bool transition;
};

#endif