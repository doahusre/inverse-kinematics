#ifndef MY_SPRING_SIMULATOR_H
#define MY_SPRING_SIMULATOR_H

#include <GLModel/GLModel.h>
#include <shared/defs.h>
#include <util/util.h>
#include "anim.h"
#include "animTcl.h"
#include "BaseSimulator.h"
#include "BaseSystem.h"
#include "ParticleSystem.h"
#include <string>
#include <glm/vec3.hpp>
#include <glm.hpp>



class SpringSimulator : public BaseSimulator
{
public:
	SpringSimulator(const std::string& name);
	int init(double time);
	int step(double time);
	void calculateForces(std::vector<double>& state);

	int command(int argc, myCONST_SPEC char** argv);

protected:
	BaseSystem* system;
	float gravity = -0;
	double prevTime = 0;
	std::vector<Spring> springs;
	int num_springs = 0;
	std::vector<int> fixed_points;
	int integration_type = 0;
	bool isFirstStep = true;
	float timestep = 0.01;
	float globalDrag = 2.0;
	float ks_ground = 0.0;
	float kd_ground = 0.0;
	glm::vec3 ground_point = { 0.0f, 0.0f, 0.0f };
	glm::vec3 ground_normal = { 0.0f, 1.0f, 0.0f };
	std::vector<glm::vec3> prev_positions;
};
 

#endif