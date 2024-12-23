#ifndef MY_PARTICLE_SIMULATOR_H
#define MY_PARTICLE_SIMULATOR_H

#include <GLModel/GLModel.h>
#include <shared/defs.h>
#include <util/util.h>
#include "animTcl.h"
#include "BaseSimulator.h"
#include "BaseSystem.h"

#include <string>

class ParticleSimulator : public BaseSimulator

{
public:
	ParticleSimulator(const std::string& name, BaseSystem* target);
	~ParticleSimulator();

	int step(double time);
	int init(double time)
	{
		m_object->getState(m_pos0);
		setVector(m_vel0, 0, 0, 0);
		return 0;
	};

	int command(int argc, myCONST_SPEC char** argv) { return TCL_OK; }

protected:
	Vector m_pos0; // initial position
	Vector m_vel0; // initial velocity
	Vector m_pos;
	Vector m_vel;
	double m_mass;

	BaseSystem* m_object;
};


#endif // !MY_PARTICLE_SIMULATOR_H
