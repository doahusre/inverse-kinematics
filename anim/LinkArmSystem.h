#ifndef MY_LINK_ARM_SYSTEM_H
#define MY_LINK_ARM_SYSTEM_H
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

class LinkArmSystem : public BaseSystem
{
public:
	LinkArmSystem(const std::string& name);
	virtual void getState(double* p);
	virtual void setState(double* p);
	void reset(double time);

	void display(GLenum mode = GL_RENDER);
	int command(int argc, myCONST_SPEC char** argv);
};



#endif
