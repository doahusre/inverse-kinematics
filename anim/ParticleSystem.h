#ifndef MY_PARTICLE_SYSTEM_H
#define MY_PARTICLE_SYSTEM_H

#include "BaseSystem.h"
#include <shared/defs.h>
#include <util/util.h>
#include "animTcl.h"
#include <GLmodel/GLmodel.h>
#include <vector>
#include <glm/vec3.hpp>

#include "shared/opengl.h"

struct Particle {
	double mass;
	glm::vec3 position;
	glm::vec3 velocity;
	glm::vec3 force;
};

struct Spring {
	int index1;
	int index2;
	float ks;
	float kd;
	float restlength;
};

class ParticleSystem : public BaseSystem
{
public:
	ParticleSystem(const std::string& name);
	virtual void getState(double* p);
	virtual void setState(double* p);
	void reset(double time);
	void display(GLenum mode = GL_RENDER);
	int command(int argc, myCONST_SPEC char** argv);


	void resetForces() {
		for (auto& p : particles) {
			p.force = glm::vec3(0.0f);
		}
	}

	void applyForce(int i, glm::vec3 f) {
		if (i < 0 || i >= particles.size()) {
			animTcl::OutputMessage("Invalid particle index");
			return;
		}
		particles.at(i).force += f;
	}

	void setSprings(const std::vector<Spring>& new_springs) {
		springs = new_springs;
	}

	int getNumParticles() {
		return particles.size();
	}

protected:
	std::vector<Particle> particles;
	std::vector<Particle> initial_particles;
	std::vector<Spring> springs;
};


#endif