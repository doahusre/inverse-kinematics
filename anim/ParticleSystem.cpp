#include "ParticleSystem.h"

ParticleSystem::ParticleSystem(const std::string& name):
	BaseSystem(name)
{
	
}

void ParticleSystem::getState(double* p) {
	for (int i = 0; i < particles.size(); i++) {
		p[i * 10 + 0] = particles.at(i).position[0];
		p[i * 10 + 1] = particles.at(i).position[1];
		p[i * 10 + 2] = particles.at(i).position[2];
		p[i * 10 + 3] = particles.at(i).velocity[0];
		p[i * 10 + 4] = particles.at(i).velocity[1];
		p[i * 10 + 5] = particles.at(i).velocity[2];
		p[i * 10 + 6] = particles.at(i).mass;
		p[i * 10 + 7] = particles.at(i).force[0];
		p[i * 10 + 8] = particles.at(i).force[1];
		p[i * 10 + 9] = particles.at(i).force[2];
	}
}

void ParticleSystem::setState(double* p) {
	for (int i = 0; i < particles.size(); i++) {
		particles.at(i).position = glm::vec3(p[i * 10 + 0], p[i * 10 + 1], p[i * 10 + 2]);
		particles.at(i).velocity = glm::vec3(p[i * 10 + 3], p[i * 10 + 4], p[i * 10 + 5]);
	}
}

void ParticleSystem::reset(double time) {
	for (int i = 0; i < particles.size(); i++) {
		particles.at(i).position = initial_particles.at(i).position;
		particles.at(i).velocity = initial_particles.at(i).velocity;
		particles.at(i).mass = initial_particles.at(i).mass;
	}
}

void ParticleSystem::display(GLenum mode) {
	glPointSize(5.0);
	glBegin(GL_POINTS);
	for (Particle p : particles) {
		glVertex3d(p.position[0], p.position[1], p.position[2]);
	}
	glEnd();

	glColor3f(1.0f, 0.0f, 0.0f); // red
	glBegin(GL_LINES);
	for (const Spring& s : springs) {
		glm::vec3 p1 = particles[s.index1].position;
		glm::vec3 p2 = particles[s.index2].position;
		glVertex3d(p1.x, p1.y, p1.z);
		glVertex3d(p2.x, p2.y, p2.z);
	}
	glEnd();
}

int ParticleSystem::command(int argc, myCONST_SPEC char** argv) {
	if (argc < 1) {
		animTcl::OutputMessage("Available commands: ");
		animTcl::OutputMessage("system <sys_name> dim <Number of particles>");
		animTcl::OutputMessage("system <sys_name> particle <index> <mass> <x y z vx vy vz>");
		animTcl::OutputMessage("system <sys_name> all_velocities  <vx vy vz>");
		return TCL_ERROR;
	}
	else if (strcmp(argv[0], "dim") == 0) {
		// system <sys_name> dim <Number of particles>
		// This command initializes the particle system to hold up to 
		// the given number of particles. 
		// Particles may be initialized to the origin of the world, 
		// or may not appear until added by the next command.
		int num_particles = atoi(argv[1]);
		animTcl::OutputMessage("number of particles %d", num_particles);
		for (int i = 0; i < num_particles; i++) {

			Particle p, ip;

			p.mass = 0.1;
			ip.mass = 0.1;

			glm::vec3 zeroVec = glm::vec3(0.0f, 0.0f, 0.0f);

			p.position = zeroVec;
			ip.position = zeroVec;

			p.velocity = zeroVec;
			ip.velocity = zeroVec;

			p.force = zeroVec;
			ip.force = zeroVec;

			particles.push_back(p);
			initial_particles.push_back(ip);
		}
	}
	else if (strcmp(argv[0], "particle") == 0) {
		// system <sys_name> particle <index> <mass> <x y z vx vy vz>
		// This command sets a position, mass, and velocity for a given particle.
		if (argc != 9) {
			animTcl::OutputMessage("wrong number of parameteres");
			return TCL_ERROR;
		}
		int index = atoi(argv[1]);
		if (index >= particles.size()) {
			animTcl::OutputMessage("index out of range. max index %d", particles.size());
			return TCL_ERROR;
		}
		double m = atof(argv[2]);
		if (m <= 0) {
			animTcl::OutputMessage("mass must be bigger than 0");
			return TCL_ERROR;
		}
		double x = atof(argv[3]);
		double y = atof(argv[4]);
		double z = atof(argv[5]);
		double vx = atof(argv[6]);
		double vy = atof(argv[7]);
		double vz = atof(argv[8]);
		particles.at(index).mass = m;
		particles.at(index).position = glm::vec3(x, y, z);
		particles.at(index).velocity = glm::vec3(vx, vy, vz);
		// save initial state
		initial_particles.at(index).mass = m;
		initial_particles.at(index).position = glm::vec3(x, y, z);
		initial_particles.at(index).velocity = glm::vec3(vx, vy, vz);
	}
	else if (strcmp(argv[0], "all_velocities") == 0) {
		// system <sys_name> all_velocities  <vx vy vz>
		// This command sets the velocity of all particles.
		if (argc != 4) {
			animTcl::OutputMessage("wrong number of parameters");
			return TCL_ERROR;
		}
		glm::vec3 v = glm::vec3(atof(argv[1]), atof(argv[2]), atof(argv[3]));
		for (Particle& p : particles) {
			p.velocity = v;
		}
		for (Particle& p : initial_particles) {
			p.velocity = v;
		}
	}
	return TCL_OK;
}
