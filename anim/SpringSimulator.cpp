#include "SpringSimulator.h"

SpringSimulator::SpringSimulator(const std::string& name) : 
	BaseSimulator(name) 
{
	system = nullptr;
}

int SpringSimulator::init(double time) {
	if (system == nullptr) {
		animTcl::OutputMessage("No system linked to the simulator");
		return TCL_ERROR;
	}
	system->reset(time);
	prevTime = time;
	isFirstStep = true;
	// Initialize prev_positions vector
	ParticleSystem* ps = dynamic_cast<ParticleSystem*>(system);
	int num_particles = ps->getNumParticles();
	prev_positions.resize(num_particles);
	return TCL_OK;
}
//spring force: F = ks * (restlength - |p1 - p2|) * (p1 - p2) / |p1 - p2|
//damper force: F = -kd * ((v1 - v2) * (p1 - p2) / |p1 - p2|) * ((p1 - p2) / |p1 - p2|)
//gravity force: F = m * g
void SpringSimulator::calculateForces(std::vector<double>& state) {
	
	int num_particles = state.size() / 10;

	for (int i = 0; i < num_particles; i++) {
		// Reset forces
		state[i * 10 + 7] = 0.0f;
		state[i * 10 + 8] = 0.0f;
		state[i * 10 + 9] = 0.0f;

		glm::vec3 v = glm::vec3(state[i * 10 + 3], state[i * 10 + 4], state[i * 10 + 5]);
		float collision_detection = glm::dot(glm::vec3(state[i * 10], state[i * 10 + 1], state[i * 10 + 2]) - ground_point, ground_normal);
		//if (collision_detection < 0.01) continue;

		// add gravity force
		float m = state[i * 10 + 6];
		glm::vec3 gravity_force = glm::vec3(0.0f, gravity * m, 0.0f);
		state[i * 10 + 7] += gravity_force.x;
		state[i * 10 + 8] += gravity_force.y;
		state[i * 10 + 9] += gravity_force.z;


		// Add ground collision response
		if (collision_detection < 0.01) {
			glm::vec3 normal_force = -ks_ground * collision_detection * ground_normal - kd_ground * glm::dot(v, ground_normal) * ground_normal;
			state[i * 10 + 7] += normal_force.x;
			state[i * 10 + 8] += normal_force.y;
			state[i * 10 + 9] += normal_force.z;
			if (collision_detection > 0 && (timestep - 0.01) < 0.001) { // hack for stopping jittering
				state[i * 10 + 8] = 0.0f; // y force to 0
				state[i * 10 + 4] = 0.0f; // y velocity to 0
			}
		}

		// Add global drag force
		glm::vec3 drag_force = -globalDrag * v;
		state[i * 10 + 7] += drag_force.x;
		state[i * 10 + 8] += drag_force.y;
		state[i * 10 + 9] += drag_force.z;
	}

	// Calculate spring forces
	for (int i = 0; i < springs.size(); i++) {
		Spring s = springs[i];
		glm::vec3 p1 = glm::vec3(state[s.index1 * 10], state[s.index1 * 10 + 1], state[s.index1 * 10 + 2]);
		glm::vec3 p2 = glm::vec3(state[s.index2 * 10], state[s.index2 * 10 + 1], state[s.index2 * 10 + 2]);
		glm::vec3 v1 = glm::vec3(state[s.index1 * 10 + 3], state[s.index1 * 10 + 4], state[s.index1 * 10 + 5]);
		glm::vec3 v2 = glm::vec3(state[s.index2 * 10 + 3], state[s.index2 * 10 + 4], state[s.index2 * 10 + 5]);
		float m1 = state[s.index1 * 10 + 6];
		float m2 = state[s.index2 * 10 + 6];
		glm::vec3 diff = p1 - p2;
		float dist = glm::length(diff);
		glm::vec3 force = s.ks * (s.restlength - dist) * (diff / dist);
		glm::vec3 damp_force = -s.kd * (glm::dot((v1 - v2), diff / dist)) * (diff / dist);
		state[s.index1 * 10 + 7] += force.x + damp_force.x;
		state[s.index1 * 10 + 8] += force.y + damp_force.y;
		state[s.index1 * 10 + 9] += force.z + damp_force.z;
		state[s.index2 * 10 + 7] -= force.x + damp_force.x;
		state[s.index2 * 10 + 8] -= force.y + damp_force.y;
		state[s.index2 * 10 + 9] -= force.z + damp_force.z;
	}
}



int SpringSimulator::step(double time) {
	double dt = time - prevTime;
	prevTime = time;

	ParticleSystem* ps = dynamic_cast<ParticleSystem*>(system);
	int num_particles = ps->getNumParticles();
	int state_size = 10 * num_particles;
	std::vector<double> state(state_size);

	ps->getState(state.data());

	// calculate forces
	calculateForces(state);

	// integrate and update positions
	for (int i = 0; i < num_particles; i++) {
		if (fixed_points[i]) continue; // Skip fixed points

		glm::vec3 p = glm::vec3(state[i * 10 + 0], state[i * 10 + 1], state[i * 10 + 2]);
		glm::vec3 v = glm::vec3(state[i * 10 + 3], state[i * 10 + 4], state[i * 10 + 5]);
		glm::vec3 f = glm::vec3(state[i * 10 + 7], state[i * 10 + 8], state[i * 10 + 9]);

		float m = state[i * 10 + 6];
		glm::vec3 a = f / m;

		float deltaTime = timestep;

		
		// Update position based on integration type (e.g., Euler, Symplectic Euler, or Verlet)
		glm::vec3 new_p = glm::vec3(0.0f);
		glm::vec3 new_v = glm::vec3(0.0f);
		glm::vec3 prev_p = prev_positions.at(i);

		if (integration_type == 0) {
			// Forward Euler integration
			new_v = v + a * deltaTime;
			new_p = p + v * deltaTime;
		}
		else if (integration_type == 1) {
			// Symplectic Euler integration
			new_v = v + a * deltaTime;
			new_p = p + new_v * deltaTime;
		}
		else if (integration_type == 2) {
			// Verlet integration
			if (isFirstStep) {
				// First step of Verlet as Euler
				new_v = v + a * deltaTime;
				new_p = p + new_v * deltaTime;
			}
			else {
				// Regular Verlet
				 new_p = 2.0f * p - prev_p + a * deltaTime * deltaTime;
				 new_v = (new_p - prev_p) / (2.0f * deltaTime);
			}
		}

		// update previous position
		prev_positions.at(i) = glm::vec3(state[i * 10], state[i * 10 + 1], state[i * 10 + 2]);

		// Store the updated position and velocity in the state array
		state[i * 10 + 0] = new_p[0];
		state[i * 10 + 1] = new_p[1];
		state[i * 10 + 2] = new_p[2];
		state[i * 10 + 3] = new_v[0];
		state[i * 10 + 4] = new_v[1];
		state[i * 10 + 5] = new_v[2];
	}

	if (isFirstStep) {
		isFirstStep = false;
	}

	ps->setState(state.data());
	return TCL_OK;
}

int SpringSimulator::command(int argc, myCONST_SPEC char** argv) {
	if (argc < 1) {
		return TCL_ERROR;
	}
	else if (strcmp(argv[0], "link") == 0) {
		// simulator <sim_name> link <sys name> <Number of Springs>
		// This links the simulator to a particular particle system 
		// and initializes it to work with a given number of springs.

		system = GlobalResourceManager::use()->getSystem(argv[1]);
		if (system == nullptr) {
			animTcl::OutputMessage("System not found");
			return TCL_ERROR;
		}
		if (atoi(argv[2]) < 0) {
			animTcl::OutputMessage("Number of springs must be a positive number");
			return TCL_ERROR;
		}
		num_springs = atoi(argv[2]);

		ParticleSystem* ps = dynamic_cast<ParticleSystem*>(system);
		int num_particles = ps->getNumParticles();
		for (int i = 0; i < num_particles; i++) {
			fixed_points.push_back(0);
		}
		prev_positions.resize(num_particles); // Initialize prev_positions vector
	}
	else if (strcmp(argv[0], "spring") == 0) {
		// simulator <sim_name> spring <index1> <index2> <ks> <kd> <restlength>
		// This sets up a given spring. If the rest length is a negative number, the system 
		// should automatically set the rest length of the spring to the distance between 
		// the corresponding particles at the time the command is given.
		if (num_springs == springs.size()) {
			animTcl::OutputMessage("Number of springs exceeded");
			return TCL_ERROR;
		}
		Spring s;
		s.index1 = atoi(argv[1]);
		s.index2 = atoi(argv[2]);
		s.ks = atof(argv[3]); // spring constant
		s.kd = atof(argv[4]); // damping constant
		s.restlength = atof(argv[5]);
		if (s.restlength < 0) {
			ParticleSystem* ps = dynamic_cast<ParticleSystem*>(system);
			int num_particles = ps->getNumParticles();
			int state_size = 10 * num_particles;
			std::vector<double> state(state_size);
			ps->getState(state.data());
			glm::vec3 p1 = glm::vec3(state[s.index1 * 10], state[s.index1 * 10 + 1], state[s.index1 * 10 + 2]);
			glm::vec3 p2 = glm::vec3(state[s.index2 * 10], state[s.index2 * 10 + 1], state[s.index2 * 10 + 2]);
			glm::vec3 diff = p1 - p2;
			s.restlength = glm::length(diff);
		}
		springs.push_back(s);

		ParticleSystem* ps = dynamic_cast<ParticleSystem*>(system);
		if (ps) {
			ps->setSprings(springs);
		}
	}
	else if (strcmp(argv[0], "fix") == 0) {
		// simulator <sim_name> fix <index>
		// This command nails particle <index> to its current position.
		int index = atoi(argv[1]);
		if (index >= fixed_points.size()) {
			animTcl::OutputMessage("index out of range. max index %d", fixed_points.size());
			return TCL_ERROR;
		}
		fixed_points[index] = 1;

	}
	else if (strcmp(argv[0], "integration") == 0) {
		// simulator <sim_name> integration <euler|symplectic|verlet> <time step>
		// This changes the integration technique used by the given simulator and sets
		// the time step of the integration. 
		if (strcmp(argv[1], "euler") == 0) {
			integration_type = 0;
		}
		else if (strcmp(argv[1], "symplectic") == 0) {
			integration_type = 1;
		}
		else if (strcmp(argv[1], "verlet") == 0) {
			integration_type = 2;
		}
		else {
			animTcl::OutputMessage("Invalid integration type");
			return TCL_ERROR;
		}
		timestep = atof(argv[2]);
	}
	else if (strcmp(argv[0], "ground") == 0) {
		// simulator <sim_name> ground <ks> <kd>
		// Sets the parameters of the penalty forces applied to particles that try to go underground.
		ks_ground = atof(argv[1]);
		kd_ground = atof(argv[2]);
	}
	else if (strcmp(argv[0], "gravity") == 0) {
		// simulator <sim_name> gravity <g>
		// Sets the acceleration due to gravity, in unit length per unit time squared.
		gravity = atof(argv[1]);
	}
	else if (strcmp(argv[0], "drag") == 0) {
		// simulator <sim_name> drag <kdrag>
		// Sets the global drag (friction) coefficient (Fi = -kdrag vi).  
		// The command expects a positive number 
		if (atof(argv[1]) < 0) {
			animTcl::OutputMessage("Drag coefficient must be a positive number");
			return TCL_ERROR;
		}
		globalDrag = atof(argv[1]);
	}
	return TCL_OK;
}
