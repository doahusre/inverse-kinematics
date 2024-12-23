#include "SampleGravitySimulator.h"

SampleGravitySimulator::SampleGravitySimulator( const std::string& name, BaseSystem* target ):
	BaseSimulator( name ),
	m_object( target )
{
}	// SampleGravitySimulator

SampleGravitySimulator::~SampleGravitySimulator()
{
}	// SampleGravitySimulator::~SampleGravitySimulator

int SampleGravitySimulator::step(double time)
{
	animTcl::OutputMessage("Gravity simulator stepping at time %f", time);
	double g = -9.8;
	double particles[NUM_PARTICLES * NUM_PROPERTIES];
	m_object->getState(particles);
	for (int i = 0; i < NUM_PARTICLES; i++) {
		int k = i * NUM_PROPERTIES;
		if (particles[k + 6] > time) {
			particles[k + 6] += 0;
		}
		else {
			particles[k + 0] += particles[k + 3] * (time - particles[k + 6]);

			particles[k + 1] += particles[k + 4] * (time - particles[k+6]) + 0.5 * g * (time - particles[k + 6]) * (time - particles[k + 6]);

			particles[k + 2] += particles[k + 5] * (time - particles[k + 6]);

			//animTcl::OutputMessage("Particle %d: %f %f %f", i, particles[k + 0], particles[k + 1], particles[k + 2]);
		}
	}
    
	m_object->setState(particles);

	return 0;

}	// SampleGravitySimulator::step
