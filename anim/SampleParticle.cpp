#include "SampleParticle.h"

SampleParticle::SampleParticle( const std::string& name ):
	BaseSystem( name ),
	m_sx( 1.0f ),
	m_sy( 1.0f ),
	m_sz( 1.0f )
{ 

	setVector( m_pos, 0, 0, 0 );

	double x = .4;
	double y = .7;
	double z = 0;

	for (int i = 0; i < NUM_PARTICLES; i++) {
		int k = i * NUM_PROPERTIES;
		double degrees = rand() % 360;
		double radians = degrees * (PI / 180);
		double x_dir = x * cos(radians) + z * sin(radians);
		double y_dir = y;
		double z_dir = -x * sin(radians) + z * cos(radians);

		particles[k] = 0; // position
		particles[k + 1] = -3;
		particles[k + 2] = 0;
		particles[k + 3] = x_dir; // velocity
		particles[k + 4] = y_dir;
		particles[k + 5] = z_dir;
	 	particles[k + 6] = i / 50.0; // time till activation

		// save initial state
		inital_particles[k] = particles[k + 0];
		inital_particles[k + 1] = particles[k + 1];
		inital_particles[k + 2] = particles[k + 2];
		inital_particles[k + 3] = particles[k + 3];
		inital_particles[k + 4] = particles[k + 4];
		inital_particles[k + 5] = particles[k + 5];
		inital_particles[k + 6] = particles[k + 6];
	}
	
}	// SampleParticle


void SampleParticle::getState( double* p )
{ 
	for (int i = 0; i < NUM_PARTICLES * NUM_PROPERTIES; i++) {
		p[i] = particles[i];
	}
	//VecCopy( p, m_pos ); 

}	// SampleParticle::getState

void SampleParticle::setState( double  *p )
{ 
	for (int i = 0; i < NUM_PARTICLES * NUM_PROPERTIES; i++) {
		particles[i] = p[i];
	}
	//VecCopy(m_pos,p); 

}	// SampleParticle::setState

void SampleParticle::reset( double time ) 
{ 
	for (int i = 0; i < NUM_PARTICLES * NUM_PROPERTIES; i++) {
		particles[i] = inital_particles[i];
	}
	//setVector( m_pos, 0, 0, 0 ); 
	
}	// SampleParticle::Reset


int SampleParticle::command(int argc, myCONST_SPEC char **argv) 
{
	if( argc < 1 )
	{
		animTcl::OutputMessage("system %s: wrong number of params.", m_name.c_str()) ;
		return TCL_ERROR ;
	}
	else if (strcmp(argv[0], "dim") == 0) {
		// system <system_name> dim <num_particles>
		// initailizes the particle system to hold up to the given number of particles
	}
	else if (strcmp(argv[0], "particle") == 0) {
		// system <system_name> particle <index> <mass> <x y z vx vy vz>
		// sets position mass and velocity for a given particle
	}
	else if (strcmp(argv[0], "all_velocities") == 0) {
		// system <system_name> all_velocities <vx vy vz>
		// sets the velocity for all particles
	}
	else if (strcmp(argv[0], "link") == 0) {
		// system <
	}
	else if( strcmp(argv[0], "read") == 0 )
	{
		if( argc == 2 )
		{
			m_model.ReadOBJ(argv[1]) ;
			glmFacetNormals(&m_model) ;
			glmVertexNormals(&m_model, 90) ;
			return TCL_OK ;
		}
		else 
		{
			animTcl::OutputMessage("Usage: read <file_name>") ;
			return TCL_ERROR ;
		}
	}
	else if( strcmp(argv[0], "scale") == 0 )
	{
		if( argc == 4 )
		{
			m_sx = (float)atof(argv[1]) ;
			m_sy = (float)atof(argv[2]) ;
			m_sz = (float)atof(argv[3]) ;
		}
		else
		{
			animTcl::OutputMessage("Usage: scale <sx> <sy> <sz> ") ;
			return TCL_ERROR ;

		}
	}
	else if( strcmp(argv[0], "pos") == 0 )
	{
		if( argc == 4 )
		{
			m_pos[0] = atof(argv[1]) ;
			m_pos[1] = atof(argv[2]) ;
			m_pos[2] = atof(argv[3]) ;
		}
		else
		{
			animTcl::OutputMessage("Usage: pos <x> <y> <z> ") ;
			return TCL_ERROR ;

		}
	}
	else if( strcmp(argv[0], "flipNormals") == 0 )
	{
		flipNormals() ;
		return TCL_OK ;
		
	}
	else if( strcmp(argv[0], "reset") == 0)
	{
		double p[3] = {0,0,0} ;
		setState(p) ;
	}
    
    glutPostRedisplay() ;
	return TCL_OK ;

}	// SampleParticle::command

void SampleParticle::display( GLenum mode ) 
{
	glEnable(GL_LIGHTING) ;
	glMatrixMode(GL_MODELVIEW) ;
	//glPushMatrix() ;
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	//glTranslated(m_pos[0],m_pos[1],m_pos[2]) ;
	//glScalef(m_sx,m_sy,m_sz) ;

	//if( m_model.numvertices > 0 )
	//	glmDraw(&m_model, GLM_SMOOTH | GLM_MATERIAL);
	//else
	//	glutSolidSphere(1.0,20,20) ;

	//glPopMatrix();
	 //draw particles as spheres
	for (int i = 0; i < NUM_PARTICLES; i++) {
		int k = i * NUM_PROPERTIES;
		glPushMatrix();
		glTranslated(particles[k], particles[k + 1], particles[k + 2]);
		glutSolidSphere(0.1, 20, 20);
		glPopMatrix();
	}

	glPopAttrib();

}	// SampleParticle::display
