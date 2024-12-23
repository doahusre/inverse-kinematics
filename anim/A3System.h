#ifndef MY_A3_SYSTEM_H
#define MY_A3_SYSTEM_H




#include "BaseSystem.h"
#include <shared/defs.h>
#include <util/util.h>
#include "animTcl.h"
#include <GLmodel/GLmodel.h>
#include "shared/opengl.h"
#include <vector>
#include <glm/glm.hpp>
#undef Success
#include <Eigen/Dense>

struct Bob {
	glm::vec3 Troot;
	glm::vec3 Tsh;
	glm::vec3 Rsh;
	glm::vec3 Tel;
	glm::vec3 Rel;
	glm::vec3 Twr;
	glm::vec3 Rwr;
	glm::vec3 phand;
	glm::vec3 currentP;
	glm::vec3 targetP;

	Bob() {
		Troot = glm::vec3(0, -.8, 8);
		Tsh = glm::vec3(0.4 * 3, 1.2 * 3, 0);
		Rsh = glm::vec3(0, 0, 0);
		Tel = glm::vec3(2 * 3, 0, 0);
		Rel = glm::vec3(0, 0, 0);
		Twr = glm::vec3(2 * 3, 0, 0);
		Rwr = glm::vec3(0, 0, 0);
		phand = glm::vec3(0.9 * 3, 0, 0);
		currentP = glm::vec3(0, 0, 0);
		targetP = glm::vec3(0, 0, 0);
	}
};

class A3System : public BaseSystem
{
public:
	A3System(const std::string& name);
	virtual void getState(double* p);
	virtual void setState(double* p);
	void setRest();
	void reset(double time);
	void display(GLenum mode = GL_RENDER);
	int command(int argc, myCONST_SPEC char** argv);

	void drawBob();
	void drawBackground();
	void drawEllipse(double x, double y);

	Eigen::Matrix4d A3System::T(glm::vec3 p);
	Eigen::Matrix4d Rx(double theta);
	Eigen::Matrix4d Ry(double theta);
	Eigen::Matrix4d Rz(double theta);
	Eigen::Matrix4d Rxd(double theta);
	Eigen::Matrix4d Ryd(double theta);
	Eigen::Matrix4d Rzd(double theta);
	
	void getcurrentP();
	Eigen::MatrixXd calcJ();
	void IKSolver();
	void IKSolve(Eigen::Vector3d targetP);


protected:
	Bob* bob = new Bob();
};


#endif
