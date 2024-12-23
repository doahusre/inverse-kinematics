#include "A3System.h"

A3System::A3System(const std::string& name) :
	BaseSystem(name)
{
}

void A3System::getState(double* p) {
	getcurrentP();
	p[0] = bob->currentP.x;
	p[1] = bob->currentP.y;
	p[2] = bob->currentP.z;
}

void A3System::setState(double* p) {
	bob->targetP = glm::vec3(p[0], p[1], p[2]);
	IKSolver();
}

void A3System::setRest() {
	bob->Rsh = glm::vec3(-15, 5 , -24);
	bob->Rel = glm::vec3(-40, 95, 0);
	bob->Rwr = glm::vec3(0, 15, -25);
}

void A3System::reset(double time) {

}

int A3System::command(int argc, myCONST_SPEC char** argv) {
	if (argc < 1) {
		animTcl::OutputMessage("Available commands: ");
	}
	else if (strcmp(argv[0],"position")==0) {
		bob->Troot = glm::vec3(atof(argv[1]), atof(argv[2]), atof(argv[3]));
	}
	else if (strcmp(argv[0], "currentp") == 0) {
		getcurrentP();
		animTcl::OutputMessage("Current position: %f %f %f", bob->currentP.x, bob->currentP.y, bob->currentP.z);
	}
	else {
		animTcl::OutputMessage("Unknown command: %s", argv[0]);
	}
	return 0;
}


Eigen::Matrix4d A3System::T(glm::vec3 p) {
	Eigen::Matrix4d m;
	m << 1, 0, 0, p.x,
		0, 1, 0, p.y,
		0, 0, 1, p.z,
		0, 0, 0, 1;
	return m;
}

Eigen::Matrix4d A3System::Rx(double theta) {
	Eigen::Matrix4d m;
	m << 1, 0, 0, 0,
		0, cos(theta * PI / 180), -sin(theta * PI / 180), 0,
		0, sin(theta * PI / 180), cos(theta * PI / 180), 0,
		0, 0, 0, 1;
	return m;
}

Eigen::Matrix4d A3System::Ry(double theta) {
	Eigen::Matrix4d m;
	m << cos(theta * PI / 180), 0, sin(theta * PI / 180), 0,
		0, 1, 0, 0,
		-sin(theta * PI / 180), 0, cos(theta * PI / 180), 0,
		0, 0, 0, 1;
	return m;
}

Eigen::Matrix4d A3System::Rz(double theta) {
	Eigen::Matrix4d m;
	m << cos(theta * PI / 180), -sin(theta * PI / 180), 0, 0,
		sin(theta * PI / 180), cos(theta * PI / 180), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
	return m;
}

Eigen::Matrix4d A3System::Rxd(double theta) {
	Eigen::Matrix4d m;
	m << 1, 0, 0, 0,
		0, -sin(theta * PI / 180), -cos(theta * PI / 180), 0,
		0, cos(theta * PI / 180), -sin(theta * PI / 180), 0,
		0, 0, 0, 1;
	return m;
}

Eigen::Matrix4d A3System::Ryd(double theta) {
	Eigen::Matrix4d m;
	m << -sin(theta * PI / 180), 0, cos(theta * PI / 180), 0,
		0, 1, 0, 0,
		-cos(theta * PI / 180), 0, -sin(theta * PI / 180), 0,
		0, 0, 0, 1;
	return m;
}

Eigen::Matrix4d A3System::Rzd(double theta) {
	Eigen::Matrix4d m;
	m << -sin(theta * PI / 180), -cos(theta * PI / 180), 0, 0,
		cos(theta * PI / 180), -sin(theta * PI / 180), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
	return m;
}

void A3System::getcurrentP() {
	Eigen::Vector4d phand = Eigen::Vector4d(bob->phand.x, bob->phand.y, bob->phand.z, 1);
	Eigen::Vector4d currentP = T(bob->Troot) * T(bob->Tsh) * Rz(bob->Rsh.z) * Ry(bob->Rsh.y) * Rx(bob->Rsh.x) * T(bob->Tel) * Ry(bob->Rel.y) * Rx(bob->Rel.x) * T(bob->Twr) * Ry(bob->Rwr.y) * Rz(bob->Rwr.z) * phand;
	bob->currentP = glm::vec3(currentP.x(), currentP.y(), currentP.z());
	//animTcl::OutputMessage("Computed currentP: %f, %f, %f", bob->currentP.x, bob->currentP.y, bob->currentP.z);
}

Eigen::MatrixXd A3System::calcJ() {
	Eigen::MatrixXd J(3, 7); 
	Eigen::Vector4d phand(bob->phand.x, bob->phand.y, bob->phand.z, 1.0);

	// Compute Jacobian columns
	for (int i = 0; i < 7; i++) {
		Eigen::Matrix4d d_transform = Eigen::Matrix4d::Identity();

		if (i == 0) {
			// Shoulder X derivative
			d_transform =  T(bob->Troot) * 
				T(bob->Tsh) * Rz(bob->Rsh.z) * Ry(bob->Rsh.y) * Rxd(bob->Rsh.x) *
				T(bob->Tel) * Ry(bob->Rel.y) * Rx(bob->Rel.x) *
				T(bob->Twr) * Ry(bob->Rwr.y) * Rz(bob->Rwr.z);
		}
		else if (i == 1) {
			// Shoulder Y derivative
			d_transform = T(bob->Troot) *
				T(bob->Tsh) * Rz(bob->Rsh.z) * Ryd(bob->Rsh.y) * Rx(bob->Rsh.x) *
				T(bob->Tel) * Ry(bob->Rel.y) * Rx(bob->Rel.x) *
				T(bob->Twr) * Ry(bob->Rwr.y) * Rz(bob->Rwr.z);
		}
		else if (i == 2) {
			// Shoulder Z derivative
			d_transform = T(bob->Troot) *
				T(bob->Tsh) * Rzd(bob->Rsh.z) * Ry(bob->Rsh.y) * Rx(bob->Rsh.x) *
				T(bob->Tel) * Ry(bob->Rel.y) * Rx(bob->Rel.x) *
				T(bob->Twr) * Ry(bob->Rwr.y) * Rz(bob->Rwr.z);
		}
		else if (i == 3) {
			// Elbow X derivative
			d_transform = T(bob->Troot) *
				T(bob->Tsh) * Rz(bob->Rsh.z) * Ry(bob->Rsh.y) * Rx(bob->Rsh.x) *
				T(bob->Tel) * Ry(bob->Rel.y) * Rxd(bob->Rel.x) *
				T(bob->Twr) * Ry(bob->Rwr.y) * Rz(bob->Rwr.z);
		}
		else if (i == 4) {
			// Elbow Y derivative
			d_transform = T(bob->Troot) *
				T(bob->Tsh) * Rx(bob->Rsh.x) * Ry(bob->Rsh.y) * Rz(bob->Rsh.z) *
				T(bob->Tel) * Ryd(bob->Rel.y) * Rx(bob->Rel.x) *
				T(bob->Twr) * Ry(bob->Rwr.y) * Rz(bob->Rwr.z);
		}
		else if (i == 5) {
			// Wrist Y derivative
			d_transform = T(bob->Troot) *
				T(bob->Tsh) * Rx(bob->Rsh.x) * Ry(bob->Rsh.y) * Rz(bob->Rsh.z) *
				T(bob->Tel) * Rx(bob->Rel.x) * Ry(bob->Rel.y) *
				T(bob->Twr) * Ryd(bob->Rwr.y) * Rz(bob->Rwr.z);
		}
		else if (i == 6) {
			// Wrist Z derivative
			d_transform = T(bob->Troot) *
				T(bob->Tsh) * Rx(bob->Rsh.x) * Ry(bob->Rsh.y) * Rz(bob->Rsh.z) *
				T(bob->Tel) * Rx(bob->Rel.x) * Ry(bob->Rel.y) *
				T(bob->Twr) * Ry(bob->Rwr.y) * Rzd(bob->Rwr.z);
		}

		Eigen::Vector4d d_phand_world = d_transform * phand;

		J(0, i) = d_phand_world(0);
		J(1, i) = d_phand_world(1);
		J(2, i) = d_phand_world(2);
	}

	//animTcl::OutputMessage("Jacobian is computed");
	return J;
}

void A3System::IKSolver() {
	double threshold = 0.01;
	int i = 0;
	int max = 1000;

	while (true) {
		i++;
		getcurrentP();
		Eigen::Vector3d currentP = { bob->currentP.x, bob->currentP.y, bob->currentP.z };
		Eigen::Vector3d targetP = { bob->targetP.x, bob->targetP.y, bob->targetP.z };
		Eigen::Vector3d error = targetP - currentP;
		
		if (error.norm() < threshold) {

			return;
		}

		Eigen::Vector3d pTargetP = 0.1 * error + currentP;

		IKSolve(targetP);
	}
}

void A3System::IKSolve(Eigen::Vector3d targetP) {
	Eigen::Matrix<double, 3, 7> jacobian;
	Eigen::Matrix<double, 7, 3> jacobianTranspose;
	Eigen::Matrix<double, 3, 3> jjT;

	getcurrentP();
	Eigen::Vector3d currentP = { bob->currentP.x, bob->currentP.y, bob->currentP.z };
	Eigen::Vector3d error = targetP - currentP;

	jacobian = calcJ(); 
	jacobianTranspose = jacobian.transpose(); 
	jjT = jacobian * jacobianTranspose;

	// Damped least squares (DLS)
	double lambda = 0.1;
	Eigen::Matrix3d dampingTerm = lambda * lambda * Eigen::Matrix3d::Identity();
	Eigen::Matrix3d dlsMatrix = jjT + dampingTerm;

	// Pseudo-inverse calculation using DLS
	Eigen::Vector3d beta = dlsMatrix.ldlt().solve(error);
	Eigen::Vector<double, 7> deltaTheta = jacobianTranspose * beta;


	bob->Rsh.x += deltaTheta(0) * 180.0 / PI;
	bob->Rsh.y += deltaTheta(1) * 180.0 / PI;
	bob->Rsh.z += deltaTheta(2) * 180.0 / PI;
	bob->Rel.x += deltaTheta(3) * 180.0 / PI;
	bob->Rel.y += deltaTheta(4) * 180.0 / PI;
	bob->Rwr.y += deltaTheta(5) * 180.0 / PI;
	bob->Rwr.z += deltaTheta(6) * 180.0 / PI;

	bob->Rsh.x = fmod(bob->Rsh.x, 360.0);
	bob->Rsh.y = fmod(bob->Rsh.y, 360.0);
	bob->Rsh.z = fmod(bob->Rsh.z, 360.0);
	bob->Rel.x = fmod(bob->Rel.x, 360.0);
	bob->Rel.y = fmod(bob->Rel.y, 360.0);
	bob->Rwr.y = fmod(bob->Rwr.y, 360.0);
	bob->Rwr.z = fmod(bob->Rwr.z, 360.0);
}


void A3System::drawEllipse(double x, double y) {
	glBegin(GL_LINE_LOOP);
	for (int i = 0; i < 360; i++)
	{
		glVertex2f(cos(i * PI / 180) * x / 2, sin(i * PI / 180) * y / 2);
	}
	glEnd();
}

void A3System::drawBob() {
	glColor3f(0, 1, 0);

	glPushMatrix();
		glTranslated(bob->Troot.x, bob->Troot.y, bob->Troot.z);
		drawEllipse(1.5 * 3, 2.8 * 3);		
		// Head
		glPushMatrix();
			glTranslated(0, 2 * 3, 0);
			drawEllipse(1 * 3, 1 * 3);
		glPopMatrix();

		glPushMatrix(); // shoulder
			glTranslated(bob->Tsh.x, bob->Tsh.y, bob->Tsh.z);
			glRotated(bob->Rsh.z, 0, 0, 1);
			glRotated(bob->Rsh.y, 0, 1, 0);
			glRotated(bob->Rsh.x, 1, 0, 0);
			//animTcl::OutputMessage("Shoulder: %f %f %f", bob->Rsh.x, bob->Rsh.y, bob->Rsh.z);
			glColor3f(1, 0, 0);
			drawEllipse(0.1 * 3, 0.1 * 3); // joint origin
			glPushMatrix();
				glTranslated(1 * 3, 0, 0);
				glColor3f(0, 1, 0);
				drawEllipse(2 * 3, 0.4 * 3);
			glPopMatrix();
			glPushMatrix();
				glTranslated(bob->Tel.x, bob->Tel.y, bob->Tel.z);
				glRotated(bob->Rel.y, 0, 1, 0);
				glRotated(bob->Rel.x, 1, 0, 0);
				//animTcl::OutputMessage("Elbow: %f %f %f", bob->Rel.x, bob->Rel.y, bob->Rel.z);
				glColor3f(1, 0, 0);
				drawEllipse(0.1 * 3, 0.1 * 3); // joint origin
				glPushMatrix();
					glTranslated(1 * 3, 0, 0);
					glColor3f(0, 1, 0);
					drawEllipse(2 * 3, 0.4 * 3);
				glPopMatrix();
				glPushMatrix();
					glTranslated(bob->Twr.x, bob->Twr.y, bob->Twr.z);
					glRotated(bob->Rwr.y, 0, 1, 0);
					glRotated(bob->Rwr.z, 0, 0, 1);
					//animTcl::OutputMessage("Wrist: %f %f %f", bob->Rwr.x, bob->Rwr.y, bob->Rwr.z);
					glColor3f(1, 0, 0);	
					drawEllipse(0.1 * 3, 0.1 * 3); // joint origin
					glPushMatrix();
						glTranslated(0.45*3, 0, 0);
						glColor3f(0, 1, 0);
						drawEllipse(0.9*3, 0.5*3);
					glPopMatrix();
					glPushMatrix();
					glTranslated(bob->phand.x, bob->phand.y, bob->phand.z);
					drawEllipse(0.1 * 3, 0.1 * 3);

					glPopMatrix();
				glPopMatrix();
			glPopMatrix();
		glPopMatrix();

		glPushMatrix();
			glTranslated(-0.4 * 3, -1.3 * 3, 0);
			glPushMatrix();
				glTranslated(0, -1 * 3, 0);
				drawEllipse(0.5 * 3, 2 * 3);
			glPopMatrix();

			glPushMatrix();
				glTranslated(0, -2 * 3, 0);
				glPushMatrix();
					glTranslated(0, -1 * 3, 0);
					drawEllipse(0.5 * 3, 2 * 3);
				glPopMatrix();

				glPushMatrix();
					glTranslated(0, -2 * 3, 0);
					glPushMatrix();
						glTranslated(0, -0.1 * 3, 0);
						drawEllipse(0.6 * 3, 0.2 * 3);
					glPopMatrix();

				glPopMatrix();
			glPopMatrix();
		glPopMatrix();

		glPushMatrix();
		glTranslated(0.4 * 3, -1.3 * 3, 0);
		glPushMatrix();
		glTranslated(0, -1 * 3, 0);
		drawEllipse(0.5 * 3, 2 * 3);
		glPopMatrix();

		glPushMatrix();
		glTranslated(0, -2 * 3, 0);
		glPushMatrix();
		glTranslated(0, -1 * 3, 0);
		drawEllipse(0.5 * 3, 2 * 3);
		glPopMatrix();

		glPushMatrix();
		glTranslated(0, -2 * 3, 0);
		glPushMatrix();
		glTranslated(0, -0.1 * 3, 0);
		drawEllipse(0.6 * 3, 0.2 * 3);
		glPopMatrix();
		glPopMatrix();
		glPopMatrix();
		glPopMatrix();

	glPopMatrix();
}

void A3System::drawBackground() {
	// Wall
	glBegin(GL_QUADS);
	glColor3f(1, 1, 1);
	glVertex3d(18, 18, -0.02);
	glVertex3d(18, -18, -0.02);
	glVertex3d(-18, -18, -0.02);
	glVertex3d(-18, 18, -0.02);
	glEnd();

	// Blackboard
	glBegin(GL_QUADS);
	glColor3f(.2, .2, .2);
	glVertex3d(12, 8, -0.01);
	glVertex3d(12, -8, -0.01);
	glVertex3d(-12, -8, -0.01);
	glVertex3d(-12, 8, -0.01);
	glEnd();

	// Floor
	glBegin(GL_QUADS);
	glColor3f(1, .8, 0);
	glVertex3d(-18, -18, 0);
	glVertex3d(18, -18, 0);
	glVertex3d(18, -18, 18);
	glVertex3d(-18, -18, 18);
	glEnd();
}

void A3System::display(GLenum mode) {
	glDisable(GL_LIGHTING);

	drawBackground();

	drawBob();
}