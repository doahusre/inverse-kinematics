#include "A3Simulator.h"

A3Simulator::A3Simulator(const std::string& name, BaseSystem* target) :
    BaseSimulator(name),
    m_object(target),
    hermite_t(0),
    prev_t(0),
    transition(true)
{
    hermite = new Hermite("hermite");
    GlobalResourceManager::use()->addSystem(hermite, true);
}

int A3Simulator::init(double time) {
    return TCL_OK;
}

int A3Simulator::step(double time) {
    double delta_t = time - prev_t;

    if (hermite_t > 0.990) {
        hermite_t = 0;
        transition = true;
    }

    if (hermite->size() == 0) {
        animTcl::OutputMessage("No spline file loaded.");
        return TCL_ERROR;
    }

    glm::vec3 currentP, targetP, error;

    double p[3];
    m_object->getState(p);
    currentP.x = p[0];
    currentP.y = p[1];
    currentP.z = p[2];
    //animTcl::OutputMessage("current position: %f %f %f", currentP.x, currentP.y, currentP.z);

    VectorObj splineStart = hermite->getIntermediatePoint(0); // Start of spline
    glm::vec3 splineStartP(splineStart[0], splineStart[1], splineStart[2]);

    if (transition) {
        error = splineStartP - currentP;
        //animTcl::OutputMessage("Transitioning to spline start, Error Length: %f", glm::length(error));
        if (glm::length(error) < 0.05) {
            transition = false;
        }
        else {
            glm::vec3 incrementStep = glm::normalize(error) * (float)0.1;
            glm::vec3 nextTarget = currentP + incrementStep;
            //animTcl::OutputMessage("spine start: %f %f %f", splineStartP.x, splineStartP.y, splineStartP.z);
            //animTcl::OutputMessage("Increment Step size %f", glm::length(incrementStep));
            //animTcl::OutputMessage("next target: %f %f %f", nextTarget.x, nextTarget.y, nextTarget.z);

            double newTarget[3] = { nextTarget.x, nextTarget.y, nextTarget.z };
            m_object->setState(newTarget);

            prev_t = time;
            return TCL_OK; 
        }
    }

    VectorObj target = hermite->getIntermediatePoint(hermite_t);
    targetP.x = target[0];
    targetP.y = target[1];
    targetP.z = target[2];

    error = targetP - currentP;

    if (glm::length(error) < 0.15) {
        hermite_t += 0.00005;
    }

    double newTarget[3] = { targetP.x, targetP.y, targetP.z };
    m_object->setState(newTarget);

    prev_t = time;
    return TCL_OK;
}


int A3Simulator::command(int argc, myCONST_SPEC char** argv) {
    if (argc < 1) {
        animTcl::OutputMessage("Available commands: ");
        return TCL_ERROR;
    }
    if (strcmp(argv[0], "read") == 0) {
        if (argc == 2) {
            hermite->loadFromFile2D(argv[1]);
            animTcl::OutputMessage("Spline loaded from file.");

            hermite_t = 0;
            transition = true;

            dynamic_cast<A3System*>(m_object)->setRest();
        }
        else {
            animTcl::OutputMessage("Usage: simulator A3Simulator read <file>");
            return TCL_ERROR;
        }
    }

    return TCL_OK;
}
