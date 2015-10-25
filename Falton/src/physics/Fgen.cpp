//
// Created by Kevin Yu on 10/9/15.
//

#include "falton/physics/Fgen.h"

namespace falton {

    Gravity::Gravity(const Vector3 &gravity) : m_gravity(gravity) {};

    void Gravity::update(RigidBody *rigidBody,real) {
        rigidBody->applyForceToCenterOfMass(m_gravity);
    }

    void ForceRegistry::add( RigidBody *rigidBody,ForceGenerator *forceGenerator) {
        ForceRegistry::ForceRegistration fr;
        fr.forceGenerator = forceGenerator;
        fr.rigidBody = rigidBody;

        registry.push_back(fr);
    }

    void ForceRegistry::clear() {
        registry.clear();
    }

    void ForceRegistry::applyForce(real duration) {
        for (Registry::iterator current = registry.begin(); current!=registry.end();current++) {
            ForceGenerator *forceGenerator = current->forceGenerator;
            RigidBody *rigidBody = current->rigidBody;
            forceGenerator->update(rigidBody,duration);
        }
    }

    Aero::Aero(const Matrix3x3 &tensor, const Vector3 &position, Vector3 *windspeed) {
        this->tensor = tensor;
        this->position = position;
        this->windspeed = windspeed;
    }

    void Aero::update(RigidBody *body, real ) {
        addForceFromTensor(body,tensor);
    }

    void Aero::addForceFromTensor(RigidBody *rigidBody, const Matrix3x3 &tensor) {

        Vector3 velocity = rigidBody->getLinearVelocity() + *windspeed;

        Vector3 bodyVel = rigidBody->worldToLocalDirection(velocity);

        Vector3 bodyForce = tensor.transform(bodyVel);
        Vector3 worldForce = rigidBody->localToWorldDirection(bodyForce);

        Vector3 worldPosition = rigidBody->localToWorldPosition(position);

        rigidBody->applyForce(worldForce,worldPosition);

    }

    AeroControl::AeroControl(const Matrix3x3 &baseTensor, const Matrix3x3 &minTensor, const Matrix3x3 &maxTensor, const Vector3 &position,
                             Vector3 *windspeed) : Aero(baseTensor, position, windspeed){
        this->minTensor = minTensor;
        this->maxTensor = maxTensor;
        controlSetting = 0.0f;
    }

    Matrix3x3 AeroControl::getTensor() {
        if (controlSetting<=-1.0f) return minTensor;
        else if (controlSetting>=1.0f) return maxTensor;
        else if (controlSetting<0) return Matrix3x3::linearInterpolate(minTensor,tensor,controlSetting + 1.0f);
        else if (controlSetting>0) return Matrix3x3::linearInterpolate(tensor,maxTensor,controlSetting);
        return tensor;
    }

    void AeroControl::update(RigidBody *body,real) {
        addForceFromTensor(body, getTensor());
    }

    void AeroControl::setControl(real controlSetting) {
        this->controlSetting = controlSetting;
    }
}