//
// Created by Kevin Yu on 9/26/15.
//

#include "falton/physics/RigidBody.h"
#include <iostream>
#include <limits>

typedef std::numeric_limits< double > dbl;
using namespace std;

namespace falton {

    void RigidBody::setMass(real mass) {
        if (mass != 0) {
            m_inverseMass = ((real) 1.0)/ mass;
        } else {
            m_inverseMass = INFINITY;
        }
    }

    void RigidBody::setPosition(const Vector3 &position) {
        m_worldPosition = position;
    }

    void RigidBody::setOrientation(const Quaternion &orientation) {
        m_orientation = orientation;
    }

    void RigidBody::setLinearVelocity(const Vector3 &linearVelocity) {
        m_linearVelocity = linearVelocity;
    }

    void RigidBody::setAngularVelocity(const Vector3 &angularVelocity) {
        m_angularVelocity = angularVelocity;
    }

    void RigidBody::setLinearDamping(real linearDamping) {
        m_linearDamping = linearDamping;
    }

    void RigidBody::setAngularDamping(real angularDamping) {
        m_angularDamping = angularDamping;
    }

    void RigidBody::setInertiaTensor(const Matrix3x3 &inertiaTensor) {
        m_localInverseInertiaTensor = inertiaTensor.getInverse();
        Matrix3x3 rotMatrix(m_orientation);
        m_worldlInverseInertiaTensor = rotMatrix * m_localInverseInertiaTensor * rotMatrix.getTranspose();
    }

    Vector3 RigidBody::getLinearVelocity() const {
        return m_linearVelocity;
    }

    Vector3 RigidBody::getAngularVelocity() const {
        return m_angularVelocity;
    }

    Vector3 RigidBody::getPosition() const {
        return m_worldPosition;
    }

    Quaternion RigidBody::getOrientation() const {
        return m_orientation;
    }

    real RigidBody::getMass() const {
        return 1 / m_inverseMass;
    }

    real RigidBody::getInverseMass() const {
        return m_inverseMass;
    }

    void RigidBody::applyForce(const Vector3 &force, const Vector3 &position) {
        m_forceAccumulator += force;
        m_torqueAcumulator += (position - m_worldPosition).cross(force);
    }

    void RigidBody::applyForceToCenterOfMass(const Vector3 &force) {
        m_forceAccumulator += force;
    }

    Vector3 RigidBody::localToWorldPosition(const Vector3 &position) const {
        return  m_orientation.transform(position) + m_lastPosition;
    }

    Vector3 RigidBody::wordToLocalPosition(const Vector3 &position) const {
        return m_orientation.transformInverse(position - m_worldPosition);
    }

    Vector3 RigidBody::localToWorldDirection(const Vector3 &vector) const {
        return m_orientation.transform(vector);
    }

    Vector3 RigidBody::worldToLocalDirection(const Vector3 &vector) const {
        return m_orientation.transformInverse(vector);
    }

    void RigidBody::integrate(real duration) {

        Vector3 linearAcceleration = m_forceAccumulator * m_inverseMass;
        Vector3 angularAcceleration = m_worldlInverseInertiaTensor * m_torqueAcumulator;

        m_linearVelocity += (linearAcceleration * duration);
        m_linearVelocity *= real_pow(m_linearDamping, duration);

        m_angularVelocity += (angularAcceleration * duration);
        m_angularVelocity *= real_pow(m_angularDamping, duration);

        m_worldPosition += (m_linearVelocity * duration);

        Quaternion rotation(m_angularVelocity * duration);
        m_orientation = rotation * m_orientation;
        m_orientation.normalise();

        clear_accumulator();
        Matrix3x3 rotMatrix(m_orientation);
        m_worldlInverseInertiaTensor = rotMatrix * m_localInverseInertiaTensor * rotMatrix.getTranspose();
        m_lastPosition = m_worldPosition;

    }

    void RigidBody::clear_accumulator() {
        m_forceAccumulator.zero();
        m_torqueAcumulator.zero();
    }

}