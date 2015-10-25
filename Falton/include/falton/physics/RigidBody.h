//
// Created by Kevin Yu on 9/26/15.
//

#ifndef FALTON_RIGIDBODY_H
#define FALTON_RIGIDBODY_H

#include "falton/math.h"

namespace falton {
    class RigidBody {

    public :
        void setMass(real mass);
        void setPosition(const Vector3 &position);
        void setOrientation(const Quaternion &orientation);
        void setLinearVelocity(const Vector3 &linearVelocity);
        void setAngularVelocity(const Vector3 &angularVelocity);
        void setLinearDamping(real linearDamping);
        void setAngularDamping(real angularDamping);
        void setInertiaTensor(const Matrix3x3 &inertiaTensor);

        Vector3 getLinearVelocity() const;
        Vector3 getAngularVelocity() const;
        Vector3 getPosition() const;
        Quaternion getOrientation() const;
        real getMass() const;
        real getInverseMass() const;

        Vector3 localToWorldPosition(const Vector3 &position) const;
        Vector3 wordToLocalPosition(const Vector3 &position) const;
        Vector3 localToWorldDirection(const Vector3 &vector) const;
        Vector3 worldToLocalDirection(const Vector3 &vector) const;

        void applyForce(const Vector3 &force, const Vector3 &position);
        void applyForceToCenterOfMass(const Vector3 &force);
        void integrate(real duration);
        void clear_accumulator();

        void calculateDerivedData();

    private:
        real m_inverseMass;

        Vector3 m_worldPosition;

        Vector3 m_lastPosition;

        Quaternion m_orientation;

        Vector3 m_linearVelocity;
        Vector3 m_angularVelocity;

        Vector3 m_forceAccumulator;
        Vector3 m_torqueAcumulator;

        Matrix3x3 m_localInverseInertiaTensor;
        Matrix3x3 m_worldlInverseInertiaTensor;

        real m_linearDamping;
        real m_angularDamping;

    };
}



#endif //FALTON_RIGIDBODY_H
