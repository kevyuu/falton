//
// Created by Kevin Yu on 10/9/15.
//

#ifndef TUTORIAL_FGEN_H
#define TUTORIAL_FGEN_H

#include "falton/math.h"
#include "falton/physics/RigidBody.h"
#include "vector"

namespace falton {
    class ForceGenerator {
    public:
        virtual void update(RigidBody* rigidBody, real duration) = 0;
    };

    class Gravity : public ForceGenerator {

    public:

        Gravity(const Vector3 &gravity);
        virtual void update(RigidBody* rigidBody, real duration);

    private:
        Vector3 m_gravity;
    };

    class ForceRegistry {

    private:

        struct ForceRegistration {
            ForceGenerator *forceGenerator;
            RigidBody *rigidBody;
        };

        typedef std::vector<ForceRegistration> Registry;
        Registry registry;

    public:

        void add(RigidBody *rigidBody,ForceGenerator *forceGenerator);

        void clear();

        void applyForce(real duration);
    };

    class Aero : public ForceGenerator{
    protected:

        Matrix3x3 tensor;
        Vector3 position;
        Vector3 *windspeed;

    protected:

        void addForceFromTensor(RigidBody *rigidBody, const Matrix3x3 &tensor);
    public:

        Aero(const Matrix3x3 &tensor, const Vector3 &position, Vector3 *windspeed);
        virtual void update(RigidBody *body,real duration);

    };

    class AeroControl : public Aero {
    private:

        Matrix3x3 minTensor;
        Matrix3x3 maxTensor;
        real controlSetting;

    public:
        Matrix3x3 getTensor();
        AeroControl(const Matrix3x3 &baseTensor, const Matrix3x3 &minTensor, const Matrix3x3 &maxTensor,
                const Vector3& position, Vector3 *windspeed);
        void setControl(real controlSetting);

        virtual void update(RigidBody *body,real duration);
    };
}

#endif //TUTORIAL_FGEN_H
