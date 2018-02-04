#pragma once
#include "falton/physics.h"

namespace Phyvis {
	namespace Entity {

		enum BodyType {
			BODY_TYPE_STATIC = 0,
			BODY_TYPE_KINEMATIC,
			BODY_TYPE_DYNAMIC,
			BODY_TYPE_TOTAL
		};

		static const char* BODY_TYPE_LABELS[3] = {
			"Static",
			"Kinematic",
			"Dynamic"
		};

		enum ShapeType {
			SHAPE_TYPE_CIRCLE,
			SHAPE_TYPE_SQUARE,
			SHAPE_TYPE_POLYGON,
			SHAPE_TYPE_TOTAL
		};

		static const char* SHAPE_TYPE_LABELS[SHAPE_TYPE_TOTAL] = {
			"Circle",
			"Square",
			"Polygon",
		};

		enum JointType {
			JOINT_TYPE_DISTANCE,
			JOINT_TYPE_DYNAMO,
			JOINT_TYPE_HINGE,
			JOINT_TYPE_PISTON,
			JOINT_TYPE_SPRING,
			JOINT_TYPE_TOTAL
		};

		static const char* JOINT_TYPE_LABELS[JOINT_TYPE_TOTAL] = {
			"Distance",
			"Dynamo",
			"Hinge",
			"Piston",
			"Spring"
		};

		struct Body;
		struct Collider;
		struct Joint;

		struct Body {
			int bodyID;
			char alias[512];
			BodyType bodyType;
			real mass;
			
			ftTransform transform;
			ftVector2 velocity;
			
			bool isCustomCenterOfMass;
			ftVector2 centerOfMass;

			
			Collider* collider;
			Joint* joint;
			ftBody* faltonBody;
		};

		struct Collider {
			int colliderID;
			char alias[512];
			ftTransform transform;
			real friction = 0.2;
			real restitution = 0.2;
			uint32 group = 0;
			uint32 category = 0xFFFF;
			uint32 mask = 0xFFFF;
			int spatialID;

			ftShape* shape;
			Body* body;
			Collider* prev;
			Collider* next;
		};

		struct Joint {
			int jointID;
			char alias[512];
			JointType jointType;
			Body* body[2];
			Joint* prev[2];
			Joint* next[2];
			union {
				struct Hinge {
					ftVector2 anchorPoint;
					real torqueFriction;
					bool enableLimit;
					real lowerLimit;
					real upperLimit;
				} hinge;
				struct Distance {
					ftVector2 localAnchorA;
					ftVector2 localAnchorB;
					bool isCustomDistance;
					real distance;
				} distance;
				struct Dynamo {
					real targetRate;
					real maxTorque;
				} dynamo;
				struct Piston{
					ftVector2 axis;
					ftVector2 localAnchorA;
					ftVector2 localAnchorB;
				} piston;
				struct Spring{
					ftVector2 localAnchorA;
					ftVector2 localAnchorB;
					real stiffness;
					real restLength;
				} spring;
			};
		};

		struct Database {
			ftIDBuffer<Body> bodies;
			ftIDBuffer<Collider> colliders;
			ftIDBuffer<Joint> joints;
			ftHierarchicalGrid spatialIndex;
		};

		typedef void(*BodyIterFunc) (Body* body, void* data);
		typedef void(*ColliderIterFunc) (Collider* collider, void* data);

		void Init(Database* database);
		void Cleanup(Database* database);
		void Save(Database* database, const char* filepath);
		void Load(Database* database, const char* filepath);


		int AddBody(Database* database, ftTransform transform);
		void DeleteBody(Database* database, int bodyID);
		void DeleteBody(Database* database, Body* body);
		Body* GetBody(Database* database, int bodyID);
		int DeepCopyBody(Database* database, int bodyID, const ftTransform& transform);
		void TransformBody(Database* database, int bodyID, const ftTransform& transform);
		template <typename T>
		void ForEveryBody(Database* database, T func);
		template <typename T>
		void ForEveryJointInBody(Database* database, T func);

		int AddCollider(Database* database, int bodyID, ftTransform transform, ftShape* shape);
		Collider* GetCollider(Database* database, int colliderID);
		template<typename T>
		void ForEveryCollider(Database* database, T func);

		int CreateDistanceJoint(Database* database, int bodyIDA, int bodyIDB, 
			ftVector2 localAnchorA, ftVector2 localAnchorB);
		int CreateDynamoJoint(Database* database, int bodyIDA, int bodyIDB,
			real targetRate, real maxTorque);
		int CreateHingeJoint(Database* database, int bodyIDA, int bodyIDB,
			ftVector2 anchorPoint);
		int CreateSpringJoint(Database* database, int bodyIDA, int bodyIDB,
			ftVector2 localAnchorA, ftVector2 localAnchorB);
		int CreatePistonJoint(Database* database, int bodyIDA, int bodyIDB,
			ftVector2 localAnchorA, ftVector2 localAnchorB, ftVector2 axis);
		void DeleteJoint(Database* database, int jointID);
		void DeleteJoint(Database* database, Joint* joint);
		Joint* GetJoint(Database* database, int jointID);
		template<typename T>
		void ForEveryJoint(Database* database, T func);

		bool PointQueryJoint(Database* database, ftVector2 point, float radius, int* jointID);

		bool PointQueryBody(Database* database, ftVector2 point, float radius, int* bodyID);
		void PointQueryBodies(Database* database, ftVector2 point, float radius, ftVectorArray<int>* bodies);
		bool PointQueryVertexes(Database* database, ftVector2 point, float radius, ftVector2* vertex);
		void RegionQueryBodies(Database* database, ftVector2 corner1, ftVector2 corner2, ftVectorArray<int>* bodies);
			
		bool PointQueryCollider(Database* database, ftVector2 point, int* colliderInRegion);
		void PointQueryCollider(Database* database, ftVector2 point, ftVectorArray<Collider*>* colliders);
		void RegionQueryCollider(Database* database, ftVector2 corner1, ftVector2 corner2, ftVectorArray<int>* collidersInRegion);
	
		template <typename T>
		void ForEveryBody(Database* database, T func) {
			for (int i = 0; i < database->bodies.size; ++i) {
				func(&database->bodies.buffer[i]);
			}
		}

		template <typename T>
		void ForEveryCollider(Database* database, T func) {
			for (int i = 0; i < database->colliders.size; ++i) {
				func(&database->colliders.buffer[i]);
			}
		}

		template<typename T>
		void ForEveryJoint(Database* database, T func) {
			for (int i = 0; i < database->joints.size; ++i) {
				func(&database->joints.buffer[i]);
			}
		}

		template<typename T>
		void ForEveryColliderInBody(Database* database, int bodyID, T func) {
			Entity::Body* body = Entity::GetBody(database, bodyID);
			Entity::Collider* collider = body->collider;
			while(collider != nullptr) {
				func(collider);
				collider = collider->next;
			}
		}

		template<typename T>
		void ForEveryJointInBody(Database* database, int bodyID, T func) {
			Entity::Body* body = Entity::GetBody(database, bodyID);
			Entity::Joint* joint = body->joint;
			while (joint != nullptr) {
				func(joint);
				if (joint->body[0] == body) {
					joint = joint->next[0];
				} else {
					joint = joint->next[1];
				}
			}
		}
	}
}