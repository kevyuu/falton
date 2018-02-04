#include "phyvis_entity.h"

namespace Phyvis {
	namespace Entity {

		static int GetBodyOrderInJoint(Joint* joint, Body* body) {
			if (joint->body[0] == body) {
				return 0;
			}
			else {
				return 1;
			}
		}

		static int CreateJoint(Database* database, int bodyIDA, int bodyIDB) {
			int jointID = database->joints.alloc();
			Joint* oldStart = database->joints.buffer;
			Joint* joint = database->joints.getAddress(jointID);
			Joint* newStart = database->joints.buffer;
			if (newStart != oldStart) {
				for (int i = 0; i < database->bodies.size; ++i) {
					Joint* oldAddr = database->bodies.buffer[i].joint;
					database->bodies.buffer[i].joint = newStart + (oldAddr - oldStart);
				}
				for (int i = 0; i < database->joints.size; ++i) {
					for (int j = 0; j < 2; ++j) {
						Joint* oldPrevAddr = database->joints.buffer[i].prev[j];
						database->joints.buffer[i].prev[j] = newStart + (oldPrevAddr - oldStart);
						Joint* oldNextAddr = database->joints.buffer[i].next[j];
						database->joints.buffer[i].next[j] = newStart + (oldNextAddr - oldStart);
					}
				}
			}

			joint->body[0] = GetBody(database, bodyIDA);
			joint->next[0] = joint->body[0]->joint;
			if (joint->body[0]->joint != nullptr) {
				int bodyOrder = GetBodyOrderInJoint(joint->body[0]->joint, joint->body[0]);
				joint->body[0]->joint->prev[bodyOrder] = joint;
			}
			joint->body[0]->joint = joint;
			joint->prev[0] = nullptr;

			joint->body[1] = GetBody(database, bodyIDB);
			joint->next[1] = joint->body[1]->joint;
			if (joint->body[1]->joint != nullptr) {
				int bodyOrder = GetBodyOrderInJoint(joint->body[1]->joint, joint->body[1]);
				joint->body[1]->joint->prev[bodyOrder] = joint;
			}
			joint->body[1]->joint = joint;
			joint->prev[1] = nullptr;

			joint->jointID = jointID;

			sprintf(joint->alias, "Joint%d", jointID);

			return jointID;
		}

		void Init(Database* database) {
			database->bodies.init(128);
			database->colliders.init(128);
			database->joints.init(128);
			database->spatialIndex.setConfiguration(ftHierarchicalGrid::ftConfig());
			database->spatialIndex.init();
		}

		void Cleanup(Database* database) {
			database->bodies.cleanup();
			database->colliders.cleanup();
			database->joints.cleanup();
			database->spatialIndex.shutdown();
		}

		void Save(Database* database, const char* filepath) {
			FILE* file = fopen(filepath, "w");
			if (file == NULL) return;
			fprintf(file, "%d %d %d\n", database->bodies.size, 
				database->colliders.size, 
				database->joints.size);
			for (int i = 0; i < database->bodies.size; ++i) {
				Body* body = &database->bodies.buffer[i];
				fprintf(file, "%s\n", body->alias);
				fprintf(file, "%d\n", body->bodyType);
				fprintf(file, "%f\n", body->mass);
				fprintf(file, "%f %f\n", body->transform.center.x, body->transform.center.y);
				fprintf(file, "%f\n", body->transform.rotation.angle);
				fprintf(file, "%f %f\n", body->velocity.x, body->velocity.y);
				fprintf(file, "%d\n", body->isCustomCenterOfMass);
				fprintf(file, "%f %f\n", body->centerOfMass.x, body->centerOfMass.y);

				if (body->collider != nullptr) 
					fprintf(file, "%d\n", (int)(body->collider - database->colliders.buffer));
				else fprintf(file, "%d\n", -1);
			}

			for (int i = 0; i < database->colliders.size; ++i) {
				Collider* collider = &database->colliders.buffer[i];
				ftShape* shape = collider->shape;
				fprintf(file, "%s\n", collider->alias);
				fprintf(file, "%lld\n", collider->body - database->bodies.buffer);
				fprintf(file, "%f %f", collider->transform.center.x, collider->transform.center.y);
				fprintf(file, "%f\n", collider->transform.rotation.angle);
				fprintf(file, "%f\n", collider->friction);
				fprintf(file, "%f\n", collider->restitution);
				fprintf(file, "%d\n", collider->group);
				fprintf(file, "%d\n", collider->category);
				fprintf(file, "%d\n", collider->mask);
				fprintf(file, "%d\n", shape->shapeType);
				
				if (shape->shapeType == SHAPE_CIRCLE) {
					ftCircle* circle = (ftCircle*)shape;
					fprintf(file, "%f\n", circle->radius);
				}
				else if (shape->shapeType == SHAPE_POLYGON) {
					ftPolygon* polygon = (ftPolygon*)shape;
					fprintf(file, "%d\n", polygon->numVertex);
					for (int j = 0; j < polygon->numVertex; ++j) {
						fprintf(file, "%f %f\n", polygon->vertices[j].x, polygon->vertices[j].y);
					}
				}

				if (collider->prev != nullptr)
					fprintf(file, "%d\n", (int)(collider->prev - database->colliders.buffer));
				else fprintf(file, "%d\n", -1);
				if (collider->next != nullptr)
					fprintf(file, "%d\n", (int) (collider->next - database->colliders.buffer));
				else fprintf(file, "%d\n", -1);
			}

			for (int i = 0; i < database->joints.size; ++i) {
				Joint* joint = &database->joints.buffer[i];
				fprintf(file, "%s\n", joint->alias);
				fprintf(file, "%d\n", joint->jointType);
				fprintf(file, "%d %d\n", (int)(joint->body[0] - database->bodies.buffer),
					(int)(joint->body[1] - database->bodies.buffer));
				switch (joint->jointType) {
				case JOINT_TYPE_HINGE:
					fprintf(file, "%f %f\n", joint->hinge.anchorPoint.x, joint->hinge.anchorPoint.y);
					fprintf(file, "%f\n", joint->hinge.torqueFriction);
					fprintf(file, "%d\n", joint->hinge.enableLimit);
					fprintf(file, "%f\n", joint->hinge.upperLimit);
					fprintf(file, "%f\n", joint->hinge.lowerLimit);
					break;
				case JOINT_TYPE_DISTANCE:
					fprintf(file, "%f %f\n", joint->distance.localAnchorA.x, joint->distance.localAnchorA.y);
					fprintf(file, "%f %f\n", joint->distance.localAnchorB.x, joint->distance.localAnchorB.y);
					fprintf(file, "%d\n", joint->distance.isCustomDistance);
					fprintf(file, "%f\n", joint->distance.distance);
					break;
				case JOINT_TYPE_DYNAMO: 
					fprintf(file, "%f %f\n", joint->dynamo.targetRate,
						joint->dynamo.maxTorque);
					break;
				case JOINT_TYPE_PISTON:
					fprintf(file, "%f %f\n", joint->piston.axis.x, joint->piston.axis.y);
					fprintf(file, "%f %f\n", joint->piston.localAnchorA.x, joint->piston.localAnchorA.y);
					fprintf(file, "%f %f\n", joint->piston.localAnchorB.x, joint->piston.localAnchorB.y);
					break;
				case JOINT_TYPE_SPRING:
					fprintf(file, "%f %f\n", joint->spring.localAnchorA.x, joint->spring.localAnchorA.y);
					fprintf(file, "%f %f\n", joint->spring.localAnchorB.x, joint->spring.localAnchorB.y);
					fprintf(file, "%f\n", joint->spring.stiffness);
					fprintf(file, "%f\n", joint->spring.restLength);
					break;
				}
			}
			fclose(file);
		}

		void Load(Database* database, const char* filepath) {
			FILE* file = fopen(filepath, "r");
			if (file == NULL) return;
			int bodyCount, colliderCount, jointCount;
			fscanf(file, "%d %d %d", &bodyCount, &colliderCount,& jointCount);

			database->bodies.removeAll();
			database->bodies.reserve(bodyCount);

			database->colliders.removeAll();
			database->colliders.reserve(colliderCount);

			database->joints.removeAll();
			database->joints.reserve(jointCount);

			std::cout << "Collider Count : " << colliderCount << std::endl;

			for (int i = 0; i < bodyCount; ++i) {
				int bodyID = database->bodies.alloc();
				Body* body = database->bodies.getAddress(bodyID);
				body->bodyID = bodyID;
				fscanf(file, "%s", body->alias);
				fscanf(file, "%d", &body->bodyType);
				fscanf(file, "%f", &body->mass);
				fscanf(file, "%f %f", &body->transform.center.x, &body->transform.center.y);
				real angle;
				fscanf(file, "%f", &angle);
				body->transform.rotation = ftRotation::Angle(angle);
				fscanf(file, "%f %f", &body->velocity.x, &body->velocity.y);
				fscanf(file, "%d", &body->isCustomCenterOfMass);
				fscanf(file, "%f %f", &body->centerOfMass.x, &body->centerOfMass.y);

				int colliderOffset;
				fscanf(file, "%d", &colliderOffset);
				if (colliderOffset < 0) body->collider = nullptr;
				else body->collider = database->colliders.buffer + colliderOffset;
				body->joint = nullptr;
			}

			for (int i = 0; i < colliderCount; ++i) {
				int colliderID = database->colliders.alloc();
				Collider* collider = database->colliders.getAddress(colliderID);
				collider->colliderID = colliderID;

				fscanf(file, "%s", collider->alias);

				int bodyOffset;
				fscanf(file, "%d", &bodyOffset);
				Body* body = database->bodies.buffer + bodyOffset;
				collider->body = body;
				body->collider = collider;

				sprintf(collider->alias, "Collider%d", i);
				fscanf(file, "%f %f", &collider->transform.center.x, &collider->transform.center.y); \
					real angle;
				fscanf(file, "%f", &angle);
				collider->transform.rotation = ftRotation::Angle(angle);
				fscanf(file, "%f", &collider->friction);
				fscanf(file, "%f", &collider->restitution);
				fscanf(file, "%d", &collider->group);
				fscanf(file, "%d", &collider->category);
				fscanf(file, "%d", &collider->mask);

				int shapeType;
				fscanf(file, "%d", &shapeType);
				if (shapeType == SHAPE_CIRCLE) {
					real radius;
					fscanf(file, "%f", &radius);
					collider->shape = ftCircle::create(radius);
				}
				else {
					ftVector2 vertexes[128];
					int vertexCount;
					fscanf(file, "%d", &vertexCount);
					for (int j = 0; j < vertexCount; ++j) {
						fscanf(file, "%f %f", &(vertexes[j]).x, &(vertexes[j]).y);
					}
					collider->shape = ftPolygon::create(vertexCount, vertexes);
				}
				collider->spatialID = database->spatialIndex.addShape(collider->shape,
					body->transform * collider->transform,
					(void*)colliderID);

				int prevColliderOffset, nextColliderOffset;
				fscanf(file, "%d", &prevColliderOffset);
				fscanf(file, "%d", &nextColliderOffset);
				if (prevColliderOffset < 0) collider->prev = nullptr;
				else collider->prev = database->colliders.buffer + prevColliderOffset;
				if (nextColliderOffset < 0) collider->next = nullptr;
				else collider->next = database->colliders.buffer + nextColliderOffset;
			}

			for (int i = 0; i < jointCount; ++i) {
				char jointAlias[512];
				fscanf(file,"%s", jointAlias);
				JointType jointType;
				fscanf(file, "%d", &jointType);
				int bodyOffset[2];
				fscanf(file, "%d %d", &bodyOffset[0], &bodyOffset[1]);
				Body* body[2] = {
					&database->bodies.buffer[bodyOffset[0]],
					&database->bodies.buffer[bodyOffset[1]]
				};
				int jointID = CreateJoint(database, body[0]->bodyID, body[1]->bodyID);
				Joint* joint = GetJoint(database, jointID);
				strcpy(joint->alias, jointAlias);
				joint->jointType = jointType;
				switch (jointType) {
				case JOINT_TYPE_HINGE:
					fscanf(file, "%f %f", &joint->hinge.anchorPoint.x, &joint->hinge.anchorPoint.y);
					fscanf(file, "%f", &joint->hinge.torqueFriction);
					fscanf(file, "%d", &joint->hinge.enableLimit);
					fscanf(file, "%f", &joint->hinge.upperLimit);
					fscanf(file, "%f", &joint->hinge.lowerLimit);
					break;
				case JOINT_TYPE_DISTANCE:
					fscanf(file, "%f %f", &joint->distance.localAnchorA.x, &joint->distance.localAnchorA.y);
					fscanf(file, "%f %f", &joint->distance.localAnchorB.x, &joint->distance.localAnchorB.y);
					fscanf(file, "%d", &joint->distance.isCustomDistance);
					fscanf(file, "%f", &joint->distance.distance);
					break;
				case JOINT_TYPE_DYNAMO:
					fscanf(file, "%f %f", &joint->dynamo.targetRate,
						&joint->dynamo.maxTorque);
					break;
				case JOINT_TYPE_PISTON:
					fscanf(file, "%f %f", &joint->piston.axis.x, &joint->piston.axis.y);
					fscanf(file, "%f %f", &joint->piston.localAnchorA.x, &joint->piston.localAnchorA.y);
					fscanf(file, "%f %f", &joint->piston.localAnchorB.x, &joint->piston.localAnchorB.y);
					break;
				case JOINT_TYPE_SPRING:
					fscanf(file, "%f %f\n", &joint->spring.localAnchorA.x, &joint->spring.localAnchorA.y);
					fscanf(file, "%f %f\n", &joint->spring.localAnchorB.x, &joint->spring.localAnchorB.y);
					fscanf(file, "%f\n", &joint->spring.stiffness);
					fscanf(file, "%f\n", &joint->spring.restLength);
					break;
				}


			}
			fclose(file);
		}

		int AddBody(Database* database, ftTransform transform) {
			Body* oldStart = database->bodies.buffer;
			int bodyID = database->bodies.alloc();
			Body* newStart = database->bodies.buffer;
			if (newStart - oldStart != 0) {
				for (int i = 0; i < database->colliders.size; ++i) {
					Body* initAddr = database->colliders.buffer[i].body;
					database->colliders.buffer[i].body = newStart + (initAddr - oldStart);
				}
				for (int i = 0; i < database->joints.size; ++i) {
					for (int j = 0; j < 2; ++j) {
						Body* initAddr = database->joints.buffer[i].body[j];
						database->joints.buffer[i].body[j] = newStart + (initAddr - oldStart);
					}
					
				}
			}

			Body* body = database->bodies.getAddress(bodyID);
			body->bodyID = bodyID;
			body->transform = transform;
			body->velocity = { 0, 0 };
			sprintf(body->alias, "Body%d", bodyID);
			body->isCustomCenterOfMass = false;
			body->centerOfMass = {0, 0};
			body->collider = nullptr;
			body->joint = nullptr;
			return bodyID;
		}

		int DeepCopyBody(Database* database, int bodyID, const ftTransform& transform) {
			int newBodyID = AddBody(database, transform);
			Body* refBody = GetBody(database, bodyID);
			Body* newBody = GetBody(database, newBodyID);

			newBody->bodyType = refBody->bodyType;
			newBody->mass = refBody->mass;
			newBody->velocity = refBody->velocity;
			newBody->collider = nullptr;
			newBody->joint = nullptr;
			newBody->faltonBody = nullptr;

			Collider* refCollider = refBody->collider;
			while (refCollider != nullptr) {
				ftShape* refShape = refCollider->shape;
				ftShape* newShape;
				switch (refShape->shapeType) {
				case SHAPE_CIRCLE: {
					ftCircle * refCircle = (ftCircle*)refShape;
					newShape = ftCircle::create(refCircle->radius);
					break;
				}
				case SHAPE_POLYGON: {
					ftPolygon * refPolygon = (ftPolygon*)refShape;
					newShape = ftPolygon::create(refPolygon->numVertex, refPolygon->vertices);
					break;
				}
				}
				int newColliderID = AddCollider(database, 
					newBodyID, 
					refCollider->transform, 
					newShape);
				Collider* newCollider = GetCollider(database, newColliderID);
				newCollider->group = refCollider->group;
				newCollider->category = refCollider->category;
				newCollider->mask = refCollider->mask;
				newCollider->restitution = refCollider->restitution;
				newCollider->friction = refCollider->friction;

				refCollider = refCollider->next;
			}
			return newBodyID;
		}

		void TransformBody(Database* database, int bodyID, const ftTransform& transform) {
			Body* body = database->bodies.getAddress(bodyID);
			body->transform = transform;
			Collider* collider = body->collider;
			if (collider == NULL) return;
			database->spatialIndex.moveShape(collider->spatialID,
				collider->shape,
				body->transform * collider->transform);
		}

		void DeleteBody(Database* database, int bodyID) {
			Body* body = database->bodies.getAddress(bodyID);
			DeleteBody(database, body);
		}

		void DeleteBody(Database* database, Body* body) {
			Collider* collider = body->collider;
			
			while (collider != nullptr) {
				Collider* next = collider->next;
				database->spatialIndex.removeShape(collider->spatialID);
				delete collider->shape;
				database->colliders.remove(collider);
				collider->body->collider = collider;
				if (collider->prev != nullptr)
					collider->prev->next = collider;
				if (collider->next != nullptr)
					collider->next->prev = collider;
				collider = next;
			}

			std::cout << "Delete Joint" << std::endl;
			while (body->joint != nullptr) {
				DeleteJoint(database, body->joint);
			}
			
			database->bodies.remove(body);

			collider = body->collider;
			while (collider != nullptr) {
				collider->body = body;
				collider = collider->next;
			}

			Entity::Body* prevAddr = database->bodies.buffer + database->bodies.size + 1;
			Joint* joint = body->joint;
			while (joint != nullptr) {
				int order = GetBodyOrderInJoint(joint,prevAddr);
				joint->body[order] = body;
				joint = joint->next[order];
			}
		}

		Body* GetBody(Database* database, int bodyID) {
			return database->bodies.getAddress(bodyID);
		}

		int AddCollider(Database* database, int bodyID, ftTransform transform, ftShape* shape) {
			Body* body = database->bodies.getAddress(bodyID);
			Collider* oldStart = database->colliders.buffer;
			int colliderID = database->colliders.alloc();
			Collider* newStart = database->colliders.buffer;
			if (newStart - oldStart != 0) {
				for (int i = 0; i < database->bodies.size; ++i) {
					Body* body = &database->bodies.buffer[i];
					Collider* initAddr = body->collider;
					if (body->collider != nullptr) body->collider = newStart + (initAddr - oldStart);
				}

				for (int i = 0; i < database->colliders.size; ++i) {
					Collider* collider = &database->colliders.buffer[i];
					if (collider->prev != nullptr) collider->prev = newStart + (collider->prev - oldStart);
					if (collider->next != nullptr) collider->next = newStart + (collider->next - oldStart);;
				}
			}
			Collider* collider = database->colliders.getAddress(colliderID);
			collider->colliderID = colliderID;
			collider->body = body;
			collider->transform = transform;
			collider->friction = 0.2f;
			collider->restitution = 0.2f;
			collider->group = 0;
			collider->mask = 0xFFFF;
			collider->category = 0xFFFF;
			collider->shape = shape;

			collider->spatialID = database->spatialIndex.addShape(shape,
				body->transform * collider->transform,
				(void*) colliderID);
			if (body->collider != nullptr) body->collider->prev = collider;
			collider->next = body->collider;
			collider->prev = nullptr;
			body->collider = collider;

			sprintf(collider->alias, "Collider%d", colliderID);

			return colliderID;
		}

		void DeleteCollider(Database* database, int colliderID) {

			Collider* collider = GetCollider(database, colliderID);
			database->spatialIndex.removeShape(collider->spatialID);
			if (collider->prev != nullptr)
				collider->prev->next = collider->next;
			if (collider->next != nullptr)
				collider->next->prev = collider->prev;

			delete collider->shape;
			
			database->colliders.remove(collider);
			if (collider->prev != nullptr)
				collider->prev->next = collider;
			else
				collider->body->collider = collider;
			if (collider->next != nullptr)
				collider->next->prev = collider;
		}

		Collider* GetCollider(Database* database, int colliderID) {
			return database->colliders.getAddress(colliderID);
		}

		int CreateDistanceJoint(Database* database, int bodyIDA, int bodyIDB, 
			ftVector2 localAnchorA, ftVector2 localAnchorB) {
			int jointID = CreateJoint(database, bodyIDA, bodyIDB);
			Joint* joint = GetJoint(database, jointID);
			joint->jointType = JOINT_TYPE_DISTANCE;
			joint->distance.localAnchorA = localAnchorA;
			joint->distance.localAnchorB = localAnchorB;
			joint->distance.isCustomDistance = false;
			ftVector2 worldAnchorA = joint->body[0]->transform * localAnchorA;
			ftVector2 worldAnchorB = joint->body[1]->transform * localAnchorB;
			joint->distance.distance = (worldAnchorA - worldAnchorB).magnitude();
			return jointID;
		}

		int CreateDynamoJoint(Database* database, int bodyIDA, int bodyIDB, 
			real targetRate, real maxTorque) {
			int jointID = CreateJoint(database, bodyIDA, bodyIDB);
			Joint* joint = GetJoint(database, jointID);
			joint->jointType = JOINT_TYPE_DYNAMO;
			joint->dynamo.targetRate = targetRate;
			joint->dynamo.maxTorque = maxTorque;
			return jointID;
		}

		int CreateHingeJoint(Database* database, int bodyIDA, int bodyIDB, ftVector2 anchorPoint) {
			int jointID = CreateJoint(database, bodyIDA, bodyIDB);
			Joint* joint = GetJoint(database, jointID);
			joint->jointType = JOINT_TYPE_HINGE;
			joint->hinge.anchorPoint = anchorPoint;
			joint->hinge.enableLimit = false;
			joint->hinge.upperLimit = 0;
			joint->hinge.lowerLimit = 0;
			return jointID;
		}

		int CreatePistonJoint(Database* database, int bodyIDA, int bodyIDB,
			ftVector2 localAnchorA, ftVector2 localAnchorB, ftVector2 axis) {
			int jointID = CreateJoint(database, bodyIDA, bodyIDB);
			Joint* joint = GetJoint(database, jointID);
			joint->jointType = JOINT_TYPE_PISTON;
			joint->piston.axis = axis;
			joint->piston.localAnchorA = localAnchorA;
			joint->piston.localAnchorB = localAnchorB;
			return jointID;
		}

		int CreateSpringJoint(Database* database, int bodyIDA, int bodyIDB,
			ftVector2 localAnchorA, ftVector2 localAnchorB) {
			int jointID = CreateJoint(database, bodyIDA, bodyIDB);
			Joint* joint = GetJoint(database, jointID);
			joint->jointType = JOINT_TYPE_SPRING;
			joint->spring.localAnchorA = localAnchorA;
			joint->spring.localAnchorB = localAnchorB;
			return jointID;
		}

		void DeleteJoint(Database* database, int jointID) {
			Joint* joint = GetJoint(database, jointID);
			DeleteJoint(database, joint);
		}

		void DeleteJoint(Database* database, Joint* joint) {
			for (int i = 0; i < 2; ++i) {
				if (joint->prev[i] != nullptr) {
					int bodyOrder = GetBodyOrderInJoint(joint->prev[i], joint->body[i]);
					joint->prev[i]->next[bodyOrder] = joint->next[i];
				}
				else {
					joint->body[i]->joint = joint->next[i];
				}
				if (joint->next[i] != nullptr) {
					int bodyOrder = GetBodyOrderInJoint(joint->next[i], joint->body[i]);
					joint->next[i]->prev[bodyOrder] = joint->prev[i];
				}
			}

			bool isLast = ((joint - database->joints.buffer) == database->joints.size - 1);

			database->joints.remove(joint);

			if (!isLast) {
				for (int i = 0; i < 2; ++i) {
					if (joint->prev[i] != nullptr) {
						int bodyOrder = GetBodyOrderInJoint(joint->prev[i], joint->body[i]);
						joint->prev[i]->next[bodyOrder] = joint;
					}
					else {
						joint->body[i]->joint = joint;
					}
					if (joint->next[i] != nullptr) {
						int bodyOrder = GetBodyOrderInJoint(joint->next[i], joint->body[i]);
						joint->next[i]->prev[bodyOrder] = joint;
					}
				}
			}
		}

		Joint* GetJoint(Database* database, int jointID) {
			return database->joints.getAddress(jointID);
		}

		bool PointQueryJoint(Database* database,
			ftVector2 point,
			real radius,
			int* jointID) {
			for (int i = 0; i < database->joints.size; ++i) {
				Entity::Joint* joint = &database->joints.buffer[i];
				switch (joint->jointType) {
				case JOINT_TYPE_HINGE: {
					ftVector2 diff = point - joint->hinge.anchorPoint;
					if (diff.x * diff.x + diff.y * diff.y < radius * radius) {
						(*jointID) = joint->jointID;
						return true;
					}
					break;
				}
				case JOINT_TYPE_DYNAMO: {
					ftVector2 diff = point - joint->body[1]->transform.center;
					if (diff.x * diff.x + diff.y * diff.y < radius * radius) {
						(*jointID) = joint->jointID;
						return true;
					}
					break;
				}
				case JOINT_TYPE_SPRING: {
					ftVector2 anchorA = joint->body[0]->transform * joint->spring.localAnchorA;
					ftVector2 anchorB = joint->body[1]->transform * joint->spring.localAnchorB;
					ftVector2 diffA = point - anchorA;
					ftVector2 diffB = point - anchorB;
					if (diffA.x * diffA.x + diffA.y * diffA.y < radius * radius) {
						(*jointID) = joint->jointID;
						return true;
					}
					if (diffB.x * diffB.x + diffB.y * diffB.y < radius * radius) {
						(*jointID) = joint->jointID;
						return true;
					}
					break;
				}
				case JOINT_TYPE_DISTANCE: {
					ftVector2 anchorA = joint->body[0]->transform * joint->distance.localAnchorA;
					ftVector2 anchorB = joint->body[1]->transform * joint->distance.localAnchorB;
					ftVector2 diffA = point - anchorA;
					ftVector2 diffB = point - anchorB;
					if (diffA.x * diffA.x + diffA.y * diffA.y < radius * radius) {
						(*jointID) = joint->jointID;
						return true;
					}
					if (diffB.x * diffB.x + diffB.y * diffB.y < radius * radius) {
						(*jointID) = joint->jointID;
						return true;
					}
					break;
				}
				case JOINT_TYPE_PISTON: {
					ftVector2 anchor = joint->body[0]->transform * joint->piston.localAnchorA;
					ftVector2 diff = point - anchor;
					if (diff.x * diff.x + diff.y * diff.y < radius * radius) {
						(*jointID) = joint->jointID;
						return true;
					}
					break;
				}
				}
			}
			return false;
		}

		bool PointQueryVertexes(Database* database, 
			ftVector2 point,
			real radius,
			ftVector2* vertex) {
			
			for (int i = 0; i < database->bodies.size; ++i) {
				Body* body = &database->bodies.buffer[i];
				Collider* collider = body->collider;
				while (collider != nullptr) {
					ftTransform transform = body->transform * collider->transform;
					if (collider->shape->shapeType == SHAPE_POLYGON) {
						ftPolygon* polygon = (ftPolygon*) collider->shape;
						for (int j = 0; j < polygon->numVertex; ++j) {
							(*vertex) = transform * polygon->vertices[j];
							ftVector2 diff = point - (*vertex);
							if (diff.x * diff.x + diff.y * diff.y < radius * radius) {
								return true;
							}
						}
					}
					collider = collider->next;
				}
			}
			return false;
		}

		bool PointQueryCollider(Database* database,
			ftVector2 point,
			int* colliderInRegion) {

			bool found = false;
			ftChunkArray<const void*> proxies;
			proxies.init(128);
			database->spatialIndex.regionQuery(ftAABB::Create(point, point + ftVector2(0.00000001, 0.00000001)),
				&proxies);
			std::cout << "Point Query Collider" << proxies.getSize() << std::endl;
			for (int i = 0; i < proxies.getSize(); ++i) {
				int colliderID = (int)proxies[i];

				Collider* collider = GetCollider(database, colliderID);
				Body* body = collider->body;
				ftTransform transform = body->transform * collider->transform;
				ftShape* shape = collider->shape;

				switch (shape->shapeType) {
				case SHAPE_CIRCLE: {
					ftCircle* circle = (ftCircle*)shape;
					float diffX = point.x - transform.center.x;
					float diffY = point.y - transform.center.y;
					float distSquared = diffX * diffX + diffY * diffY;
					std::cout << "Test" << std::endl;
					if (distSquared < circle->radius * circle->radius) {
						std::cout << "Pass Test" << std::endl;
						(*colliderInRegion) = collider->colliderID;
						found = true;
					}
					break;
				}
				case SHAPE_POLYGON: {
					ftPolygon* polygon = (ftPolygon*)shape;
					float polygonArea = 0;
					ftVector2 *transformedVertices = (ftVector2*)malloc(sizeof(ftVector2) * polygon->numVertex);
					for (int i = 0; i < polygon->numVertex; ++i) {
						transformedVertices[i] = transform * polygon->vertices[i];
					}
					for (int i = 0; i < polygon->numVertex; ++i) {
						int verticeIdx0 = i;
						int verticeIdx1 = (i + 1) % polygon->numVertex;
						float x1 = transformedVertices[verticeIdx0].x;
						float x2 = transformedVertices[verticeIdx1].x;
						float x3 = point.x;
						float y1 = transformedVertices[verticeIdx0].y;
						float y2 = transformedVertices[verticeIdx1].y;
						float y3 = point.y;

						float triangleArea = 0.5 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));
						polygonArea += abs(triangleArea);
					}
					free(transformedVertices);

					if (fabs(polygon->area - polygonArea) <= 0.001) {
						(*colliderInRegion) = collider->colliderID;
						found = true;
					}
					break;
				}
				}
			}
			proxies.cleanup();
			return found;
		}

		void PointQueryBodies(Database* database,
			ftVector2 point,
			float radius,
			ftVectorArray<int>* bodies) {
			for (int i = 0; i < database->bodies.size; ++i) {
				Body* body = &database->bodies.buffer[i];
				ftVector2 pos = body->transform.center;
				ftVector2 diff = point - pos;
				real distSquared = diff.x * diff.x + diff.y * diff.y;
				if (distSquared < radius * radius) {
					bodies->push(body->bodyID);
				}
			}
		}

		bool PointQueryBody(Database* database,
			ftVector2 point,
			float radius,
			int* bodyID) {
			for (int i = 0; i < database->bodies.size; ++i) {
				Body* body = &database->bodies.buffer[i];
				ftVector2 pos = body->transform.center;
				ftVector2 diff = point - pos;
				real distSquared = diff.x * diff.x + diff.y * diff.y;
				if (distSquared < radius * radius) {
					(*bodyID) = body->bodyID;
					return true;
				}
			}
			return false;
		}

		void RegionQueryBodies(Database* database,
			ftVector2 corner1,
			ftVector2 corner2,
			ftVectorArray<int>* bodies) {

			ftAABB aabb = ftAABB::Create(corner1, corner2);
			for (int i = 0; i < database->bodies.size; ++i) {
				Body* body = &database->bodies.buffer[i];
				ftVector2 pos = body->transform.center;
				if (pos.x > aabb.min.x && pos.x < aabb.max.x && 
					pos.y > aabb.min.y && pos.y < aabb.max.y) {
					bodies->push(body->bodyID);
				}
			}

		}

		void PointQueryCollider(Database* database,
			ftVector2 point,
			ftVectorArray<int>* colliders) {

			ftChunkArray<const void*> proxies;
			proxies.init(128);
			database->spatialIndex.regionQuery(ftAABB::Create(point, point + ftVector2(0.00000001, 0.0000001)),
				&proxies);
			for (int i = 0; i < proxies.getSize(); ++i) {
				int colliderID = (int)proxies[i];
				
				Collider* collider = GetCollider(database, colliderID);
				Body* body = collider->body;
				ftTransform transform = body->transform * collider->transform;
				ftShape* shape = collider->shape;

				switch (shape->shapeType) {
				case SHAPE_CIRCLE: {
					ftCircle* circle = (ftCircle*)shape;
					float diffX = point.x - transform.center.x;
					float diffY = point.y - transform.center.y;
					float distSquared = diffX * diffX + diffY * diffY;
					if (distSquared < circle->radius * circle->radius) {
						colliders->push(collider->colliderID);
					}
					break;
				}
				case SHAPE_POLYGON: {
					ftPolygon* polygon = (ftPolygon*)shape;
					float polygonArea = 0;
					ftVector2 *transformedVertices = (ftVector2*)malloc(sizeof(ftVector2) * polygon->numVertex);
					for (int i = 0; i < polygon->numVertex; ++i) {
						transformedVertices[i] = transform * polygon->vertices[i];
					}
					for (int i = 0; i < polygon->numVertex; ++i) {
						int verticeIdx0 = i;
						int verticeIdx1 = (i + 1) % polygon->numVertex;
						float x1 = transformedVertices[verticeIdx0].x;
						float x2 = transformedVertices[verticeIdx1].x;
						float x3 = point.x;
						float y1 = transformedVertices[verticeIdx0].y;
						float y2 = transformedVertices[verticeIdx1].y;
						float y3 = point.y;

						float triangleArea = 0.5 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));
						polygonArea += abs(triangleArea);
					}
					free(transformedVertices);

					if (fabs(polygon->area - polygonArea) <= 0.001) {
						colliders->push(collider->colliderID);
					}
					break;
				}
				}
			}
			proxies.cleanup();
		}

		void RegionQueryCollider(Database* database,
			ftVector2 corner1,
			ftVector2 corner2,
			ftVectorArray<int>* collidersInRegion) {

			ftChunkArray<const void*> proxies;
			proxies.init(128);
			database->spatialIndex.regionQuery(ftAABB::Create(corner1, corner2),
				&proxies);
			ftAABB region = ftAABB::Create(corner1, corner2);
			for (int i = 0; i < proxies.getSize(); ++i) {
				int colliderID = (int)proxies[i];
				Collider* collider = GetCollider(database, colliderID);
				Body* body = collider->body;
				ftTransform transform = body->transform * collider->transform;
				ftShape* shape = collider->shape;
				ftAABB shapeAABB = shape->constructAABB(transform);

				if (region.isContain(shapeAABB)) {
					collidersInRegion->push(collider->colliderID);
				}
			}
			proxies.cleanup();

		}
	}
}