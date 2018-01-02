#include "phyvis_entity.h"

namespace Phyvis {
    namespace Entity {

        void Init(Database* database) {
            database->bodies.init(128);
            database->colliders.init(128);
            database->spatialProxies.init(128);
            database->spatialIndex.setConfiguration(ftHierarchicalGrid::ftConfig());
            database->spatialIndex.init();
        }

        void Cleanup(Database* database) {
            database->bodies.cleanup();
            database->colliders.cleanup();
            database->spatialProxies.cleanup();
            database->spatialIndex.shutdown();
        }
        
        void Save(Database* database, const char* filepath) {
            FILE* file = fopen(filepath, "w");
            if (file == NULL) return;
            fprintf(file, "%d %d\n", database->bodies.size, database->colliders.size);
            for (int i = 0; i < database->bodies.size; ++i) {
                Body* body = &database->bodies.buffer[i];
                fprintf(file, "%s\n", body->alias);
                fprintf(file, "%d\n", body->bodyType);
                fprintf(file, "%f %f\n", body->transform.center.x, body->transform.center.y);
                fprintf(file, "%f\n", body->transform.rotation.angle);
                fprintf(file, "%f %f\n", body->velocity.x, body->velocity.y);
                fprintf(file, "%f %f\n", body->centerOfMass.x, body->centerOfMass.y);
                fprintf(file, "%f %f\n", body->mass, body->moment);
            }
        
            for (int i = 0; i < database->colliders.size; ++i) {
                Collider* collider = &database->colliders.buffer[i];
                ftShape* shape = collider->spatialProxy->shape;
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
                    ftCircle* circle = (ftCircle*) shape;
                    fprintf(file, "%f\n", circle->radius);
                }
                else if (shape->shapeType == SHAPE_POLYGON) {
                    ftPolygon* polygon = (ftPolygon*) shape;
                    fprintf(file, "%d\n", polygon->numVertex);
                    for (int j = 0; j < polygon->numVertex; ++j) {
                        fprintf(file, "%f %f\n", polygon->vertices[j].x, polygon->vertices[j].y);
                    }
                }
            }
            fclose(file);
        }

        void Load(Database* database, const char* filepath) {
            FILE* file = fopen(filepath, "r");
            if (file == NULL) return;
            int bodyCount, colliderCount;
            fscanf(file, "%d %d", &bodyCount, &colliderCount);
            for (int i = 0; i < bodyCount; ++i) {
                int bodyID = database->bodies.alloc();
                Body* body = database->bodies.getAddress(bodyID);
				body -> bodyID = bodyID;
                fscanf(file, "%s", body->alias);
                fscanf(file, "%d", &body->bodyType);
                fscanf(file, "%f %f", &body->transform.center.x, &body->transform.center.y);
                real angle;
                fscanf(file, "%f", &angle);
                body->transform.rotation = ftRotation::Angle(angle);
                fscanf(file, "%f %f", &body->velocity.x, &body->velocity.y);
                fscanf(file, "%f %f", &body->centerOfMass.x, &body->centerOfMass.y);
                fscanf(file, "%f %f", &body->mass, &body->moment);
            }
        
            for (int i = 0; i < colliderCount; ++i) {
                int colliderID = database->colliders.alloc();
                int spatialProxyID = database->spatialProxies.alloc();
                Collider* collider = database->colliders.getAddress(colliderID);
				collider->colliderID = colliderID;
                SpatialProxy* spatialProxy = database->spatialProxies.getAddress(spatialProxyID);
                collider->spatialProxy = spatialProxy;
                spatialProxy->collider = collider;
        
                int bodyOffset;
                fscanf(file, "%d", &bodyOffset);
                Body* body = database->bodies.buffer + bodyOffset;
                collider->body = body;
                body->collider = collider;
        
                fscanf(file, "%f %f", &collider->transform.center.x, &collider->transform.center.y);\
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
                    spatialProxy->shape = ftCircle::create(radius);
                }
                else {
                    ftVector2 vertexes[128];
                    int vertexCount;
                    fscanf(file, "%d", &vertexCount);
                    for (int j = 0; j< vertexCount; ++j) {
                        fscanf(file, "%f %f", &(vertexes[j]).x, &(vertexes[j]).y);
                    }
                    spatialProxy->shape = ftPolygon::create(vertexCount, vertexes);
                }
                spatialProxy->spatialID = database->spatialIndex.addShape(spatialProxy->shape, 
                    body->transform * collider->transform, 
                    (void*) spatialProxyID); 
            }
            fclose(file);
        }
        
        int AddBody(Database* database, ftTransform transform) {
            int bodyID = database->bodies.alloc();
            Body* body = database->bodies.getAddress(bodyID);
			body->bodyID = bodyID;
            body->transform = transform;
            body->velocity = {0, 0};
            sprintf(body->alias, "Body%d", bodyID);
            return bodyID;
        }

		void TransformBody(Database* database, int bodyID, ftTransform transform) {
			Body* body = database->bodies.getAddress(bodyID);
			body->transform = transform;
			Collider* collider = body->collider;
			if (collider == NULL) return;
			SpatialProxy* spatialProxy = collider->spatialProxy;
			if (spatialProxy == NULL) return;
			database->spatialIndex.moveShape(spatialProxy->spatialID,
				spatialProxy->shape,
				body->transform * collider->transform);
		}

        void DeleteBody(Database* database, int bodyID) {
            Body* body = database->bodies.getAddress(bodyID);
            DeleteBody(database, body);

        }

        void DeleteBody(Database* database, Body* body) {
            Collider* collider = body->collider;
            SpatialProxy* spatialProxy = collider->spatialProxy;

            database->spatialIndex.removeShape(spatialProxy->spatialID);
            database->bodies.remove(body);
            body->collider->body = body;
            database->colliders.remove(collider);
            collider->spatialProxy->collider = collider;
			collider->body->collider = collider;
            database->spatialProxies.remove(spatialProxy);
			spatialProxy->collider->spatialProxy = spatialProxy;
        }

        Body* GetBody(Database* database, int bodyID) {
            return database->bodies.getAddress(bodyID);
        }

        void ForEveryBody(Database* database, BodyIterFunc iterFunc, void* iterData) {
            for (int i = 0; i < database->bodies.size; ++i) {
                Body* body = &(database->bodies.buffer[i]);
                iterFunc(body, iterData);
            }
        }

        int AddCollider(Database* database, int bodyID, ftTransform transform, ftShape* shape) {
            Body* body = database->bodies.getAddress(bodyID);
            int colliderID= database->colliders.alloc();
            Collider* collider = database->colliders.getAddress(colliderID);
			collider->colliderID = colliderID;
            collider->body = body;
            collider->transform = transform;
            collider->friction = 0.2f;
            collider->restitution = 0.2f;
            body->collider = collider;

            AddToSpatialIndex(database, colliderID, shape);

            return colliderID;
        }

        Collider* GetCollider(Database* database, int colliderID) {
            return database->colliders.getAddress(colliderID);
        }

        void ForEveryCollider(Database* database, ColliderIterFunc iterFunc, void* iterData) {
            for (int i = 0; i < database->colliders.size; ++i) {
                iterFunc(&(database->colliders.buffer[i]), iterData);
            }
        }

        void AddToSpatialIndex(Database* database, int colliderID, ftShape* shape) {
            int spatialProxyID = database->spatialProxies.alloc();
            Collider* collider = database->colliders.getAddress(colliderID);
            Body* body = collider->body;
            SpatialProxy* spatialProxy = database->spatialProxies.getAddress(spatialProxyID);
            spatialProxy->shape = shape;
            spatialProxy->collider = collider;
            spatialProxy->spatialID = database->spatialIndex.addShape(shape, 
                                    body->transform * collider->transform, 
                                    (void*) spatialProxyID);
            collider->spatialProxy = spatialProxy;
        }

        SpatialProxy* IterateSpatialProxy(Database* database) {
            return database->spatialProxies.buffer;
        }

		void PointQueryCollider(Database* database, 
			ftVector2 point,
			ftVectorArray<int>* colliders) {

            std::cout<<"Point Query Collider"<<std::endl;
			ftChunkArray<const void*> proxies;
			proxies.init(128);
			database->spatialIndex.regionQuery(ftAABB::Create(point, point + ftVector2(0.000001, 0.000001)),
				&proxies);
			for (int i = 0; i < proxies.getSize(); ++i) {
				int spatialProxyID = (int)proxies[i];
				SpatialProxy* proxy = database->spatialProxies.getAddress((spatialProxyID));

				Collider* collider = proxy->collider;
				Body* body = collider->body;
				ftTransform transform = body->transform * collider->transform;
				ftShape* shape = proxy->shape;
				
				switch (shape->shapeType) {
				case SHAPE_CIRCLE: {
					ftCircle* circle = (ftCircle*)shape;
					float diffX = point.x - transform.center.x;
					float diffY = point.y - transform.center.y;
					float distSquared = diffX * diffX + diffY * diffY;
					if (distSquared > circle->radius * circle->radius) {
						colliders->push(collider->colliderID);
					}
				}
				case SHAPE_POLYGON: {
                    ftPolygon* polygon = (ftPolygon*)shape;
                    float polygonArea = 0;
                    ftVector2 *transformedVertices = (ftVector2*)malloc(sizeof(float) * polygon->numVertex);
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
				}
				}
			}
            proxies.cleanup();
            std::cout<<"End Point Query Collider"<<std::endl;
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
				int spatialProxyID = (int)proxies[i];
				SpatialProxy* proxy = database->spatialProxies.getAddress((spatialProxyID));

				Collider* collider = proxy->collider;
				Body* body = collider->body;
				ftTransform transform = body->transform * collider->transform;
				ftShape* shape = proxy->shape;
				ftAABB shapeAABB = shape->constructAABB(transform);
				
				if (region.isContain(shapeAABB)) {
					collidersInRegion->push(collider->colliderID);
				}
			}
			proxies.cleanup();

		}
    }
}