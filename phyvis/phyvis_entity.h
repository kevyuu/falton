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
        
        enum ShapeType {
            SHAPE_TYPE_CIRCLE,
            SHAPE_TYPE_SQUARE,
            SHAPE_TYPE_POLYGON,
            SHAPE_TYPE_TOTAL
        };
        
        struct Body;
        struct Collider;
        struct SpatialProxy;

        struct Body {
			int bodyID;
            char alias[512];
            BodyType bodyType;
            ftTransform transform;
            ftVector2 velocity;
            ftVector2 centerOfMass;
            real mass;
            real moment;
            ftBody* faltonBody;
            Collider* collider;
        };

        struct Collider {
			int colliderID;
            Body* body;
            SpatialProxy* spatialProxy;
            ftTransform transform;
            real friction = 0.2;
            real restitution = 0.2;
            uint32 group = 0;
            uint32 category = 0xFFFF;
            uint32 mask = 0xFFFF;
        };

        struct SpatialProxy {
            ftShape* shape;
            Collider* collider;
            int spatialID;
        };

        struct Database {
            ftIDBuffer<Body> bodies;
            ftIDBuffer<Collider> colliders;
            ftIDBuffer<SpatialProxy> spatialProxies;
            ftHierarchicalGrid spatialIndex;
        };

        typedef void (*BodyIterFunc) (Body* body, void* data);
        typedef void (*ColliderIterFunc) (Collider* collider, void* data);

        void Init(Database* database);
        void Cleanup(Database* database);
        void Save(Database* database, const char* filepath);
        void Load(Database* database, const char* filepath);
        

        int AddBody(Database* database, ftTransform transform);
        void DeleteBody(Database* database, int bodyID);
        void DeleteBody(Database* database, Body* body);
        Body* GetBody(Database* database, int bodyID);
		void TransformBody(Database* database, ftTransform transform);
        void ForEveryBody(Database* database, BodyIterFunc iterFunc, void* iterData);
        
        int AddCollider(Database* database, int bodyID, ftTransform transform, ftShape* shape);
        Collider* GetCollider(Database* database, int colliderID);
        void ForEveryCollider(Database* database, ColliderIterFunc iterFunc, void* iterData);

        void AddToSpatialIndex(Database* database, int colliderID, ftShape* shape);

		void PointQueryCollider(Database* database, ftVector2 point, ftVectorArray<Collider*>* colliders);
		void RegionQueryCollider(Database* database, ftVector2 corner1, ftVector2 corner2, ftVectorArray<int>* collidersInRegion);
	}
}