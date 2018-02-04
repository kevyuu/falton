#include "falton/physics.h"

namespace Phyvis {
    namespace SpatialIndex {

        struct Grid {
            int *buckets;
            int capacity;
            int pointCount;
        };

        void Init(Buffer* buffer, ftVector2 gridSize, int capacity);
        void AddPoint(ftVector2 pos, int id);
        void DeletePoint(ftVector2 pos, int id);
        bool RegionQuery(ftAABB aabb, ftVector<int> ids);
        bool PointQuery(ftVector2 point, real radius, int id, ftVector2* pos);
        void Cleanup(Buffer* buffer);
    }
}