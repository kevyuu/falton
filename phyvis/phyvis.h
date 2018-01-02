#pragma once


#include "dirent_portable.h"

#include "phyvis_render.h"
#include "phyvis_render.cpp"
#include "phyvis_entity.h"
#include "phyvis_entity.cpp"

#include "falton/math.h"
#include "falton/physics.h"

namespace Phyvis {

    enum BroadphaseType {
        BROADPHASE_TYPE_NSQUARED,
        BROADPHASE_TYPE_HGRID,
        BROADPHASE_TYPE_DYNAMICBVH,
        BROADPHASE_TYPE_TOROIDALGRID,
        BROADPHASE_TYPE_QUADTREE,
        BROADPHASE_TYPE_TOTAL
    };

    enum EditorState {
        EDITOR_STATE_SELECT_BODY,
        EDITOR_STATE_SELECT_ACTION,
        EDITOR_STATE_MOVE_BODY,
		EDITOR_STATE_COPY_BODY,
        EDITOR_STATE_REMOVE_BODY,
        EDITOR_STATE_ROTATE_BODY,
        EDITOR_STATE_ADD_BODY,
		EDITOR_STATE_MOVE_CAMERA,
        EDITOR_STATE_SIMULATION_RUNNING,
        EDITOR_STATE_TOTAL
    };

    enum SelectMode {
        SELECT_MODE_SELECT,
        SELECT_MODE_MOVE,
        SELECT_MODE_ROTATE,
        SELECT_MODE_TOTAL
    };

    
    enum ToolType {
        TOOL_TYPE_SELECT,
        TOOL_TYPE_ADD_BODY,
		TOOL_TYPE_MOVE_CAMERA,
        TOOL_TYPE_TOTAL
    };

    enum SelectAction {
        SELECT_ACTION_MOVE,
        SELECT_ACTION_ROTATE,
		SELECT_ACTION_COPY,
		SELECT_ACTION_FLIP,
        SELECT_ACTION_REMOVE,
		SELECT_ACTION_ALIGN_HORIZONTAL,
		SELECT_ACTION_EVEN_SPACE_HORIZONTAL,
        SELECT_ACTION_NONE,
        SELECT_ACTION_TOTAL
    };

    struct ToolsConfig {
        ToolType toolType;
        SelectMode selectMode;
        Entity::BodyType bodyType;
        Entity::ShapeType shapeType;
    };

    struct ControlsConfig {
        enum Event {
            EVENT_START,
            EVENT_RESTART,
            EVENT_PLAY,
            EVENT_PAUSE,
            EVENT_STOP,
            EVENT_NONE,
            EVENT_TOTAL
        };

        bool isRunning;
        bool isPlaying;
    };

    struct PhysicsConfig {
        ftPhysicsSystem::ftConfig systemConfig;
        ftHierarchicalGrid::ftConfig hierarchicalConfig;
        ftDynamicBVH::ftConfig bvhConfig;
        ftToroidalGrid::ftConfig toroidalConfig;
        ftQuadTree::ftConfig quadConfig;
        BroadphaseType broadphaseType;
    };

    struct Camera {
        ftVector2 center;
        float halfWidth, halfHeight;
        float scale;

        void MoveRight();
        void MoveLeft();
        void MoveUp();
        void MoveDown();
        void ZoomIn();
        void ZoomOut();
        ftVector2 ScreenToWorldSpaceDir(ftVector2 screenDir);
        ftVector2 ScreenToWorldSpace(ftVector2 viewCoord);
        ftVector2 WorldToScreenSpace(ftVector2 worldCoord);
        void GetProjectionMat(float* projectionMat);
    };
    
    struct Context {

        char projectName[128];

        Entity::Database database;
        Render::Context renderContext;

        ftPhysicsSystem physicsSystem;
        ftBroadphaseSystem* broadphaseSystem;

        Camera camera = {{0.0f, 0.0f}, 1280.0f/2000, 720.0f/2000, 1000};

		ftVectorArray<int> selectedColliders;
		ftVectorArray<ftCollider*> selectedFaltonColliders;

        PhysicsConfig physicsConfig;
        ToolsConfig toolsConfig;
        ControlsConfig controlsConfig;

        EditorState editorState;
		
		bool isMouseReleased[2];
		bool isMouseFirstDown[2];
		bool isMouseDown[2];
		bool isMouseDragging[2];
		ftVector2 mousePos;

		Render::Color bodyColors[Entity::BODY_TYPE_TOTAL];

    };

/* window related function */
    bool ToolsButton(const char* label, bool isSelected);
    bool ToolsGUI(ToolsConfig* toolsConfig);
    ControlsConfig::Event ControlsGUI(ControlsConfig* controlsConfig);
    void BodyInfoGUI(Context* context);
    void PhysicsConfigGUI(PhysicsConfig* config);
    void MenuBarGUI(Context* context);
    SelectAction SelectActionGUI(ftVector2 centerPos);
	void FaltonInfoGUI(ftVectorArray<ftCollider*>* selectedColliders);

        
    void Init(Context* context);
    void InitPhysics(Context* context);
    void Cleanup(Context* context);
    void Tick(Context* context, float widthPixel, float heightPixel);
    ftAABB GetSelectedAABB(Context* context);
    
}
