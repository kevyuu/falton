#pragma once

#include <GLFW/glfw3.h>
#include "dirent_portable.h"

#include "phyvis_render.h"
#include "phyvis_entity.h"

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
        EDITOR_STATE_SELECT_TOOL,
        EDITOR_STATE_EDIT_BODY,
				EDITOR_STATE_TRANSLATE_BODY,
				EDITOR_STATE_COPY_BODY,
				EDITOR_STATE_REMOVE_BODY,
				EDITOR_STATE_ROTATE_BODY,
				EDITOR_STATE_EDIT_BODY_END,
        EDITOR_STATE_ADD_BODY,
		EDITOR_STATE_ADD_COLLIDER,
		EDITOR_STATE_EDIT_JOINT,
				EDITOR_STATE_EDIT_JOINT_HINGE,
				EDITOR_STATE_EDIT_JOINT_DISTANCE,
				EDITOR_STATE_EDIT_JOINT_DYNAMO,
				EDITOR_STATE_EDIT_JOINT_PISTON,
				EDITOR_STATE_EDIT_JOINT_SPRING,
				EDITOR_STATE_EDIT_JOINT_END,
		EDITOR_STATE_ADD_JOINT,
				EDITOR_STATE_ADD_JOINT_HINGE,
				EDITOR_STATE_ADD_JOINT_DISTANCE,
				EDITOR_STATE_ADD_JOINT_DYNAMO,
				EDITOR_STATE_ADD_JOINT_PISTON,
				EDITOR_STATE_ADD_JOINT_SPRING,
				EDITOR_STATE_ADD_JOINT_END,
        EDITOR_STATE_MOVE_CAMERA,
        EDITOR_STATE_SWIPE_COPY,
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
		TOOL_TYPE_ADD_COLLIDER,
		TOOL_TYPE_EDIT_JOINT,
        TOOL_TYPE_ADD_JOINT,
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
        SELECT_ACTION_SWIPE_COPY,
        SELECT_ACTION_NONE,
        SELECT_ACTION_TOTAL
    };

    struct ToolsConfig {
        ToolType toolType;
        SelectMode selectMode;
        Entity::BodyType bodyType;
        Entity::ShapeType shapeType;
		Entity::JointType jointType;
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
    };

    struct FaltonInfo {
        static constexpr int NUM_FRAMES_TO_RECORD = 100;
        float frameTimes[NUM_FRAMES_TO_RECORD];
        float maxDuration;
        long frameCount;
        ftVectorArray<ftCollider*> selectedColliders;
		ftContact* selectedContact;
    };

	struct Style {
		Render::Color bodyColors[Entity::BODY_TYPE_TOTAL];
		Render::Color gridColor;

		enum HighlightColor {
			HIGHLIGHT_COLOR_SELECT_BODY,
			HIGHLIGHT_COLOR_HOVER_BODY,
			HIGHLIGHT_COLOR_JOINT_BODY_A,
			HIGHLIGHT_COLOR_JOINT_BODY_B,
			HIGHLIGHT_COLOR_TOTAL
		};
		Render::Color hightlightColors[HIGHLIGHT_COLOR_TOTAL];
		
		real dashLength;
	};

	struct MouseInput {
		static constexpr int LEFT_BUTTON = 0;
		static constexpr int RIGHT_BUTTON = 1;
		static constexpr int MIDDLE_BUTTON = 2;

		bool isFirstDown[3];
		bool isDown[3];
		bool isDragging[3];
		bool isReleased[3];
		ftVector2 clickScreenPos[3];
		ftVector2 clickWorldPos[3];

		ftVector2 screenDelta;
		ftVector2 worldDelta;
		ftVector2 prevScreenPos;
		ftVector2 screenPos;
		ftVector2 prevWorldPos;
		ftVector2 worldPos;

		float wheel;

	};
    
    struct Context {

        char projectName[128];

        Entity::Database database;
        Render::Context renderContext;

        ftPhysicsSystem physicsSystem;
        ftBroadphaseSystem* broadphaseSystem;

        Camera camera = {{0.0f, 0.0f}, 1280.0f/2000, 720.0f/2000, 1000};
        FaltonInfo faltonInfo;

		ftVectorArray<int> selectedBodies;
		int selectedJoint;

        ftPhysicsSystem::ftConfig faltonConfig;
        ToolsConfig toolsConfig;
        ControlsConfig controlsConfig;

		float gridSize;

		Style style;
        EditorState editorState;
		
		bool isMouseReleased[2];
		bool isMouseFirstDown[2];
		bool isMouseDown[2];
		bool isMouseDragging[2];
		ftVector2 mousePos;

        Render::Color bodyColors[Entity::BODY_TYPE_TOTAL];
        
		GLFWwindow* window;
    };

/* window related function */
    bool ToolsButton(const char* label, bool isSelected);
    bool ToolsGUI(float positionX, float positionY, ToolsConfig* toolsConfig);
    ControlsConfig::Event ControlsGUI(ControlsConfig* controlsConfig);
    void BodyInfoGUI(Context* context);
    void PhysicsConfigGUI(ftPhysicsSystem::ftConfig* config);
    void MenuBarGUI(Context* context);
    SelectAction SelectActionGUI(ftVector2 centerPos);
	void FaltonInfoGUI(float screenWidth, float screenHeight, FaltonInfo* faltonInfo);

        
    void Init(Context* context, GLFWwindow* window);
    void InitFalton(Context* context);
    void Cleanup(Context* context);
    void Tick(Context* context, float widthPixel, float heightPixel);
    ftAABB GetSelectedAABB(Context* context);
    
}
