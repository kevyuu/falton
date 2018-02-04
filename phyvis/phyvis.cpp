#include "phyvis.h"
#include "phyvis_overlay.h"
#include <IMGUI/imgui.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <chrono>

namespace Phyvis {

	static ftVector2 WorldToScreenSpace(Camera* camera, ftVector2 worldCoord) {
		float worldHalfWidth = camera->halfWidth;
		float worldHalfHeight = camera->halfHeight;
		float screenHalfWidth = worldHalfWidth * camera->scale;
		float screenHalfHeight = worldHalfHeight * camera->scale;

		float screenX = (worldCoord.x - (camera->center.x - worldHalfWidth)) * screenHalfWidth / worldHalfWidth;
		float screenY = ((camera->center.y + worldHalfHeight) - worldCoord.y) * screenHalfHeight / worldHalfHeight;
		return { screenX, screenY };
	}

    static ftVector2 ScreenToWorldSpaceDir(Camera* camera, ftVector2 screenDir) {
        float screenHalfWidth = camera->halfWidth * camera->scale;
        float screenHalfHeight = camera->halfHeight * camera->scale;
        
        float worldX = screenDir.x * camera->halfWidth / screenHalfWidth;
        float worldY = - screenDir.y * camera->halfHeight / screenHalfHeight;
        return {worldX, worldY};
    }

    static ftVector2 ScreenToWorldSpace(Camera* camera, ftVector2 screenCoord) {
        float screenHalfWidth = camera->halfWidth * camera->scale;
        float screenHalfHeight = camera->halfHeight * camera->scale;

        float worldX = (screenCoord.x * camera->halfWidth / screenHalfWidth) + (camera->center.x - camera->halfWidth);
        float worldY = (camera->center.y + camera->halfHeight) - (screenCoord.y * camera->halfHeight / screenHalfHeight);
        return {worldX, worldY};
    }

    static void GetProjectionMat(Camera* camera, float* projectionMat) {
        float l = camera->center.x - camera->halfWidth;
        float r = camera->center.x + camera->halfWidth;
        float b = camera->center.y - camera->halfHeight;
        float t = camera->center.y + camera->halfHeight;

        projectionMat[0] = 2.0f / (r - l);
        projectionMat[1] = 0.0f;
        projectionMat[2] = 0.0f;
        projectionMat[3] = 0.0f;

        projectionMat[4] = 0.0f;
        projectionMat[5] = 2.0f / (t - b);
        projectionMat[6] = 0.0f;
        projectionMat[7] = 0.0f;

        projectionMat[8] = 0.0f;
        projectionMat[9] = 0.0f;
        projectionMat[10] = 0.0f;
        projectionMat[11] = 0.0f;

        projectionMat[12] = - (r + l) / (r - l);
        projectionMat[13] = - (t + b) / (t - b);
        projectionMat[14] = 0.0f;
        projectionMat[15] = 1.0f;
    }


    inline void SetupImGuiStyle( bool bStyleDark_, float alpha_  )
    {
        ImGuiStyle& style = ImGui::GetStyle();
        
        style.Alpha = 1.0f;
        style.FrameRounding = 3.0f;
        style.Colors[ImGuiCol_Text]                  = ImVec4(0.80f, 0.80f, 0.80f, 1.00f);
        style.Colors[ImGuiCol_TextDisabled]          = ImVec4(0.60f, 0.60f, 0.60f, 1.00f);
        style.Colors[ImGuiCol_WindowBg]              = ImVec4(0.94f, 0.94f, 0.94f, 0.6f);
        style.Colors[ImGuiCol_ChildWindowBg]         = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
        style.Colors[ImGuiCol_PopupBg]               = ImVec4(1.00f, 1.00f, 1.00f, 0.94f);
        style.Colors[ImGuiCol_Border]                = ImVec4(1.00f, 1.00f, 1.00f, 1.00f);
        style.Colors[ImGuiCol_BorderShadow]          = ImVec4(1.00f, 1.00f, 1.00f, 0.10f);
        style.Colors[ImGuiCol_FrameBg]               = ImVec4(1.00f, 1.00f, 1.00f, 0.94f);
        style.Colors[ImGuiCol_FrameBgHovered]        = ImVec4(0.26f, 0.59f, 0.98f, 0.40f);
        style.Colors[ImGuiCol_FrameBgActive]         = ImVec4(0.26f, 0.59f, 0.98f, 0.67f);
        style.Colors[ImGuiCol_TitleBg]               = ImVec4(0.96f, 0.96f, 0.96f, 0.00f);
        style.Colors[ImGuiCol_TitleBgCollapsed]      = ImVec4(1.00f, 1.00f, 1.00f, 0.51f);
        style.Colors[ImGuiCol_TitleBgActive]         = ImVec4(0.3f, 0.3f, 0.3f, 0.00f);
        style.Colors[ImGuiCol_MenuBarBg]             = ImVec4(0.3f, 0.3f, 0.3f, 1.00f);
        style.Colors[ImGuiCol_ScrollbarBg]           = ImVec4(0.98f, 0.98f, 0.98f, 0.53f);
        style.Colors[ImGuiCol_ScrollbarGrab]         = ImVec4(0.69f, 0.69f, 0.69f, 1.00f);
        style.Colors[ImGuiCol_ScrollbarGrabHovered]  = ImVec4(0.59f, 0.59f, 0.59f, 1.00f);
        style.Colors[ImGuiCol_ScrollbarGrabActive]   = ImVec4(0.49f, 0.49f, 0.49f, 1.00f);
        style.Colors[ImGuiCol_ComboBg]               = ImVec4(0.86f, 0.86f, 0.86f, 0.99f);
        style.Colors[ImGuiCol_CheckMark]             = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
        style.Colors[ImGuiCol_SliderGrab]            = ImVec4(0.24f, 0.52f, 0.88f, 1.00f);
        style.Colors[ImGuiCol_SliderGrabActive]      = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
        style.Colors[ImGuiCol_Button]                = ImVec4(0.169f, 0.222f, 0.249f, 1.00f);
        style.Colors[ImGuiCol_ButtonHovered]         = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
        style.Colors[ImGuiCol_ButtonActive]          = ImVec4(0.06f, 0.53f, 0.98f, 1.00f);
        style.Colors[ImGuiCol_Header]                = ImVec4(0.26f, 0.59f, 0.98f, 0.31f);
        style.Colors[ImGuiCol_HeaderHovered]         = ImVec4(0.26f, 0.59f, 0.98f, 0.80f);
        style.Colors[ImGuiCol_HeaderActive]          = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
        style.Colors[ImGuiCol_Column]                = ImVec4(0.39f, 0.39f, 0.39f, 1.00f);
        style.Colors[ImGuiCol_ColumnHovered]         = ImVec4(0.26f, 0.59f, 0.98f, 0.78f);
        style.Colors[ImGuiCol_ColumnActive]          = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
        style.Colors[ImGuiCol_ResizeGrip]            = ImVec4(1.00f, 1.00f, 1.00f, 0.50f);
        style.Colors[ImGuiCol_ResizeGripHovered]     = ImVec4(0.26f, 0.59f, 0.98f, 0.67f);
        style.Colors[ImGuiCol_ResizeGripActive]      = ImVec4(0.26f, 0.59f, 0.98f, 0.95f);
        style.Colors[ImGuiCol_CloseButton]           = ImVec4(0.59f, 0.59f, 0.59f, 0.50f);
        style.Colors[ImGuiCol_CloseButtonHovered]    = ImVec4(0.98f, 0.39f, 0.36f, 1.00f);
        style.Colors[ImGuiCol_CloseButtonActive]     = ImVec4(0.98f, 0.39f, 0.36f, 1.00f);
        style.Colors[ImGuiCol_PlotLines]             = ImVec4(0.39f, 0.39f, 0.39f, 1.00f);
        style.Colors[ImGuiCol_PlotLinesHovered]      = ImVec4(1.00f, 0.43f, 0.35f, 1.00f);
        style.Colors[ImGuiCol_PlotHistogram]         = ImVec4(0.90f, 0.70f, 0.00f, 1.00f);
        style.Colors[ImGuiCol_PlotHistogramHovered]  = ImVec4(1.00f, 0.60f, 0.00f, 1.00f);
        style.Colors[ImGuiCol_TextSelectedBg]        = ImVec4(0.26f, 0.59f, 0.98f, 0.35f);
        style.Colors[ImGuiCol_ModalWindowDarkening]  = ImVec4(0.40f, 0.40f, 0.40f, 0.35f);

        if( bStyleDark_ )
        {
            for (int i = 0; i <= ImGuiCol_COUNT; i++)
            {
                ImVec4& col = style.Colors[i];
                float H, S, V;
                ImGui::ColorConvertRGBtoHSV( col.x, col.y, col.z, H, S, V );

                if( S < 0.1f )
                {
                V = 1.0f - V;
                }
                ImGui::ColorConvertHSVtoRGB( H, S, V, col.x, col.y, col.z );
                if( col.w < 1.00f )
                {
                    col.w *= alpha_;
                }
            }
        }
        else
        {
            for (int i = 0; i <= ImGuiCol_COUNT; i++)
            {
                ImVec4& col = style.Colors[i];
                if( col.w < 1.00f )
                {
                    col.x *= alpha_;
                    col.y *= alpha_;
                    col.z *= alpha_;
                    col.w *= alpha_;
                }
            }
        }
    }

	static void InitStyle(Style* style) {
		style->bodyColors[Entity::BODY_TYPE_STATIC] = { 0.5f, 0.04f, 0.0f, 1.0f };
		style->bodyColors[Entity::BODY_TYPE_KINEMATIC] = { 0.35f, 0.57f, 0.35f, 1.0f };
		style->bodyColors[Entity::BODY_TYPE_DYNAMIC] = { 0.32f, 0.62f, 0.84f, 1.0f };
		style->gridColor = { 0.16f, 0.29f, 0.35f, 1.0f };

		style->hightlightColors[Style::HIGHLIGHT_COLOR_SELECT_BODY] = { 0.87f, 0.46f, 0.03f, 1.0f };
		style->hightlightColors[Style::HIGHLIGHT_COLOR_HOVER_BODY] = { 0.4f, 0.4f, 0.4f, 0.4f };
		style->hightlightColors[Style::HIGHLIGHT_COLOR_JOINT_BODY_A] = { 1.0f, 0.0f, 0.0f, 0.7f };
		style->hightlightColors[Style::HIGHLIGHT_COLOR_JOINT_BODY_B] = { 0.0f, 1.0f, 0.0f, 0.7f };

		Overlay::g_Style.colors[Overlay::COLOR_ID_SELECTION] = { 0.87f, 0.46f, 0.03f, 1.0 };
		Overlay::g_Style.colors[Overlay::COLOR_ID_BODY_X_AXIS] = { 0.8f, 0.0f, 0.0f, 1.0f };
		Overlay::g_Style.colors[Overlay::COLOR_ID_BODY_Y_AXIS] = { 0.0f, 0.8f, 0.0f, 1.0f };
		Overlay::g_Style.colors[Overlay::COLOR_ID_SNAP] = { 1.0f, 1.0f, 1.0f, 1.0f };
		Overlay::g_Style.colors[Overlay::COLOR_ID_SWIPE_COPY_LINE] = { 0.8f, 0.8f, 0.8f, 1.0f };
		Overlay::g_Style.colors[Overlay::COLOR_ID_CONTACT] = { 0.4f, 0.6f, 0.4f, 1.0f };
		Overlay::g_Style.colors[Overlay::COLOR_ID_REF_X_AXIS] = { 1.0f, 0.0f, 0.0f, 1.0f };
		Overlay::g_Style.colors[Overlay::COLOR_ID_REF_Y_AXIS] = { 0.0f, 1.0f, 0.0f, 1.0f };
		Overlay::g_Style.colors[Overlay::COLOR_ID_REF_POINT] = { 0.0f, 0.0f, 1.0f, 1.0f };
		Overlay::g_Style.colors[Overlay::COLOR_ID_JOINT_SIGIL] = { 1.0f, 1.0f, 1.0f, 1.0f };
		Overlay::g_Style.colors[Overlay::COLOR_ID_JOINT_LINK_A] = { 1.0f, 0.0f, 0.0f, 1.0f };
		Overlay::g_Style.colors[Overlay::COLOR_ID_JOINT_LINK_B] = { 0.0f, 1.0f, 0.0f, 1.0f };

		style->dashLength = 0.1f;
	}

    void Init(Context* context, GLFWwindow* window) {
		Overlay::Init();
        Render::Init(&context->renderContext);
        Entity::Init(&context->database);
		InitStyle(&context->style);

        strcpy(context->projectName, "NewProject");
        context->editorState = EDITOR_STATE_EDIT_BODY;
        context->toolsConfig.toolType = TOOL_TYPE_SELECT;
        context->toolsConfig.shapeType = Entity::SHAPE_TYPE_CIRCLE;
		context->toolsConfig.bodyType = Entity::BODY_TYPE_DYNAMIC;
		context->toolsConfig.jointType = Entity::JOINT_TYPE_HINGE;
        context->controlsConfig.isRunning = false;
        context->controlsConfig.isPlaying = false;
        context->selectedBodies.init(128);

		context->editorState = EDITOR_STATE_EDIT_BODY;
		
		context->faltonInfo.selectedColliders.init(128);
		context->selectedJoint = -1;

		context->camera.center = { 0, 0 };
		context->camera.scale = 100;

		context->window = window;

		context->gridSize = 1;

        SetupImGuiStyle(false, 0.4);
    }

    void Cleanup(Context* context) {
        Render::Cleanup(&context->renderContext);
		Entity::Cleanup(&context->database);;
		Overlay::Cleanup();
        context->selectedBodies.cleanup();
		context->faltonInfo.selectedColliders.cleanup();
    }

    void MenuBarGUI(Context* context) {
        
        bool isOpenProject = false;
        bool isSaveProject = false;

        if (ImGui::BeginMainMenuBar()) {
            if (ImGui::BeginMenu("Project")) {
                if (ImGui::MenuItem("New")) {
                    Entity::Cleanup(&context->database);
                    Entity::Init(&context->database);
                }
                if (ImGui::MenuItem("Open"))
                    isOpenProject = true;
                if (ImGui::MenuItem("Save")) 
                    isSaveProject = true;
                ImGui::EndMenu();
            }
            ImGui::EndMainMenuBar();
        }
        ImGui::SetNextWindowPosCenter();

        static char (*projectNames)[128] = nullptr;
        static int projectCount = 0;

        if (isOpenProject) {
            projectCount = 0;
            ImGui::OpenPopup("Open Project");
            DIR *dir = opendir("./demo");
            for (dirent* ent = readdir(dir); ent!= NULL; ent = readdir(dir)) {
                char* dotPosition = strchr(ent->d_name, '.');
                if (dotPosition == NULL) continue;
                char* extension = dotPosition + 1;
                if (strcmp(extension, "pvs") == 0) ++projectCount;
            }
            closedir(dir);

            if (projectNames != NULL) free(projectNames);
            projectNames = (char (*)[128])malloc(sizeof(*projectNames) * projectCount);
            projectCount = 0;
            dir = opendir("./demo");
            for (dirent* ent = readdir(dir); ent!= NULL; ent = readdir(dir)) {
                char* dotPosition = strchr(ent->d_name, '.');
                if (dotPosition == NULL) continue;
                char* extension = dotPosition + 1;
                if (strcmp(extension, "pvs") == 0) {
                    strcpy(projectNames[projectCount], ent->d_name);
                    ++projectCount;
                } 
            }
            closedir(dir);

        }
        if (ImGui::BeginPopupModal("Open Project", NULL, ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::Text("Project List");
            ImGui::Separator();
            
            ImGui::BeginChild("Project List", ImVec2(150, 150), true);
            static int selectedProject = 0;
            for (int i = 0; i < projectCount; ++i) {
                if (ImGui::Selectable(projectNames[i], i == selectedProject)) {
                    selectedProject = i;
                }
            }
            ImGui::EndChild();
            if (ImGui::Button("Open")) {
                Entity::Cleanup(&context->database);
                Entity::Init(&context->database);
                char filepath[128] = "./demo/";
                strcat(filepath, projectNames[selectedProject]);
				strcpy(context->projectName, projectNames[selectedProject]);
				char* dotPosition = strchr(context->projectName, '.');
				*dotPosition = '\0';
                Entity::Load(&context->database, filepath);
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel")) { ImGui::CloseCurrentPopup();}
            ImGui::EndPopup();
        }

        if (isSaveProject) ImGui::OpenPopup("Save Project");
        if (ImGui::BeginPopupModal("Save Project", NULL, ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::InputText("Project Name", context->projectName, 128);
            if (ImGui::Button("Save")) {
				char filepath[128] = "./demo/";
				strcat(filepath, context->projectName);
				strcat(filepath, ".pvs");
                Entity::Save(&context->database, filepath);
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel")) {
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }
        
    }

    ControlsConfig::Event ControlsGUI(ControlsConfig* config) {

        static const ImVec2 buttonSize = ImVec2(50.0 , 15.0);
        static const ImColor buttonDisabledColor = ImColor(200,200,200);
        static const ImColor buttonActiveColor = ImColor(200, 10, 10);
        static const char* runActive = "Restart";
        static const char* runDeactive = "Start";
        static const char* playActive  = "Pause";
        static const char* playDeactive = "Play";

        bool* running = &(config->isRunning);
        bool* playing = &(config->isPlaying);
        ControlsConfig::Event event = ControlsConfig::EVENT_NONE;
        
        ImGuiWindowFlags window_flags = 0;
        window_flags |= ImGuiWindowFlags_NoTitleBar;
        window_flags |= ImGuiWindowFlags_NoResize;
        window_flags |= ImGuiWindowFlags_NoMove;
        window_flags |= ImGuiWindowFlags_NoScrollbar;
        window_flags |= ImGuiWindowFlags_NoCollapse;
        ImGui::SetNextWindowPosCenterH(20, ImGuiSetCond_Always);
        ImGui::Begin("Play", nullptr, window_flags);

        const char* runText = runDeactive;
        if (*running) runText = runActive;

        const char* playText = playDeactive;
        if (*playing) playText = playActive;

        const char* stopText = "Stop";

        if (ImGui::Button(runText, buttonSize)) {
            if (*running) event = ControlsConfig::EVENT_RESTART;
            else event = ControlsConfig::EVENT_START;
            *running = true;
        }

        bool changeColor = false;
        ImGui::PushID(1);
        if (!*running) {
            changeColor = true;
            ImGui::PushStyleColor(ImGuiCol_Button, buttonDisabledColor);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, buttonDisabledColor);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, buttonDisabledColor);
        }
        ImGui::SameLine();
        if (ImGui::Button(playText, buttonSize)) {
            if (*running) {
                event = ControlsConfig::EVENT_PLAY;
                *playing = !*playing;
            }
        }

        ImGui::SameLine();
        if (ImGui::Button(stopText, buttonSize)) {
            event = ControlsConfig::EVENT_STOP;
            *running = false;
            *playing = false;
        }

        if (changeColor) ImGui::PopStyleColor(3);
        ImGui::PopID();
        ImGui::End();
        
        return event;
    }

    bool ToolsButton(const char* label, bool isSelected) {
		float buttonWidth = ImGui::GetWindowWidth() / 3;
		buttonWidth = max(buttonWidth, 80);
        static const ImVec2 buttonSize = ImVec2(buttonWidth, 0);
        static const ImColor activeColor = ImColor(144, 164, 174);
        static const ImColor unactiveColor = ImColor(38, 50, 56);
        static const ImColor hoveredColor = ImColor(69, 90, 100);

        ImColor buttonColor = unactiveColor;
        if (isSelected) buttonColor = activeColor;
        ImGui::PushStyleColor(ImGuiCol_Button, buttonColor);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, hoveredColor);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, activeColor);
        bool isClicked = ImGui::Button(label, buttonSize);
        ImGui::PopStyleColor(3);
        return isClicked;
    }

    bool ToolsGUI(float positionX, float positionY, ToolsConfig* config) {

        bool isCloseGUI = false;

        ImGui::SetNextWindowPos(ImVec2(positionX, positionY),ImGuiSetCond_Always);
        ImGui::Begin("Tools");

		ImGui::Text("Shape Type");
		Entity::ShapeType* pShapeType = &config->shapeType;
		for (int i = 0; i < Entity::SHAPE_TYPE_TOTAL; ++i) {
			ImGui::RadioButton(Entity::SHAPE_TYPE_LABELS[i], (int*)pShapeType, i);
			if (i % 3 != 2 && i != Entity::SHAPE_TYPE_TOTAL - 1)ImGui::SameLine();
		}
		ImGui::Separator();

		ImGui::Text("Body Type");
		Entity::BodyType* pBodyType = &config->bodyType;
		for (int i = 0; i < Entity::BODY_TYPE_TOTAL; ++i) {
			ImGui::RadioButton(Entity::BODY_TYPE_LABELS[i], (int*)pBodyType, i);
			if (i % 3 != 2 && i != Entity::BODY_TYPE_TOTAL - 1)ImGui::SameLine();
		}
		ImGui::Separator();

		ImGui::Text("Joint Type");
		Entity::JointType* pJointType = &config->jointType;
		for (int i = 0; i < Entity::JOINT_TYPE_TOTAL; ++i) {
			ImGui::RadioButton(Entity::JOINT_TYPE_LABELS[i], (int*)pJointType, i);
			if (i % 3 != 2 && i != Entity::JOINT_TYPE_TOTAL - 1) ImGui::SameLine();
		}
		ImGui::Separator();

        if (ToolsButton("Select", config->toolType == TOOL_TYPE_SELECT)) {
            config->toolType = TOOL_TYPE_SELECT;
            config->selectMode = SELECT_MODE_SELECT;
            isCloseGUI = true;
        }
		ImGui::SameLine();
        if (ToolsButton("Add Body", config->toolType == TOOL_TYPE_ADD_BODY)) {
            config->toolType = TOOL_TYPE_ADD_BODY;
			isCloseGUI = true;
        }
		ImGui::SameLine();
		if (ToolsButton("Add Collider", config->toolType == TOOL_TYPE_ADD_COLLIDER)) {
			config->toolType = TOOL_TYPE_ADD_COLLIDER;
			isCloseGUI = true;
		}

		if (ToolsButton("Edit Joint", config->toolType == TOOL_TYPE_EDIT_JOINT)) {
			config->toolType = TOOL_TYPE_EDIT_JOINT;
			isCloseGUI = true;
		}
		ImGui::SameLine();
        if (ToolsButton("Add Joint", config->toolType == TOOL_TYPE_ADD_JOINT)) {
            config->toolType = TOOL_TYPE_ADD_JOINT;
			isCloseGUI = true;
        }
		ImGui::SameLine();
        if (ToolsButton("Move Camera", config->toolType == TOOL_TYPE_MOVE_CAMERA)) {
			config->toolType = TOOL_TYPE_MOVE_CAMERA;
			isCloseGUI = true;
        }
        
        ImVec2 parentWindowPos = ImGui::GetWindowPos();
        ImVec2 parentWindowSize = ImGui::GetWindowSize();
        ImGui::End();
    
        return isCloseGUI;
    }

    static void ImGui_InputBitMask(const char* label, uint32* bitmask) {
        for (int i = 0; i < 32; ++i) {
            char checkBoxLabel[100];
            sprintf(checkBoxLabel, "###%s%d", label, i);
            bool isBitOn = (*bitmask >> i) & 0x0001;
            ImGui::Checkbox(checkBoxLabel, &isBitOn);
            if (isBitOn)
                *bitmask = (*bitmask) | (1 << i);
            else
                *bitmask = (*bitmask) & (~ (1 << i));
            
            if (i%8 != 7) ImGui::SameLine();

        }
    }

	static ftAABB GetBodyAABB(Entity::Body* body) {
		Entity::Collider* collider = body->collider;
		ftAABB sumAABB = ftAABB::Create(body->transform.center, body->transform.center);
		while (collider != nullptr) {
			ftAABB aabb = collider->shape->constructAABB(body->transform * collider->transform);
			sumAABB = ftAABB::combine(aabb, sumAABB);
			collider = collider->next;
		}
		return sumAABB;
	}

	static void ContextMenuGUI(Context* context, MouseInput mouseInput) {
		float screenHeight = context->camera.halfHeight * 2 * context->camera.scale;
		float screenWidth = context->camera.halfWidth * 2 * context->camera.scale;
		ImGui::SetNextWindowPos(ImVec2(0, screenHeight - 30));
		ImGui::SetNextWindowSize(ImVec2(screenWidth, 30));
		ImGuiWindowFlags window_flags = 0;
		window_flags |= ImGuiWindowFlags_NoTitleBar;
		window_flags |= ImGuiWindowFlags_NoResize;
		window_flags |= ImGuiWindowFlags_NoMove;
		window_flags |= ImGuiWindowFlags_NoScrollbar;
		window_flags |= ImGuiWindowFlags_NoCollapse;
		ImGui::Begin("Context Menu", nullptr, window_flags);
		ImGui::PushItemWidth(100);

		static float s_gridSize = context->gridSize;
		static float s_prevSize = context->gridSize;
		static float s_currentSize = context->gridSize;
		s_currentSize = context->gridSize; 
		if (s_currentSize != s_prevSize) {
			s_gridSize = s_currentSize;
		}
		ImGui::InputFloat("Grid Size", &s_gridSize);
		if (s_gridSize > context->camera.halfWidth / 100) {
			context->gridSize = s_gridSize;
		}
		s_prevSize = s_currentSize;
		
		ImGui::SameLine();
		ImGui::Text("Mouse Pos : (%.03f, %.03f)", mouseInput.worldPos.x, mouseInput.worldPos.y);
		ImGui::PopItemWidth();
		ImGui::End();
	}

	static void EditJointGUI(Context* context, Entity::Joint* joint) {
		if (ImGui::CollapsingHeader(joint->alias)) {
			switch(joint->jointType) {
				case Entity::JOINT_TYPE_HINGE:{
					ImGui::InputFloat("Torque Friction", &joint->hinge.torqueFriction);
					ImGui::InputFloat2("Anchor Position", (float*)&joint->hinge.anchorPoint);
					ImGui::Checkbox("Enable Limit", &joint->hinge.enableLimit);
					ImGui::InputFloat("Lower Limit", &joint->hinge.lowerLimit);
					ImGui::InputFloat("Upper Limit", &joint->hinge.upperLimit);
					break;
				}
				case Entity::JOINT_TYPE_DYNAMO: {
					ImGui::InputFloat("Target Rate", &joint->dynamo.targetRate);
					ImGui::InputFloat("Mas Torque", &joint->dynamo.maxTorque);
					break;
				}
				case Entity::JOINT_TYPE_PISTON: {
					ImGui::InputFloat2("Local Anchor A", (float*) &joint->piston.localAnchorA);
					ImGui::InputFloat2("Local Anchor B", (float*) &joint->piston.localAnchorB);
					ImGui::InputFloat2("Axis", (float*) &joint->piston.axis);
					break;
				}
				case Entity::JOINT_TYPE_DISTANCE: {
					ImGui::InputFloat2("Local Anchor A", (float*)&joint->distance.localAnchorA);
					ImGui::InputFloat2("Local Anchor B", (float*)&joint->distance.localAnchorB);
					ImGui::Checkbox("##Is Custom Distance", &joint->distance.isCustomDistance);
					ImGui::SameLine();
					ImGui::InputFloat("Distance", &joint->distance.distance);
					break;
				}
				case Entity::JOINT_TYPE_SPRING: {
					ImGui::InputFloat2("Local Anchor A", (float*) &joint->spring.localAnchorA);
					ImGui::InputFloat2("Local Anchor B", (float*) &joint->spring.localAnchorB);
					ImGui::InputFloat("Rest Length", &joint->spring.restLength);
					ImGui::InputFloat("Stiffness", &joint->spring.stiffness);
					break;
				}
			}
			if (ImGui::Button("Delete")) {
				Entity::DeleteJoint(&context->database, joint->jointID);
				context->selectedJoint = -1;
			}
		}
	}

    void BodyInfoGUI(Context* context) {

        float screenWidth = context->camera.halfWidth * 2 * context->camera.scale;
        ImGui::SetNextWindowPos(ImVec2(screenWidth - 400, 20), ImGuiSetCond_Always);
        ImGui::Begin("Bodies");

        ImGui::Text("Mass Edit");
        ImGui::Separator();
        
        static bool s_changeFriction = false;
        static float s_massFriction = 0.0f;
        ImGui::Checkbox("###Friction", &s_changeFriction); ImGui::SameLine();
        ImGui::InputFloat("Friction", &s_massFriction);

        static bool s_changeRestitution = false;
        static float s_massRestitution = 0.0f;
        ImGui::Checkbox("###Restitution", &s_changeRestitution); ImGui::SameLine();
		ImGui::InputFloat("Restitution", &s_massRestitution);
		
		static bool s_changeGroup = false;
        static int s_massGroup = 0.0f;
        ImGui::Checkbox("###Group", &s_changeGroup); ImGui::SameLine();
        ImGui::InputInt("Group", &s_massGroup);

        if (ImGui::Button("Apply")) {
            for (int i = 0; i < context->selectedBodies.getSize(); ++i) {
                int bodyID = context->selectedBodies[i];
				Entity::Body* body = Entity::GetBody(&context->database, bodyID);
				Entity::Collider* collider = body->collider;
				while (collider != nullptr) {
					if (s_changeFriction) collider->friction = s_massFriction;
					if (s_changeRestitution) collider->restitution = s_massRestitution;
					if (s_changeGroup) collider->group = s_massGroup;
					collider = collider->next;
				}
            }    
        }

        for (int i = 0; i < context->selectedBodies.getSize(); ++i) {
			int bodyID = context->selectedBodies[i];
			Entity::Body* body = Entity::GetBody(&context->database, bodyID);
            char headerTitle[512];
            char headerLabel[512];
            strcpy(headerTitle, body->alias);
            sprintf(headerLabel, "###Body%d", i);
            strcat(headerTitle, headerLabel);
            if (ImGui::CollapsingHeader(headerTitle)) {
                ImGui::Combo("Body Type", (int*) &body->bodyType, 
                            Entity::BODY_TYPE_LABELS, (int)Entity::BODY_TYPE_TOTAL);
                ImGui::InputText("Alias", body->alias, sizeof(body->alias)/ sizeof(char));

                bool isTransformBody = false;
                ftVector2 bodyPosition = body->transform.center;
                real bodyAngle = body->transform.rotation.angle;
                if (ImGui::InputFloat2("Position", (float*) &bodyPosition)) 
                    isTransformBody = true;
                if (ImGui::InputFloat("Angle", &bodyAngle))
                    isTransformBody = true;
                if(isTransformBody)
                    Entity::TransformBody(&context->database, body->bodyID, ftTransform(bodyPosition, bodyAngle));
                
                ImGui::InputFloat2("Velocity", (float*) &(body->velocity));
				ImGui::InputFloat("Mass", &(body->mass));
				ImGui::Checkbox("##setCenterOfMass", &body->isCustomCenterOfMass); ImGui::SameLine();
				ImGui::InputFloat2("CenterOfMass", (float*)&body->centerOfMass);

				auto listCollider = [body](Entity::Collider* collider) {
					if (ImGui::CollapsingHeader(collider->alias)) {
						ImGui::PushItemWidth(0.35f * ImGui::GetWindowWidth());
						ImGui::SliderFloat("##Friction", &collider->friction, 0.0f, 1.0f);
						ImGui::SameLine();
						ImGui::InputFloat("Friction", &collider->friction);

						ImGui::SliderFloat("##Restitution", &collider->restitution, 0.0f, 1.0f);
						ImGui::SameLine();
						ImGui::InputFloat("Restitution", &collider->restitution);
						ImGui::PopItemWidth();

						ImGui::InputInt("Group", (int*)&collider->group);

						char maskHexString[7];
						sprintf(maskHexString, "0x%x", collider->mask);
						ImGui::InputText("Mask", maskHexString, 7,
							ImGuiInputTextFlags_CharsHexadecimal | ImGuiInputTextFlags_CharsUppercase);
						sscanf(maskHexString, "0x%x", &collider->mask);
						ImGui_InputBitMask("Mask", &collider->mask);

						char categoryHexString[7];
						sprintf(categoryHexString, "0x%x", collider->category);
						ImGui::InputText("Category", categoryHexString, 7,
							ImGuiInputTextFlags_CharsHexadecimal | ImGuiInputTextFlags_CharsUppercase);
						sscanf(categoryHexString, "0x%x", &collider->category);
						ImGui_InputBitMask("Category", &collider->category);

						ImGui::Text("Shape Info");
						ImGui::Separator();
						switch(collider->shape->shapeType) {
							case SHAPE_CIRCLE: {
								ftCircle* circle = (ftCircle*) collider->shape;
								ImGui::InputFloat("Radius",&circle->radius);
								break;
							}
							case SHAPE_POLYGON: {
								ftPolygon* polygon = (ftPolygon*) collider->shape;
								for (int j = 0; j < polygon->numVertex; ++j) {
									char labelLocal[20];
									char labelWorld[20];
									sprintf(labelLocal, "local vertex %d", j);
									sprintf(labelWorld, "world vertex %d", j);
									ImGui::InputFloat2(labelLocal, (float*)&polygon->vertices[j]);
									ftTransform transform = body->transform * collider->transform;
									ftVector2 worldCoord = transform * polygon->vertices[j];
									ftVector2 worldCoordBefore = worldCoord;
									ImGui::InputFloat2(labelWorld, (float*)&worldCoord);
									if (!(worldCoordBefore == worldCoord)) {
										polygon->vertices[j] = transform.invTransform(worldCoord);
									}
								}
								break;
							}
						}
					}
					
					collider = collider->next;
				};
				Entity::ForEveryColliderInBody(&context->database, bodyID, listCollider);

				auto listJoint = [context](Entity::Joint* joint) {
					EditJointGUI(context, joint);
				};
				Entity::ForEveryJointInBody(&context->database, bodyID, listJoint);
            }
        }
   
        ImGui::End();
    }

	static void JointInfoGUI(Context* context, Entity::Joint* joint) {
		float screenWidth = context->camera.halfWidth * 2 * context->camera.scale;
		ImGui::SetNextWindowPos(ImVec2(screenWidth - 350, 100), ImGuiSetCond_Always);
		ImGui::Begin("Joint Info");
		EditJointGUI(context, joint);
		ImGui::End();
	}

	void FaltonInfoGUI(float screenWidth, float screenHeight, FaltonInfo* faltonInfo) {
        ImGui::SetNextWindowPos(ImVec2(screenWidth - 600, 20), ImGuiSetCond_Always);
        ImGui::SetNextWindowSize(ImVec2(590, 2000));
        ImGui::Begin("Falton Info");
        ImGui::Text("Frame Count : %lld", faltonInfo->frameCount);
        ImGui::Text("Max Duration : %.0f microseconds", faltonInfo->maxDuration);
        int valueOffset = faltonInfo->frameCount % FaltonInfo::NUM_FRAMES_TO_RECORD;
        ImGui::PlotLines("Lines", faltonInfo->frameTimes, FaltonInfo::NUM_FRAMES_TO_RECORD, 
                        valueOffset, "avg 0.0", 0.0f, faltonInfo->maxDuration, ImVec2(580,150));
		for (int i = 0; i < faltonInfo->selectedColliders.getSize(); ++i) {
			ftCollider* faltonCollider = faltonInfo->selectedColliders[i];
			ftBody* body = faltonCollider->body;
			Entity::Body* editorBody = (Entity::Body*) body->userdata;
			char headerLabel[128];
			if (ImGui::CollapsingHeader(editorBody->alias)) {
				ImGui::InputFloat2("Position", (float*) &(body->transform.center));
				ImGui::InputFloat3("Rotation (cos - sin - angle)", (float*)&body->transform.rotation);
				ImGui::InputFloat2("Velocity", (float*) &(body->velocity));
				ImGui::InputFloat2("Angular Velocity", (float*)&body->angularVelocity);
				ImGui::InputFloat2("Center of Mass", (float*) &(body->centerOfMass));
				ImGui::InputInt("Body Type",(int*) &body->bodyType);
				ImGui::InputInt("Activation State", (int*) &body->activationState);
				ImGui::InputFloat("Mass", &body->mass);
				ImGui::InputFloat("Inverse Mass", &body->inverseMass);
				ImGui::InputFloat("Moment", &body->moment);
				ImGui::InputFloat("Inverse Moment", &body->inverseMoment);
				const auto listColliderInfo = [](ftCollider* collider) {
					char colliderHeaderLabel[128];
					sprintf(colliderHeaderLabel, "Collider%04x", collider);
					if (ImGui::CollapsingHeader(colliderHeaderLabel)) {
						ImGui::InputFloat("Friction", &collider->friction);
						ImGui::InputFloat("Restitution", &collider->restitution);
						ImGui::InputInt("Group", (int*)&collider->group);
						char maskHexString[7];
						sprintf(maskHexString, "0x%x", collider->mask);
						ImGui::InputText("Mask", maskHexString, 7,
							ImGuiInputTextFlags_CharsHexadecimal | ImGuiInputTextFlags_CharsUppercase);
						sscanf(maskHexString, "0x%x", &collider->mask);
						ImGui_InputBitMask("Mask", &collider->mask);

						char categoryHexString[7];
						sprintf(categoryHexString, "0x%x", collider->category);
						ImGui::InputText("Category", categoryHexString, 7,
							ImGuiInputTextFlags_CharsHexadecimal | ImGuiInputTextFlags_CharsUppercase);
						sscanf(categoryHexString, "0x%x", &collider->category);
						ImGui_InputBitMask("Category", &collider->category);
					}
				};
				body->forEachCollider(listColliderInfo);

				auto listContactProperty = [](ftContact* contact) {
					static const char* collisionStateString[3] = {
						"Begin Collision",
						"In Collision",
						"End Collision"
					};
					char contactHeaderLabel[128];
					sprintf(contactHeaderLabel, "Contact%04x", contact);
					if (ImGui::CollapsingHeader(contactHeaderLabel)) {
						ImGui::Text("Collision State : %s", collisionStateString[contact->collisionState]);
						ImGui::InputInt("Timestamp", (int*)&contact->timestamp);
						ImGui::InputInt("Island Index", (int*)&contact->islandIndex);
						ImGui::Text("Manifold");

						ImGui::InputFloat2("Normal", (float*)&contact->manifold.normal);
						ImGui::InputInt("Num Contact", (int*)&contact->manifold.numContact);

						for (int i = 0; i < contact->manifold.numContact; ++i) {
							ImGui::PushID(i);
							ImGui::Text("Contact Points %d", i);
							ImGui::InputFloat2("r1", (float*)&contact->manifold.contactPoints[i].r1);
							ImGui::InputFloat2("r2", (float*)&contact->manifold.contactPoints[i].r2);
							ImGui::InputFloat("nIAcc", &contact->manifold.contactPoints[i].nIAcc);
							ImGui::InputFloat("tIAcc", &contact->manifold.contactPoints[i].tIAcc);
							ImGui::InputFloat("Penetration Depth", &contact->manifold.penetrationDepth[i]);
							ImGui::PopID();
						}
					}
				
				};
				body->forEachContact(listContactProperty);

				auto listJointProperty = [](ftJoint* joint) {
					char jointHeaderLabel[30];
					sprintf(jointHeaderLabel, "Joint%04x", joint);
					if (ImGui::CollapsingHeader(jointHeaderLabel)) {
						switch (joint->jointType) {
							case ftJoint::DISTANCE_JOINT: {
								ftDistanceJoint* distance = (ftDistanceJoint*) joint;
								ImGui::InputFloat("Distance", &distance->distance);
								ImGui::InputFloat2("Local Anchor A", (float*)&distance->localAnchorA);
								ImGui::InputFloat2("Local Anchor B", (float*)&distance->localAnchorB);
								ImGui::InputFloat("InvK", &distance->invK);
								ImGui::InputFloat("Inverse Mass A", &distance->invMassA);
								ImGui::InputFloat("Inverse Mass B", &distance->invMassB);
								ImGui::InputFloat("Inverse Moment A", &distance->invMomentA);
								ImGui::InputFloat("Inverse Moment B", &distance->invMomentB);
								ImGui::InputFloat("Position Bias A", &distance->positionBias);
								ImGui::InputFloat2("jVa", (float*) &distance->jVa);
								ImGui::InputFloat("jWa", &distance->jWa);
								ImGui::InputFloat("jWb", &distance->jWb);
								ImGui::InputFloat("Impulse Acc", &distance->iAcc);
								break;
							}
							case ftJoint::DYNAMO_JOINT : {
								ftDynamoJoint* dynamo = (ftDynamoJoint*) joint;
								ImGui::InputFloat("Target Rate", &dynamo->targetRate);
								ImGui::InputFloat("Max Torque", &dynamo->maxTorque);
								ImGui::InputFloat("Max Impulse", &dynamo->maxImpulse);
								ImGui::InputFloat("Impulse Acc", &dynamo->iAcc);
								ImGui::InputFloat("InvK", &dynamo->invK);
								ImGui::InputFloat("Inverse Moment A", &dynamo->invMomentA);
								ImGui::InputFloat("Inverse Moment B", &dynamo->invMomentB);
								break;
							}
							case ftJoint::HINGE_JOINT : {
								ftHingeJoint* hinge = (ftHingeJoint*) joint;
	
								ImGui::InputFloat2("Anchor", (float*) &hinge->anchorPoint);
								ImGui::InputFloat("Inverse Mass A", &hinge->invMassA);
								ImGui::InputFloat("Inverse Mass B", &hinge->invMassB);
								ImGui::InputFloat("Inverse Moment A", &hinge->invMomentA);
								ImGui::InputFloat("Inverse Moment B", &hinge->invMomentB);
								ImGui::InputFloat("fIacc", &hinge->fIAcc);
								ImGui::InputFloat("fMaxImpulse", &hinge->fMaxImpulse);
								ImGui::InputFloat("invKFriciton", &hinge->invKFriction);
								break;
							}
						}
					}
				};

				body->forEachJoint(listJointProperty);
			}

		}
		ImGui::End();
	}

    void PhysicsConfigGUI(ftPhysicsSystem::ftConfig* config) {
        ImGui::SetNextWindowSize(ImVec2(450,300), ImGuiSetCond_Once);
        ImGui::Begin("Physics Config");

        ImGui::PushItemWidth(ImGui::GetContentRegionAvailWidth() * 0.5f);
        ImGui::Text("General");
        ImGui::InputFloat("Linear sleep limit", &config->sleepLinearLimit);
        ImGui::InputFloat("Angular sleep limit", &config->sleepAngularLimit);
		ImGui::InputFloat("Sleep time limit", &config->sleepTimeLimit);
		ImGui::SliderFloat("Sleep Ratio", &config->sleepRatio, 0.0f, 1.0f, "%.3f");
		ImGui::InputFloat2("Gravity",(float*)&config->gravity);

        ImGui::SliderFloat("Baumgarte Coefficient", &config->solverConfig.baumgarteCoef, 0.0f, 1.0f, "%.3f");
		ImGui::SliderFloat("Allowed Penetration", &config->solverConfig.allowedPenetration, 0.0f, 1.0f, "%.3f");
		ImGui::SliderFloat("Restitution Slop", &config->solverConfig.restitutionSlop, 0.0f, 1.0f, "%.3f");
        ImGui::InputInt("Solver Iteration",(int*)&config->solverConfig.numIteration);
		
		ImGui::NewLine();
        ImGui::Separator();
        ImGui::Text("Broadphase");

        const char* broadphaseNames[5] = {"NSquared Broadphase", "Dynamic BVH", "Toroidal Grid", "Hierarchical Grid", "Quad Tree"};

        ImGui::Combo("Broadphase", reinterpret_cast<int*>(&config->collisionConfig.broadphaseType), broadphaseNames, sizeof(broadphaseNames)/ sizeof(char*));
		
        switch (config->collisionConfig.broadphaseType) {
            case FT_BROADPHASE_TYPE_HIERARCHICAL_GRID : {

                ImGui::InputInt("Number Of Level", (int *)&config->collisionConfig.hierarchicalGridConfig.nLevel);
                ImGui::InputFloat("Base Size", &config->collisionConfig.hierarchicalGridConfig.baseSize);
                ImGui::InputFloat("Size Multiplier",&config->collisionConfig.hierarchicalGridConfig.sizeMul);
                ImGui::InputInt("Number Of Bucket", (int*)&config->collisionConfig.hierarchicalGridConfig.nBucket);
                break;
            }
            case FT_BROADPHASE_TYPE_DYNAMIC_BVH : {
                ImGui::InputFloat("AABB Extension", &config->collisionConfig.dynamicBVHConfig.aabbExtension);
                break;
            }
            case FT_BROADPHASE_TYPE_TOROIDAL_GRID : {
                ImGui::InputFloat("Cell Size", &config->collisionConfig.toroidalGridConfig.cellSize);
                ImGui::InputInt("nRow", (int*)&config->collisionConfig.toroidalGridConfig.nRow);
                ImGui::InputInt("nColumn", (int*)&config->collisionConfig.toroidalGridConfig.nCol);
                break;
            }
            case FT_BROADPHASE_TYPE_QUAD_TREE : {
                ImGui::InputFloat2("min", (float*)&config->collisionConfig.quadTreeConfig.worldAABB.min);
                ImGui::InputFloat2("max", (float*)&config->collisionConfig.quadTreeConfig.worldAABB.max);
                ImGui::InputInt("Max Level", (int*)&config->collisionConfig.quadTreeConfig.maxLevel);
                break;
            }
        }

        ImGui::PopItemWidth();
        ImGui::End();
    }

    SelectAction SelectActionGUI(ftVector2 centerPos) {
        static const ImVec2 buttonSize = ImVec2(80.0 , 15.0);

        ImGuiWindowFlags window_flags = 0;
        window_flags |= ImGuiWindowFlags_NoTitleBar;
        window_flags |= ImGuiWindowFlags_NoResize;
        window_flags |= ImGuiWindowFlags_NoMove;
        window_flags |= ImGuiWindowFlags_NoScrollbar;
        window_flags |= ImGuiWindowFlags_NoCollapse;
        ImVec2 windowPos = ImVec2(centerPos.x, centerPos.y);
        ImGui::SetNextWindowPos(windowPos, ImGuiSetCond_Always);
        ImGui::Begin("SelectAction", nullptr, window_flags);

        SelectAction selectAction = SELECT_ACTION_NONE;

        if (ImGui::Button("Translate", buttonSize)) {
            selectAction = SELECT_ACTION_MOVE;
        }
        ImGui::SameLine();
        ImGui::Text("T");

        if (ImGui::Button("Rotate", buttonSize)) {
            selectAction = SELECT_ACTION_ROTATE;
        }
        ImGui::SameLine();
        ImGui::Text("R");

        if (ImGui::Button("Remove", buttonSize)) {
            selectAction = SELECT_ACTION_REMOVE;
        }
		ImGui::SameLine();
        ImGui::Text("Delete");

        if (ImGui::Button("Copy", buttonSize)) {
			selectAction = SELECT_ACTION_COPY;
		}
		ImGui::SameLine();
        ImGui::Text("C");

        if (ImGui::Button("Align Horizontal", buttonSize)) {
			selectAction = SELECT_ACTION_ALIGN_HORIZONTAL;
        }
        ImGui::SameLine();
        ImGui::Text("A");

		if (ImGui::Button("Even Space H", buttonSize)) {
			selectAction = SELECT_ACTION_EVEN_SPACE_HORIZONTAL;
        }
        ImGui::SameLine();
        ImGui::Text("E");

        if (ImGui::Button("Swipe Copy", buttonSize)) {
            selectAction = SELECT_ACTION_SWIPE_COPY;
        }
        ImGui::SameLine();
		ImGui::Text("S");
		
		if (ImGui::Button("Flip", buttonSize)) {
			selectAction = SELECT_ACTION_FLIP;
		}
		ImGui::SameLine();
		ImGui::Text("F");

        ImGui::End();

        return selectAction;
    }

    static void DrawSelectedCollidersWithOffset(Context* context, ftVector2 offset) {
        for (int i = 0; i < context->selectedBodies.getSize(); ++i) {
            int bodyID = context->selectedBodies[i];
            Entity::Body* body = Entity::GetBody(&context->database, bodyID);
            ftTransform transform = body->transform;
            transform.center += offset;
			Entity::Collider* collider = body->collider;
			while (collider != nullptr) {
				ftShape* shape = collider->shape;
				Render::Color color = context->style.bodyColors[body->bodyType];
				switch (shape->shapeType) {
				case SHAPE_CIRCLE: {
					ftCircle* circle = (ftCircle*)shape;
					Render::DrawOutlineCircle(&context->renderContext,
						transform.center,
						circle->radius,
						color);
					break;
				}
				case SHAPE_POLYGON: {
					ftPolygon* polygon = (ftPolygon*)shape;
					ftVector2 vertexesInWorldSpace[128];
					for (int j = 0; j < polygon->numVertex; ++j) {
						vertexesInWorldSpace[j] = transform * polygon->vertices[j];
					}
					Render::DrawOutlinePolygon(&context->renderContext,
						vertexesInWorldSpace,
						polygon->numVertex,
						color);
					break;
				}
				}
				collider = collider->next;
			}
        }
    }

	static void DrawRuler(Context* context) {

		static const Render::Color gridColor = { 0.4F, 0.4F, 0.4F };

		ftVector2 cameraCenter = context->camera.center;
		ftVector2 halfSpan = ftVector2(context->camera.halfWidth, context->camera.halfHeight);

		ftVector2 min = cameraCenter - halfSpan;
		ftVector2 max = cameraCenter + halfSpan;
		int minX = (int) (min.x / context->gridSize);
		int minY = (int) (min.y / context->gridSize);
		int maxX = (int) (max.x / context->gridSize);
		int maxY = (int) (max.y / context->gridSize);


		for (float i = minX; i <= maxX; ++i) {
			Render::DrawLine(&context->renderContext, 
				{ i * context->gridSize, min.y }, 
				{ i * context->gridSize, max.y }, 
				context->style.gridColor);
		}

		for (float i = minY; i <= maxY; ++i) {
			Render::DrawLine(&context->renderContext, 
				{ min.x, i * context->gridSize }, 
				{ max.x, i * context->gridSize }, 
				context->style.gridColor);
		}
	}

	static void DrawBody(Context* context, 
		Entity::Body* body, 
		Render::Color color) {
		Entity::Collider* collider = body->collider;
		while (collider != nullptr) {
			ftTransform transform = body->transform * collider->transform;
			ftShape* shape = collider->shape;
			color.a = 0.6f;
			switch (shape->shapeType) {
			case SHAPE_CIRCLE: {
				ftCircle* circle = (ftCircle*)shape;
				Render::DrawSolidCircle(&context->renderContext,
					transform.center,
					circle->radius,
					color);
				break;
			}
			case SHAPE_POLYGON: {
				ftPolygon* polygon = (ftPolygon*)shape;
				ftVector2 vertexesInWorldSpace[128];
				for (int i = 0; i < polygon->numVertex; ++i) {
					vertexesInWorldSpace[i] = transform * polygon->vertices[i];
				}
				Render::DrawSolidPolygon(&context->renderContext,
					vertexesInWorldSpace,
					polygon->numVertex,
					color);
				break;
			}
			}
			collider = collider->next;
		}
		
	}

	static void AlignHorizontalSelectedBody(Context* context) {
		float yCoordinateAccum = 0;
		for (int i = 0; i < context->selectedBodies.getSize(); ++i) {
			int bodyID = context->selectedBodies[i];
			Entity::Body* body = Entity::GetBody(&context->database, bodyID);
			yCoordinateAccum += body->transform.center.y;
		}
		float targetYCoordinate = yCoordinateAccum / context->selectedBodies.getSize();

		for (int i = 0; i < context->selectedBodies.getSize(); ++i) {
			int bodyID = context->selectedBodies[i];
			Entity::Body* body = Entity::GetBody(&context->database, bodyID);
			ftTransform newTransform = body->transform;
			newTransform.center.y = targetYCoordinate;
			Entity::TransformBody(&context->database, body->bodyID, newTransform);
		}
		
	}
    
    static long int GetMillis() {
        std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
        return millis;
    }

	static void HighlightBody(Context* context, int bodyID, Render::Color color) {
		Entity::Body* body = Entity::GetBody(&context->database, bodyID);
		Entity::Collider* collider = body->collider;
		while (collider != nullptr) {
			ftTransform transform = body->transform * collider->transform;
			ftShape* shape = collider->shape;
			color.a = 0.6f;
			switch (shape->shapeType) {
			case SHAPE_CIRCLE: {
				ftCircle* circle = (ftCircle*)shape;
				Render::DrawSolidCircle(&context->renderContext,
					transform.center,
					circle->radius,
					color);
				break;
			}
			case SHAPE_POLYGON: {
				ftPolygon* polygon = (ftPolygon*)shape;
				ftVector2 vertexesInWorldSpace[128];
				for (int i = 0; i < polygon->numVertex; ++i) {
					vertexesInWorldSpace[i] = transform * polygon->vertices[i];
				}
				Render::DrawSolidPolygon(&context->renderContext,
					vertexesInWorldSpace,
					polygon->numVertex,
					color);
				break;
			}
			}
			collider = collider->next;
		}
	}

	static ftVector2 GetSelectCenter(Context* context) {
		ftVector2 center = ftVector2(0, 0);
		for (int i = 0; i < context->selectedBodies.getSize(); ++i) {
			int bodyID = context->selectedBodies[i];
			Entity::Body* body = Entity::GetBody(&context->database, bodyID);
			center += body->transform.center;
		}
		center = center / context->selectedBodies.getSize();
		return center;
	}
    

    void InitFalton(Context* context) {
		
        context->physicsSystem.setConfiguration(context->faltonConfig);
        context->physicsSystem.init();

		const auto ImportBodyToFalton = [context](Entity::Body* body) {
			ftVector2 position = body->transform.center;
			real mass = body->mass;
			ftShape* shape = body->collider->shape;
			ftMassProperty mp;
			switch (shape->shapeType) {
			case SHAPE_CIRCLE: {
				ftCircle* circle = (ftCircle*)shape;
				mp = ftMassComputer::computeForCircle(*circle, mass, ftVector2(0, 0));
				break;
			}
			case SHAPE_POLYGON: {
				ftPolygon* polygon = (ftPolygon*)shape;
				mp = ftMassComputer::computeForPolygon(*polygon, mass, ftVector2(0, 0));
				break;
			}
			}
			real moment = mp.moment;

			ftBody* faltonBody;
			switch (body->bodyType) {
			case Entity::BODY_TYPE_STATIC:
				faltonBody = context->physicsSystem.createStaticBody(position, 0);
				break;
			case Entity::BODY_TYPE_KINEMATIC:
				faltonBody = context->physicsSystem.createKinematicBody(position, 0);
				break;
			case Entity::BODY_TYPE_DYNAMIC:
				faltonBody = context->physicsSystem.createDynamicBody(position, 0, mass, moment);
				if (!body->isCustomCenterOfMass) faltonBody->centerOfMass = mp.centerOfMass;
				else faltonBody->centerOfMass = body->centerOfMass;
				break;
			}
			faltonBody->transform = body->transform;
			faltonBody->velocity = body->velocity;

			body->faltonBody = faltonBody;
			faltonBody->userdata = body;
		};
        Entity::ForEveryBody(&context->database, ImportBodyToFalton);

		const auto ImportColliderToFalton = [context](Entity::Collider* collider) {
			ftBody* body = collider->body->faltonBody;
			ftShape* shape = collider->shape;
			ftCollider* faltonCollider = context->physicsSystem.createCollider(body, 
				shape, 
				collider->transform.center, 
				collider->transform.rotation.angle);
			faltonCollider->friction = collider->friction;
			faltonCollider->restitution = collider->restitution;
			faltonCollider->group = collider->group;
			faltonCollider->category = collider->category;
			faltonCollider->mask = collider->mask;
		};
        Entity::ForEveryCollider(&context->database, ImportColliderToFalton);

		const auto ImportJointToFalton = [context](Entity::Joint* joint) {
			switch (joint->jointType) {
				case Entity::JOINT_TYPE_HINGE: {
					ftHingeJoint* faltonJoint = context->physicsSystem.createHingeJoint(joint->body[0]->faltonBody,
						joint->body[1]->faltonBody,
						joint->hinge.anchorPoint);
					faltonJoint->torqueFriction = joint->hinge.torqueFriction;
					faltonJoint->enableLimit = joint->hinge.enableLimit;
					faltonJoint->lowerLimit = joint->hinge.lowerLimit;
					faltonJoint->upperLimit = joint->hinge.upperLimit;
					break;
				}
				case Entity::JOINT_TYPE_DISTANCE: {
					ftDistanceJoint* faltonJoint = context->physicsSystem.createDistanceJoint(joint->body[0]->faltonBody,
						joint->body[1]->faltonBody,
						joint->distance.localAnchorA,
						joint->distance.localAnchorB);
					if (joint->distance.isCustomDistance) {
						faltonJoint->distance = joint->distance.distance;
					}
					break;
				}
				case Entity::JOINT_TYPE_DYNAMO: {
					ftDynamoJoint* faltonJoint = context->physicsSystem.createDynamoJoint(joint->body[0]->faltonBody,
						joint->body[1]->faltonBody,
						joint->dynamo.targetRate,
						joint->dynamo.maxTorque);
					break;
				}
				case Entity::JOINT_TYPE_SPRING: {
					ftSpringJoint* faltonJoint = context->physicsSystem.createSpringJoint(joint->body[0]->faltonBody,
						joint->body[1]->faltonBody,
						joint->spring.localAnchorA,
						joint->spring.localAnchorB);
					if (joint->spring.restLength >= 0) faltonJoint->restLength = joint->spring.restLength;
					faltonJoint->stiffness = joint->spring.stiffness;
					break;
				}
				case Entity::JOINT_TYPE_PISTON: {
					ftPistonJoint* piston = context->physicsSystem.createPistonJoint(joint->body[0]->faltonBody,
						joint->body[1]->faltonBody,
						joint->piston.localAnchorA,
						joint->piston.localAnchorB,
						joint->piston.axis);
					break;
				}

			}
		};
		Entity::ForEveryJoint(&context->database, ImportJointToFalton);

        context->faltonInfo.maxDuration = 0;
        for (int i = 0; i < FaltonInfo::NUM_FRAMES_TO_RECORD; ++i) {
            context->faltonInfo.frameTimes[i] = 0.0f;
        }
        context->faltonInfo.frameCount = 0;

    }

    ftAABB GetSelectedAABB(Context* context) {
		int bodyID = context->selectedBodies[0];
		Entity::Body* body0 = Entity::GetBody(&context->database, bodyID);
		ftAABB sumAABB = ftAABB::Create(body0->transform.center, body0->transform.center);
        for (int i = 0; i < context->selectedBodies.getSize(); ++i) {
			int bodyID = context->selectedBodies[i];
			Entity::Body* body = Entity::GetBody(&context->database, bodyID);
			Entity::Collider* collider = body->collider;
            while (collider != nullptr) {
                ftShape* shape = collider->shape;
                ftAABB aabb = shape->constructAABB(body->transform * collider->transform);
                sumAABB = ftAABB::combine(sumAABB, aabb);
                collider = collider->next;
            }
        }
        return sumAABB;
	}
	
	static void FlipSelectedBody(Context* context) {
		for (int i = 0; i < context->selectedBodies.getSize(); ++i) {
			int bodyID = context->selectedBodies[i];
			Entity::Body* body = Entity::GetBody(&context->database, bodyID);
			body->transform.center.x = -1 * body->transform.center.x;
			auto flipCollider = [](Entity::Collider* collider) {
				collider->transform.center.x = -1 * collider->transform.center.x;
				if (collider->shape->shapeType == SHAPE_POLYGON) {
					ftPolygon* polygon = (ftPolygon*) collider->shape;
					for (int j = 0; j < polygon->numVertex; ++j) {
						polygon->vertices[j].x = -1 * polygon->vertices[j].x;
					}
				} 
			};
			Entity::ForEveryColliderInBody(&context->database, bodyID, flipCollider);
		}
	}

    void Tick(Context* context, float widthPx, float heightPx) {

		Overlay::NewFrame(widthPx, heightPx);

		Camera* camera = &context->camera;

		camera->halfWidth = widthPx / (2 * context->camera.scale);
		camera->halfHeight = heightPx / (2 * context->camera.scale);

        DrawRuler(context);
        
        float projectionMat[16];
        GetProjectionMat(camera, projectionMat);
        Render::SetProjection(&context->renderContext, projectionMat);

        if (context->controlsConfig.isRunning) {
            Render::Context* renderContext = &(context->renderContext);
            Style* style = &context->style;

            const auto drawBody = [camera, renderContext, style] (ftBody* body) {
				Overlay::BodySigil(WorldToScreenSpace(camera, body->transform.center), body->transform.rotation.angle);
                Render::Color color = style->bodyColors[body->bodyType];

                auto drawCollider = [renderContext, body, color] (ftCollider* collider) {
                    switch (collider->shape->shapeType) {
                        case SHAPE_CIRCLE: {
                            ftCircle* circle = (ftCircle*) collider->shape;
                            
                            Render::DrawOutlineCircle(renderContext, 
                                                        body->transform.center, 
                                                        circle->radius, 
                                                        color);
                            break;
                        }
                        case SHAPE_POLYGON: {
                            ftPolygon* polygon = (ftPolygon*) collider->shape;
                            ftVector2* vertices = (ftVector2*) malloc(sizeof(*vertices) * polygon->numVertex);
                            for (int i = 0; i < polygon->numVertex; ++i) {
                                vertices[i] = body->transform * collider->transform * polygon->vertices[i];
                            }
                            Render::DrawOutlinePolygon(renderContext,
                                                        vertices,
                                                        polygon->numVertex,
                                                        color);
                            free(vertices);
                            break;
                        }
                    }
                };
                body->forEachCollider(drawCollider);

            };
            context->physicsSystem.forEachBody(drawBody);

            const auto drawContact = [camera] (ftContact* contact) {
                for (int i = 0; i < contact->manifold.numContact; ++i) {
					Overlay::ContactPoint(WorldToScreenSpace(camera, contact->manifold.contactPoints[i].r1));
					Overlay::ContactPoint(WorldToScreenSpace(camera, contact->manifold.contactPoints[i].r2));
                }
            };
            context->physicsSystem.forEachContact(drawContact);

			const auto drawJoint = [camera](ftJoint* joint) {
				switch (joint->jointType) {
				case ftJoint::HINGE_JOINT: {
					ftHingeJoint* hingeJoint = (ftHingeJoint*) joint;
					ftBody* bodyA = hingeJoint->bodyA;
					ftVector2 anchorScreenPos = WorldToScreenSpace(camera, bodyA->transform * hingeJoint->localAnchorA);
					Overlay::HingeSigil(anchorScreenPos);
					break;
				}
				case ftJoint::DYNAMO_JOINT: {
					ftDynamoJoint* dynamoJoint = (ftDynamoJoint*)joint;
					ftVector2 sigilPos = WorldToScreenSpace(camera, dynamoJoint->bodyB->transform.center);
					Overlay::DynamoSigil(sigilPos);
					break;
				}
				case ftJoint::DISTANCE_JOINT: {
					ftDistanceJoint* distanceJoint = (ftDistanceJoint*) joint;
					ftVector2 anchorA = joint->bodyA->transform * distanceJoint->localAnchorA;
					ftVector2 anchorB = joint->bodyB->transform * distanceJoint->localAnchorB;
					ftVector2 screenAnchorA = WorldToScreenSpace(camera, anchorA);
					ftVector2 screenAnchorB = WorldToScreenSpace(camera, anchorB);
					Overlay::DistanceSigil(screenAnchorA);
					Overlay::DistanceSigil(screenAnchorB);
					Overlay::DistanceJointLink(screenAnchorA, screenAnchorB);
					break;
				}
				case ftJoint::SPRING_JOINT: {
					ftSpringJoint* springJoint = (ftSpringJoint*)joint;
					ftVector2 anchorA = joint->bodyA->transform * springJoint->localAnchorA;
					ftVector2 anchorB = joint->bodyB->transform * springJoint->localAnchorB;
					ftVector2 screenAnchorA = WorldToScreenSpace(camera, anchorA);
					ftVector2 screenAnchorB = WorldToScreenSpace(camera, anchorB);
					Overlay::SpringSigil(screenAnchorA);
					Overlay::SpringSigil(screenAnchorB);
					Overlay::SpringJointLink(screenAnchorA, screenAnchorB, 0.1);
					break;
				}
				case ftJoint::PISTON_JOINT: {
					ftPistonJoint* pistonJoint = (ftPistonJoint*) joint;
					ftVector2 anchorPos = joint->bodyA->transform * pistonJoint->localAnchorA;
					ftVector2 screenAnchorPos = WorldToScreenSpace(camera, anchorPos);
					Overlay::PistonSigil(screenAnchorPos);
					break;
				}
				}
			};
			context->physicsSystem.forEachJoint(drawJoint);
        } 
        else {

			

			const auto DrawCollider = [context](Entity::Collider* collider) {
				Entity::Body* body = collider->body;
				ftTransform transform = body->transform * collider->transform;

				Render::Color color = context->style.bodyColors[body->bodyType];

				ftShape* shape = collider->shape;
				switch (shape->shapeType) {
				case SHAPE_CIRCLE: {
					ftCircle* circle = (ftCircle*)shape;
					Render::DrawOutlineCircle(&context->renderContext,
						transform.center,
						circle->radius,
						color);
					break;
				}
				case SHAPE_POLYGON: {
					ftPolygon* polygon = (ftPolygon*)shape;
					ftVector2 vertexesInWorldSpace[128];
					for (int i = 0; i < polygon->numVertex; ++i) {
						vertexesInWorldSpace[i] = transform * polygon->vertices[i];
					}
					Render::DrawOutlinePolygon(&context->renderContext,
						vertexesInWorldSpace,
						polygon->numVertex,
						color);
					break;
				}
				}
			};
            Entity::ForEveryCollider(&context->database, DrawCollider);

			const auto DrawBody = [camera](Entity::Body* body) {
				ftVector2 screenPos = WorldToScreenSpace(camera, body->transform.center);
				float angle = body->transform.rotation.angle;
				Overlay::BodySigil(screenPos, angle);
			};
			Entity::ForEveryBody(&context->database, DrawBody);

			const auto DrawJoint = [camera](Entity::Joint* joint) {
				switch (joint->jointType) {
				case Entity::JOINT_TYPE_HINGE: {
					ftVector2 anchorScreenPos = WorldToScreenSpace(camera, joint->hinge.anchorPoint);
					Overlay::HingeSigil(anchorScreenPos);
					break;
				}
				case Entity::JOINT_TYPE_DYNAMO: {
					Overlay::DynamoSigil(WorldToScreenSpace(camera, joint->body[1]->transform.center));
					break;
				}
				case Entity::JOINT_TYPE_DISTANCE: {
					ftVector2 anchorA = joint->body[0]->transform * joint->distance.localAnchorA;
					ftVector2 anchorB = joint->body[1]->transform * joint->distance.localAnchorB;
					ftVector2 screenAnchorA = WorldToScreenSpace(camera, anchorA);
					ftVector2 screenAnchorB = WorldToScreenSpace(camera, anchorB);
					Overlay::DistanceSigil(screenAnchorA);
					Overlay::DistanceSigil(screenAnchorB);
					Overlay::DistanceJointLink(screenAnchorA, screenAnchorB);
					break;
				}
				case Entity::JOINT_TYPE_SPRING: {
					ftVector2 anchorA = joint->body[0]->transform * joint->spring.localAnchorA;
					ftVector2 anchorB = joint->body[1]->transform * joint->spring.localAnchorB;
					ftVector2 screenAnchorA = WorldToScreenSpace(camera, anchorA);
					ftVector2 screenAnchorB = WorldToScreenSpace(camera, anchorB);
					Overlay::SpringSigil(screenAnchorA);
					Overlay::SpringSigil(screenAnchorB);
					Overlay::SpringJointLink(screenAnchorA, screenAnchorB, 0.1);
					break;
				}
				case Entity::JOINT_TYPE_PISTON: {
					ftVector2 anchor = joint->body[0]->transform * joint->piston.localAnchorA;
					ftVector2 screenAnchor = WorldToScreenSpace(camera, anchor);
					Overlay::PistonSigil(screenAnchor);
					break;
				}
				}
			};
			Entity::ForEveryJoint(&context->database, DrawJoint);
        }
        
        PhysicsConfigGUI(&(context->faltonConfig));
       
        MenuBarGUI(context);

        static bool s_IsShowToolsGui = false;
        static float s_ToolGuiX, s_ToolGuiY;
        if (ImGui::IsMouseReleased(2)) {
            s_IsShowToolsGui = true;
            s_ToolGuiX = ImGui::GetMousePos().x;
            s_ToolGuiY = ImGui::GetMousePos().y;
        }
        if (s_IsShowToolsGui) {
            if (ToolsGUI(s_ToolGuiX, s_ToolGuiY, &(context->toolsConfig))) {
                switch(context->toolsConfig.toolType) {
                case TOOL_TYPE_SELECT:
                    context->editorState = EDITOR_STATE_EDIT_BODY;
                    break;
                case TOOL_TYPE_ADD_BODY:
                    context->editorState = EDITOR_STATE_ADD_BODY;
                    break;
				case TOOL_TYPE_ADD_COLLIDER:
					context->editorState = EDITOR_STATE_ADD_COLLIDER;
					break;
				case TOOL_TYPE_EDIT_JOINT:
					context->editorState = EDITOR_STATE_EDIT_JOINT;
					break;
                case TOOL_TYPE_ADD_JOINT:
                    context->editorState = EDITOR_STATE_ADD_JOINT;
                    break;
                case TOOL_TYPE_MOVE_CAMERA:
                    context->editorState = EDITOR_STATE_MOVE_CAMERA;
                    break;
                }
                s_IsShowToolsGui = false;
            }
        }

        ControlsConfig::Event controlEvent = ControlsGUI(&(context->controlsConfig));
        switch(controlEvent) {
        case ControlsConfig::EVENT_START:
            context->editorState = EDITOR_STATE_SIMULATION_RUNNING;
            InitFalton(context);
            break;
        case ControlsConfig::EVENT_RESTART:
            context->physicsSystem.shutdown();
            InitFalton(context);
            break;
        case ControlsConfig::EVENT_STOP:
            context->editorState = EDITOR_STATE_EDIT_BODY;
            context->physicsSystem.shutdown();
            break;
        }

        if (context->controlsConfig.isPlaying) {
            auto start = std::chrono::high_resolution_clock::now();
            context->physicsSystem.step(1.0/60);
            auto finish = std::chrono::high_resolution_clock::now();
            double duration =  std::chrono::duration_cast<std::chrono::microseconds>(finish - start).count();

            int frameIdx = context->faltonInfo.frameCount % FaltonInfo::NUM_FRAMES_TO_RECORD;
            context->faltonInfo.frameTimes[frameIdx] =  duration;
            if (context->faltonInfo.maxDuration < duration) {
                context->faltonInfo.maxDuration = duration;
            }
            ++context->faltonInfo.frameCount;
        }

		ImGuiIO io = ImGui::GetIO();

		MouseInput mouseInput;
		for (int i = 0; i < 2; ++i) {
			mouseInput.isDown[i] = ImGui::IsMouseDown(i) && !ImGui::GetIO().WantCaptureMouse;
			mouseInput.isFirstDown[i] = ImGui::IsMouseClicked(i) && !ImGui::GetIO().WantCaptureMouse;
			mouseInput.isDragging[i] = ImGui::IsMouseDragging(i) && !ImGui::GetIO().WantCaptureMouse;
			mouseInput.isReleased[i] = ImGui::IsMouseReleased(i) && !ImGui::GetIO().WantCaptureMouse;
			mouseInput.clickScreenPos[i] = { io.MouseClickedPos[i].x, io.MouseClickedPos[i].y };
			mouseInput.clickWorldPos[i] = ScreenToWorldSpace(camera, mouseInput.clickScreenPos[i]);
		}
		mouseInput.screenDelta = { io.MouseDelta.x, io.MouseDelta.y };
		mouseInput.worldDelta = ScreenToWorldSpaceDir(camera, mouseInput.screenDelta);
		mouseInput.screenPos = { ImGui::GetMousePos().x, ImGui::GetMousePos().y };
		mouseInput.worldPos = ScreenToWorldSpace(camera, mouseInput.screenPos);
		mouseInput.prevScreenPos = mouseInput.screenPos - mouseInput.screenDelta;
		mouseInput.prevWorldPos = mouseInput.worldPos - mouseInput.worldDelta;
		mouseInput.wheel = io.MouseWheel;

		// Implement snap feature
		if (io.KeyCtrl) {
			if (ImGui::IsKeyDown(GLFW_KEY_B)) {
				int bodyID = -1;
				if (Entity::PointQueryBody(&context->database,
					mouseInput.worldPos,
					10 / context->camera.scale,
					&bodyID)) {

					Entity::Body* body = Entity::GetBody(&context->database, bodyID);
					ftVector2 screenPos = WorldToScreenSpace(camera, body->transform.center);
					Overlay::Snap(screenPos);
					glfwSetCursorPos(context->window, screenPos.x, screenPos.y);
					mouseInput.worldPos = body->transform.center;
				}
			}
			if (ImGui::IsKeyDown(GLFW_KEY_G)) {
				int closestLineX = round(mouseInput.worldPos.x / context->gridSize);
				int closestLineY = round(mouseInput.worldPos.y / context->gridSize);
				float diffX = mouseInput.worldPos.x - closestLineX * context->gridSize;
				float diffY = mouseInput.worldPos.y - closestLineY * context->gridSize;

				real radius = 10 / context->camera.scale;

				if (diffX * diffX + diffY *diffY < radius * radius) {
					ftVector2 screenPos = WorldToScreenSpace(camera, ftVector2(closestLineX * context->gridSize, 
						closestLineY * context->gridSize));
					Overlay::Snap(screenPos);
					glfwSetCursorPos(context->window, screenPos.x, screenPos.y);
					mouseInput.worldPos = ftVector2(closestLineX * context->gridSize,
						closestLineY * context->gridSize);
				}
			}
			if (ImGui::IsKeyDown(GLFW_KEY_V)) {
				ftVector2 result;
				if (Entity::PointQueryVertexes(&context->database,
					mouseInput.worldPos,
					10 / context->camera.scale,
					&result)) {
					ftVector2 screenPos = WorldToScreenSpace(camera, result);
					Overlay::Snap(screenPos);
					glfwSetCursorPos(context->window, screenPos.x, screenPos.y);
					mouseInput.worldPos = result;
				}
			}

			float gridScale = pow(2, mouseInput.wheel);
			if (context->gridSize * gridScale > context->camera.halfWidth / 100) {
				context->gridSize = context->gridSize * pow(2, mouseInput.wheel);
			}

			// Re-parse Input
			double screenPosX, screenPosY;
			glfwGetCursorPos(context->window, (double*) &screenPosX, &screenPosY);
			mouseInput.worldDelta = ScreenToWorldSpaceDir(camera, mouseInput.screenDelta);
			mouseInput.screenPos = { (float)screenPosX, (float)screenPosY };
			for (int i = 0; i < 3; ++i) {
				if (mouseInput.isFirstDown[i]) {
					mouseInput.clickScreenPos[i] = mouseInput.screenPos;
					mouseInput.clickWorldPos[i] = mouseInput.worldPos;
				}
			}
			
		}

		ContextMenuGUI(context, mouseInput);
		
		static Entity::Body* s_body[2];
		static int s_bodyCount = 0;
		if (s_bodyCount > 0) {
			HighlightBody(context,
				s_body[0]->bodyID,
				context->style.hightlightColors[Style::HIGHLIGHT_COLOR_JOINT_BODY_A]);
		}
		if (s_bodyCount > 1) {
			HighlightBody(context,
				s_body[1]->bodyID,
				context->style.hightlightColors[Style::HIGHLIGHT_COLOR_JOINT_BODY_B]);
		}

		if (context->editorState >= EDITOR_STATE_EDIT_BODY && 
			context->editorState <= EDITOR_STATE_EDIT_BODY_END) {

			for (int i = 0; i < context->selectedBodies.getSize(); ++i) {
				int bodyID = context->selectedBodies[i];
				Render::Color color = context->style.hightlightColors[Style::HIGHLIGHT_COLOR_SELECT_BODY];
				HighlightBody(context, bodyID, color);
				Entity::Body* body = Entity::GetBody(&context->database, bodyID);
				Entity::Collider* collider = body->collider;
				while (collider != nullptr) {
					ftTransform transform = body->transform * collider->transform;
					ftShape* shape = collider->shape;
					Render::Color color = context->style.hightlightColors[Style::HIGHLIGHT_COLOR_HOVER_BODY];
					color.a = 0.6f;
					switch (shape->shapeType) {
					case SHAPE_CIRCLE: {
						ftCircle* circle = (ftCircle*)shape;
						Render::DrawSolidCircle(&context->renderContext,
							transform.center,
							circle->radius,
							color);
						break;
					}
					case SHAPE_POLYGON: {
						ftPolygon* polygon = (ftPolygon*)shape;
						ftVector2 vertexesInWorldSpace[128];
						for (int i = 0; i < polygon->numVertex; ++i) {
							vertexesInWorldSpace[i] = transform * polygon->vertices[i];
						}
						Render::DrawSolidPolygon(&context->renderContext,
							vertexesInWorldSpace,
							polygon->numVertex,
							color);
						break;
					}
					}
					collider = collider->next;
				}
			}

			if (context->selectedBodies.getSize() > 0) {
				BodyInfoGUI(context);
			}
		}
		else if (context->editorState >= EDITOR_STATE_EDIT_JOINT &&
			context->editorState <= EDITOR_STATE_EDIT_JOINT_END) {
			
			if (context->selectedJoint != -1) {
				Entity::Joint* joint = Entity::GetJoint(&context->database, context->selectedJoint);
				Entity::Body* bodyA = joint->body[0];
				Entity::Body* bodyB = joint->body[1];
				HighlightBody(context,
					bodyA->bodyID,
					context->style.hightlightColors[Style::HIGHLIGHT_COLOR_JOINT_BODY_A]);
				HighlightBody(context,
					bodyB->bodyID,
					context->style.hightlightColors[Style::HIGHLIGHT_COLOR_JOINT_BODY_B]);
				JointInfoGUI(context, joint);
				switch(joint->jointType) {
				case Entity::JOINT_TYPE_HINGE: {
					ftVector2 anchorScreenCoord = WorldToScreenSpace(camera, joint->hinge.anchorPoint);
					ftVector2 bodyAScreenCoord = WorldToScreenSpace(camera, bodyA->transform.center);
					ftVector2 bodyBScreenCoord = WorldToScreenSpace(camera, bodyB->transform.center);
					Overlay::JointToBodyLinkA(anchorScreenCoord,
						bodyAScreenCoord);
					Overlay::JointToBodyLinkB(anchorScreenCoord,
						bodyBScreenCoord);
					
					break;
				}
				}
				if (ImGui::IsKeyReleased(GLFW_KEY_BACKSPACE) && !ImGui::GetIO().WantCaptureKeyboard) {
					Entity::DeleteJoint(&context->database, context->selectedJoint);
					context->editorState = EDITOR_STATE_EDIT_JOINT;
					context->selectedJoint = -1;
				}

			}
		}

        switch (context->editorState) {
        case EDITOR_STATE_EDIT_BODY: {

			int hoveredBodyID = -1;
			if (PointQueryBody(&context->database, mouseInput.worldPos,
				10 / context->camera.scale,
				&hoveredBodyID)) {
				HighlightBody(context, 
					hoveredBodyID, 
					context->style.hightlightColors[Style::HIGHLIGHT_COLOR_HOVER_BODY]);
			}

			if (context->selectedBodies.getSize() > 0) {
				ftAABB sumAABB = GetSelectedAABB(context);
				ftVector2 screenMin = WorldToScreenSpace(camera, sumAABB.min);
				ftVector2 screenMax = WorldToScreenSpace(camera, sumAABB.max);
				Overlay::SelectBox(screenMin, screenMax);


				ftVector2 actionMenuPos = WorldToScreenSpace(camera, sumAABB.max);
				SelectAction action = SelectActionGUI(actionMenuPos);
				switch (action) {
				case SELECT_ACTION_MOVE:
					context->editorState = EDITOR_STATE_TRANSLATE_BODY;
					break;
				case SELECT_ACTION_ROTATE:
					context->editorState = EDITOR_STATE_ROTATE_BODY;
					break;
				case SELECT_ACTION_REMOVE:
					context->editorState = EDITOR_STATE_REMOVE_BODY;
					break;
				case SELECT_ACTION_COPY:
					context->editorState = EDITOR_STATE_COPY_BODY;
					break;
				case SELECT_ACTION_ALIGN_HORIZONTAL:
					AlignHorizontalSelectedBody(context);
					break;
				case SELECT_ACTION_EVEN_SPACE_HORIZONTAL:
					//EvenSpaceHorizontalSelectedBody(context);
					break;
				case SELECT_ACTION_FLIP:
					FlipSelectedBody(context);
					break;
				case SELECT_ACTION_SWIPE_COPY:
					context->editorState = EDITOR_STATE_SWIPE_COPY;
					break;
				}
			}

            if (mouseInput.isDragging[MouseInput::LEFT_BUTTON]) {
				Overlay::SelectBox(mouseInput.clickScreenPos[MouseInput::LEFT_BUTTON],
					mouseInput.screenPos);
            }
            if (mouseInput.isReleased[MouseInput::LEFT_BUTTON]) {
                static const float selectAreaThreshold = 0.1f;
				
				if (!io.KeyShift) {
					context->selectedBodies.removeAll();
				}
				
                float diffX = fabs(io.MouseClickedPos[0].x - ImGui::GetMousePos().x);
                float diffY = fabs(io.MouseClickedPos[0].y - ImGui::GetMousePos().y);

                if (diffX > selectAreaThreshold || diffY >selectAreaThreshold) {
                    Entity::RegionQueryBodies(&context->database, 
						mouseInput.clickWorldPos[MouseInput::LEFT_BUTTON], 
						mouseInput.worldPos, 
						&context->selectedBodies);
                }
                else {
                    Entity::PointQueryBodies(&context->database, 
						mouseInput.worldPos, 
						10 / context->camera.scale,
						&context->selectedBodies);
                }

            }
            
            break;
        }
        case EDITOR_STATE_TRANSLATE_BODY: {
            static ftVector2 s_refPoint;
            static bool s_isRefActive = false;
			static bool s_isAlignX = false;
			static bool s_isAlignY = false;

			if (s_isRefActive) {
				Overlay::ReferencePoint(WorldToScreenSpace(camera, s_refPoint));
			}

			if (mouseInput.isDragging[MouseInput::RIGHT_BUTTON]) {
				Overlay::SelectBox(mouseInput.clickScreenPos[MouseInput::RIGHT_BUTTON],
					mouseInput.screenPos);
			}

            if (mouseInput.isReleased[MouseInput::RIGHT_BUTTON]) {
                ftVector2 diffScreen = mouseInput.screenPos - mouseInput.clickScreenPos[MouseInput::RIGHT_BUTTON];
                if (fabs(diffScreen.x) < 1 || fabs(diffScreen.y < 1)) {
                    int bodyID = -1;
                    if (Entity::PointQueryBody(&context->database, 
                        mouseInput.worldPos, 
						6 / context->camera.scale,
                        &bodyID)) {
                        s_isRefActive = true;
                        Entity::Body* refBody = Entity::GetBody(&context->database, bodyID);
                        s_refPoint = refBody->transform.center;
                    } 
                    else {
                        s_isRefActive = false;
                    }
				}
				else {
					ftVectorArray<int> refBodyIDs;
					refBodyIDs.init(8);
					Entity::RegionQueryBodies(&context->database,
						mouseInput.clickWorldPos[MouseInput::RIGHT_BUTTON],
						mouseInput.worldPos,
						&refBodyIDs);

					s_refPoint = ftVector2(0, 0);
					for (int i = 0; i < refBodyIDs.getSize(); ++i) {
						int refBodyID = refBodyIDs[i];
						Entity::Body* body = Entity::GetBody(&context->database, refBodyID);
						s_refPoint += body->transform.center;
					}
					s_refPoint = s_refPoint / refBodyIDs.getSize();
					s_isRefActive = true;
					refBodyIDs.cleanup();
				}
            }

			if (ImGui::IsKeyReleased(GLFW_KEY_X)) {
				s_isAlignX = !s_isAlignX;
			}
			if (ImGui::IsKeyReleased(GLFW_KEY_Y)) {
				s_isAlignY = !s_isAlignY;
			}

            if (mouseInput.isDragging[MouseInput::LEFT_BUTTON]) {
				ftVector2 selectCenter = ftVector2(0, 0);
				for (int i = 0; i < context->selectedBodies.getSize(); ++i) {
					int bodyID = context->selectedBodies[i];
					Entity::Body* body = Entity::GetBody(&context->database, bodyID);
					selectCenter += body->transform.center;
				}
				selectCenter = selectCenter / context->selectedBodies.getSize();
				if (!s_isRefActive) {
					s_refPoint = selectCenter;
				}
                ftVector2 dragDelta = {ImGui::GetIO().MouseDelta.x, ImGui::GetIO().MouseDelta.y};
                dragDelta = ScreenToWorldSpaceDir(camera, dragDelta);

				if (s_isAlignY) {
					Overlay::AlignXLine(WorldToScreenSpace(camera, s_refPoint));
					dragDelta.x = s_refPoint.x - selectCenter.x;
				}

				if (s_isAlignX) {
					Overlay::AlignYLine(WorldToScreenSpace(camera, s_refPoint));
					dragDelta.y = s_refPoint.y - selectCenter.y;
				}

                for (int i = 0; i < context->selectedBodies.getSize(); ++i) {
					int bodyID = context->selectedBodies[i];
                    Entity::Body* body = Entity::GetBody(&context->database, bodyID);
                    ftTransform newTransform = body->transform;
                    newTransform.center += dragDelta;
                    Entity::TransformBody(&context->database, body->bodyID, newTransform);
                }

            } 
            if (mouseInput.isReleased[MouseInput::LEFT_BUTTON]) {
                context->editorState = EDITOR_STATE_EDIT_BODY;
            }
            break;
        }
        case EDITOR_STATE_ROTATE_BODY: {

            static ftVector2 s_rotationCenter;
			static bool s_initCenter = false;
			if (!s_initCenter) {
				s_rotationCenter = GetSelectCenter(context);
				s_initCenter = true;
			}
			else {
				Overlay::ReferencePoint(WorldToScreenSpace(camera, s_rotationCenter));
			}

			if (mouseInput.isDragging[MouseInput::RIGHT_BUTTON]) {
				Overlay::SelectBox(mouseInput.clickScreenPos[MouseInput::RIGHT_BUTTON],
					mouseInput.screenPos);
			}

			if (mouseInput.isReleased[MouseInput::RIGHT_BUTTON]) {
				ftVector2 diffScreen = mouseInput.screenPos - mouseInput.clickScreenPos[MouseInput::RIGHT_BUTTON];
				if (fabs(diffScreen.x) < 1 || fabs(diffScreen.y < 1)) {
					s_rotationCenter = mouseInput.worldPos;
				}
				else {
					ftVectorArray<int> refBodyIDs;
					refBodyIDs.init(8);
					Entity::RegionQueryBodies(&context->database,
						mouseInput.clickWorldPos[MouseInput::RIGHT_BUTTON],
						mouseInput.worldPos,
						&refBodyIDs);

					s_rotationCenter = ftVector2(0, 0);
					for (int i = 0; i < refBodyIDs.getSize(); ++i) {
						int refBodyID = refBodyIDs[i];
						Entity::Body* body = Entity::GetBody(&context->database, refBodyID);
						s_rotationCenter += body->transform.center;
					}

					if (refBodyIDs.getSize() > 0) {
						s_rotationCenter = s_rotationCenter / refBodyIDs.getSize();
					}
					
					refBodyIDs.cleanup();
				}
			}
			
            if (mouseInput.isDragging[MouseInput::LEFT_BUTTON]) {
                
                ftRotation prevRot = ftRotation::Direction(mouseInput.prevWorldPos - s_rotationCenter);
                ftRotation curRot = ftRotation::Direction(mouseInput.worldPos - s_rotationCenter);
                real deltaAngle = curRot.angle - prevRot.angle;
                ftRotation deltaRot = ftRotation::Angle(deltaAngle);
                
                for (int i = 0;i < context->selectedBodies.getSize(); ++i) {
					int bodyID = context->selectedBodies[i];
                    Entity::Body* body = Entity::GetBody(&context->database, bodyID);
                    ftTransform newTransform = body->transform;
                    newTransform.center = deltaRot * (body->transform.center - s_rotationCenter) + s_rotationCenter;
                    newTransform.rotation = deltaRot * body->transform.rotation;
                    Entity::TransformBody(&context->database, body->bodyID, newTransform);
                }
            }
            if (mouseInput.isReleased[MouseInput::LEFT_BUTTON]) {
				s_initCenter = false;
                context->editorState = EDITOR_STATE_EDIT_BODY;
            }
            break;
        }
        case EDITOR_STATE_ADD_BODY: {
            if (context->toolsConfig.shapeType == Entity::SHAPE_TYPE_SQUARE) {
                real width = mouseInput.worldPos.x - mouseInput.clickWorldPos[MouseInput::LEFT_BUTTON].x;
                real height = mouseInput.worldPos.y - mouseInput.clickWorldPos[MouseInput::LEFT_BUTTON].y;
				if (io.KeyShift) {
					if (width * height < 0) width = -1 * height;
					else width = height;
				}
                ftVector2 topLeft = {- width / 2, height / 2};
                ftVector2 bottomRight = { width / 2, - height / 2};
				ftVector2 corner1 = mouseInput.clickWorldPos[MouseInput::LEFT_BUTTON];
				ftVector2 corner2 = corner1 + ftVector2(width, height);
				ftVector2 rectCenterPos = (corner1 + corner2) / 2;

                if (mouseInput.isDragging[MouseInput::LEFT_BUTTON]) {
					int bodyType = context->toolsConfig.bodyType;
                    Render::DrawOutlineRect(&context->renderContext, 
                                            topLeft + rectCenterPos, 
                                            bottomRight + rectCenterPos, 
                                            context->style.bodyColors[bodyType]);
                }
                 
                if (mouseInput.isReleased[MouseInput::LEFT_BUTTON]) {
                    ftPolygon* box = ftPolygon::createBox(topLeft, bottomRight);

                    int bodyID = Entity::AddBody(&context->database, 
                        ftTransform(rectCenterPos, ftRotation::Angle(0)));
                    Entity::Body* body = Entity::GetBody(&context->database, bodyID);
                    body->bodyType = context->toolsConfig.bodyType;
                    body->mass = 1;

                    int colliderID = Entity::AddCollider(&context->database, bodyID, 
                        ftTransform({0, 0}, 0), box);
                    Entity::GetCollider(&context->database, colliderID);

                }


            } 
            else if (context->toolsConfig.shapeType == Entity::SHAPE_TYPE_CIRCLE) {
				ftVector2 dragDelta = mouseInput.worldPos - mouseInput.clickWorldPos[MouseInput::LEFT_BUTTON];
                
                if (mouseInput.isDragging[MouseInput::LEFT_BUTTON]) {
                    real radius = sqrt(dragDelta.x * dragDelta.x + dragDelta.y * dragDelta.y);
                    Render::DrawOutlineCircle(&context->renderContext, 
						mouseInput.clickWorldPos[MouseInput::LEFT_BUTTON], 
						radius, 
						context->style.bodyColors[context->toolsConfig.bodyType]);
                }
                if (mouseInput.isReleased[MouseInput::LEFT_BUTTON]) {

                    real radius = sqrt(dragDelta.x * dragDelta.x + dragDelta.y * dragDelta.y);
        
                    ftCircle *shape = ftCircle::create(radius);
                    
                    int bodyID = Entity::AddBody(&context->database, 
                        ftTransform(mouseInput.clickWorldPos[MouseInput::LEFT_BUTTON], ftRotation::Direction(dragDelta)));
                    Entity::Body* body = Entity::GetBody(&context->database, bodyID);
                    body->bodyType = context->toolsConfig.bodyType;
                    body->mass = 1;

                    int colliderID = Entity::AddCollider(&context->database, bodyID, 
                        ftTransform({0,0}, 0), shape);
                    Entity::Collider* collider = Entity::GetCollider(&context->database, colliderID);
                }

            }
            else if (context->toolsConfig.shapeType == Entity::SHAPE_TYPE_POLYGON) {
                static ftVector2 s_vertexes[128];
                static int s_vertexCount = 0;
                if (mouseInput.isReleased[MouseInput::LEFT_BUTTON]) {
                    s_vertexes[s_vertexCount] = {mouseInput.worldPos.x, mouseInput.worldPos.y};
                    ++s_vertexCount;
                }
                if (mouseInput.isReleased[MouseInput::RIGHT_BUTTON]) {
                    ftPolygon *shape = ftPolygon::create(s_vertexCount, s_vertexes);
                    ftMassProperty massProperty = ftMassComputer::computeForPolygon(*shape, 1, ftVector2(0,0));

                    int bodyID = Entity::AddBody(&context->database, 
                        ftTransform(massProperty.centerOfMass, ftRotation::Angle(0)));
                    Entity::Body* body = Entity::GetBody(&context->database, bodyID);
                    body->bodyType = context->toolsConfig.bodyType;
                    body->mass = massProperty.mass;

                    for (int i = 0;i < shape->numVertex; ++i) {
                        shape->vertices[i] -= massProperty.centerOfMass;
                    }
            
                    int colliderID = Entity::AddCollider(&context->database, 
                        bodyID, ftTransform({0, 0}, 0), shape);
                    Entity::Collider* collider = Entity::GetCollider(&context->database, colliderID);
                    collider->body = body;
                    s_vertexCount = 0;
                }

                Render::DrawOutlinePolygon(&context->renderContext, 
					s_vertexes,
					s_vertexCount,
					context->style.bodyColors[context->toolsConfig.bodyType]);
            }
            break;
        }
        case EDITOR_STATE_COPY_BODY: {
            static Entity::Collider* s_refCollider = nullptr;
            if (mouseInput.isReleased[MouseInput::RIGHT_BUTTON]) {
                int refColliderID;
                if (Entity::PointQueryCollider(&context->database, mouseInput.worldPos, &refColliderID)) {
                    s_refCollider = Entity::GetCollider(&context->database, refColliderID);
                } else {
                    s_refCollider = nullptr;
                }
            }

            ftVector2 copyPos = mouseInput.worldPos;
            if (s_refCollider != nullptr) {
                Entity::Body* refBody = s_refCollider->body;
                ftVector2 refDiff = mouseInput.worldPos - refBody->transform.center;
    
                if (fabs(refDiff.x) < fabs(refDiff.y)) refDiff.x = 0;
                else refDiff.y = 0;

                copyPos = refBody->transform.center + refDiff;

                Render::DrawLine(&context->renderContext, 
					refBody->transform.center,
					copyPos, 
					context->style.bodyColors[refBody->bodyType]);
            }
            
            ftAABB aabb = GetSelectedAABB(context);
            ftVector2 selectCenter = (aabb.min + aabb.max) / 2;
            ftVector2 offset = copyPos - selectCenter;
            DrawSelectedCollidersWithOffset(context, offset);

            if (mouseInput.isReleased[MouseInput::LEFT_BUTTON]) {
                for (int i = 0; i < context->selectedBodies.getSize(); ++i) {
                    int bodyID = context->selectedBodies[i];
                    Entity::Body* body = Entity::GetBody(&context->database, bodyID);
                    ftTransform newTransform(body->transform.center + offset, body->transform.rotation);
					Entity::DeepCopyBody(&context->database, bodyID, newTransform);
                }
                s_refCollider = nullptr;
            }
            break;
        }
        case EDITOR_STATE_REMOVE_BODY: {
            for (int i = 0; i < context->selectedBodies.getSize(); ++i) {
                int bodyID = context->selectedBodies[i];
                Entity::Body* body = Entity::GetBody(&context->database, bodyID);
                Entity::DeleteBody(&context->database, body);
            }

            context->selectedBodies.removeAll();
            context->editorState = EDITOR_STATE_EDIT_BODY;
            break;
        }
        case EDITOR_STATE_SWIPE_COPY: {

            static int s_bodyCountOffset = 0;
            s_bodyCountOffset += mouseInput.wheel;

            ftAABB selectAABB = GetSelectedAABB(context);
            ftVector2 selectCenter = (selectAABB.min + selectAABB.max) / 2;
            ftVector2 diff = mouseInput.worldPos - selectCenter;
            ftVector2 selectDim = selectAABB.max - selectAABB.min;
            if (mouseInput.isDown[MouseInput::RIGHT_BUTTON]) {
                if (fabs(diff.x) < fabs(diff.y)) diff.x = 0;
                else diff.y = 0;
            }
            int bodyCount = 0;
            if (fabs(diff.x) > fabs(diff.y)) bodyCount = fabs(diff.x) / fabs(selectDim.x);
            else bodyCount = fabs(diff.y) / fabs(selectDim.y);
            bodyCount += s_bodyCountOffset;
            ftVector2 spaceDim = diff / bodyCount;

			Overlay::SwipeLine(WorldToScreenSpace(camera, selectCenter), mouseInput.screenPos);

            for (int i = 0; i < bodyCount; ++i) {
                ftVector2 offset = (i + 1) * spaceDim;
                DrawSelectedCollidersWithOffset(context, offset);
            }

            if (mouseInput.isReleased[MouseInput::LEFT_BUTTON]) {
                for (int i = 0; i < bodyCount; ++i) {
                    ftVector2 bodyDisplacement = (i + 1) * spaceDim;
                    for (int j = 0; j < context->selectedBodies.getSize(); ++j) {
						int bodyID = context->selectedBodies[j];
                        Entity::Body* body = Entity::GetBody(&context->database, bodyID);
                        ftTransform newTransform(body->transform.center + bodyDisplacement, body->transform.rotation);
                        Entity::DeepCopyBody(&context->database, bodyID, newTransform);
                    }
                }
                context->editorState = EDITOR_STATE_EDIT_BODY;
                s_bodyCountOffset = 0;
            }
            break;

        }
        case EDITOR_STATE_MOVE_CAMERA: {
            if (mouseInput.isDragging[MouseInput::LEFT_BUTTON]) {
                ftVector2 dragDelta = { ImGui::GetIO().MouseDelta.x, ImGui::GetIO().MouseDelta.y };
                dragDelta = ScreenToWorldSpaceDir(camera, dragDelta);
                context->camera.center -= dragDelta;
            }
            context->camera.scale +=  (25 * mouseInput.wheel);
            if (context->camera.scale < 25) context->camera.scale = 25;
            break;
        }
        case EDITOR_STATE_SIMULATION_RUNNING: {
            if (mouseInput.isDragging[MouseInput::LEFT_BUTTON]) {
				Overlay::SelectBox(mouseInput.clickScreenPos[MouseInput::LEFT_BUTTON],
					mouseInput.screenPos);
            }
            if (mouseInput.isReleased[MouseInput::LEFT_BUTTON]) {
                ftAABB region = ftAABB::Create(mouseInput.clickWorldPos[MouseInput::LEFT_BUTTON], mouseInput.worldPos);
                context->faltonInfo.selectedColliders.removeAll();
                context->physicsSystem.regionQuery(region, &context->faltonInfo.selectedColliders);
				ftContact* selectedContact = nullptr;
				
            }

            FaltonInfoGUI(widthPx, heightPx, &context->faltonInfo);

            break;
        }
		case EDITOR_STATE_ADD_COLLIDER: {
			static Entity::Body* s_refBody = nullptr;

			if (s_refBody != nullptr) {
				ftAABB refAABB = GetBodyAABB(s_refBody);
				Overlay::SelectBox(WorldToScreenSpace(camera, refAABB.min),
					WorldToScreenSpace(camera, refAABB.max));
			}

			switch (context->toolsConfig.shapeType) {
			case Entity::SHAPE_TYPE_CIRCLE: {

				if (mouseInput.isReleased[MouseInput::RIGHT_BUTTON]) {
					int refColliderID = -1;
					if (PointQueryCollider(&context->database, mouseInput.worldPos, &refColliderID)) {
						Entity::Collider* refCollider = Entity::GetCollider(&context->database, refColliderID);
						s_refBody = refCollider->body;
					}
					else {
						s_refBody = nullptr;
					}
				}

				if (mouseInput.isDragging[MouseInput::LEFT_BUTTON]) {
					ftVector2 dragDelta = { mouseInput.worldPos.x - mouseInput.clickWorldPos[MouseInput::LEFT_BUTTON].x, mouseInput.worldPos.y - mouseInput.clickWorldPos[MouseInput::LEFT_BUTTON].y };
					real radius = sqrt(dragDelta.x * dragDelta.x + dragDelta.y * dragDelta.y);

					Render::Color color = context->style.bodyColors[context->toolsConfig.bodyType];
					if (s_refBody != nullptr)
						color = context->style.bodyColors[s_refBody->bodyType];
					Render::DrawOutlineCircle(&context->renderContext,
						mouseInput.clickWorldPos[MouseInput::LEFT_BUTTON],
						radius,
						color);
				}

				if (mouseInput.isReleased[MouseInput::LEFT_BUTTON]) {
					ftVector2 dragDelta = { mouseInput.worldPos.x - mouseInput.clickWorldPos[MouseInput::LEFT_BUTTON].x, mouseInput.worldPos.y - mouseInput.clickWorldPos[MouseInput::LEFT_BUTTON].y };
					real radius = sqrt(dragDelta.x * dragDelta.x + dragDelta.y * dragDelta.y);

					ftCircle *shape = ftCircle::create(radius);

					if (s_refBody == nullptr) {
						int bodyID = Entity::AddBody(&context->database,
							ftTransform(mouseInput.worldPos, ftRotation::Direction(dragDelta)));
						s_refBody = Entity::GetBody(&context->database, bodyID);
						s_refBody->bodyType = context->toolsConfig.bodyType;
						s_refBody->mass = 1;
					}

					ftVector2 localPos = mouseInput.clickWorldPos[MouseInput::LEFT_BUTTON] - s_refBody->transform.center;
					localPos = s_refBody->transform.rotation.invRotate(localPos);
					real localAngle = ftRotation::Direction(dragDelta).angle - s_refBody->transform.rotation.angle;
					int colliderID = Entity::AddCollider(&context->database, s_refBody->bodyID,
						ftTransform(localPos, localAngle), shape);
					Entity::Collider* collider = Entity::GetCollider(&context->database, colliderID);
				}
				break;
			}
			case Entity::SHAPE_TYPE_SQUARE: {
				real width = mouseInput.worldPos.x - mouseInput.clickWorldPos[MouseInput::LEFT_BUTTON].x;
				real height = mouseInput.worldPos.y - mouseInput.clickWorldPos[MouseInput::LEFT_BUTTON].y;
				ftVector2 topLeft = { -width / 2, height / 2 };
				ftVector2 bottomRight = { width / 2, -height / 2 };
				ftVector2 rectCenterPos = { (mouseInput.worldPos.x + mouseInput.clickWorldPos[MouseInput::LEFT_BUTTON].x) / 2,
					(mouseInput.worldPos.y + mouseInput.clickWorldPos[MouseInput::LEFT_BUTTON].y) / 2 };


				if (mouseInput.isReleased[MouseInput::RIGHT_BUTTON]) {
					int refColliderID = -1;
					if (PointQueryCollider(&context->database, mouseInput.worldPos, &refColliderID)) {
						Entity::Collider* refCollider = Entity::GetCollider(&context->database, refColliderID);
						s_refBody = refCollider->body;
					}
					else {
						s_refBody = nullptr;
					}
				}

				if (mouseInput.isDragging[MouseInput::LEFT_BUTTON]) {

					Render::Color color = context->style.bodyColors[context->toolsConfig.bodyType];
					if (s_refBody != nullptr)
						color = context->style.bodyColors[s_refBody->bodyType];

					ftVector2 center = (mouseInput.clickWorldPos[MouseInput::LEFT_BUTTON] + mouseInput.worldPos) / 2;
					Render::DrawOutlineRect(&context->renderContext,
						topLeft + center,
						bottomRight + center,
						color);
				}

				if (mouseInput.isReleased[MouseInput::LEFT_BUTTON]) {

					ftPolygon* box = ftPolygon::createBox(topLeft, bottomRight);
					if (s_refBody == nullptr) {
						int bodyID = Entity::AddBody(&context->database,
							ftTransform(rectCenterPos, ftRotation::Angle(0)));
						s_refBody = Entity::GetBody(&context->database, bodyID);
						s_refBody->bodyType = context->toolsConfig.bodyType;
						s_refBody->mass = 1;
					}

					ftVector2 center = (mouseInput.clickWorldPos[MouseInput::LEFT_BUTTON] + mouseInput.worldPos) / 2;
					ftVector2 localPos = center - s_refBody->transform.center;
					localPos = s_refBody->transform.rotation.invRotate(localPos);
					real localAngle = -1 * s_refBody->transform.rotation.angle;
					int colliderID = Entity::AddCollider(&context->database, s_refBody->bodyID,
						ftTransform(localPos, localAngle), box);
					Entity::Collider* collider = Entity::GetCollider(&context->database, colliderID);

				}
				break;
			}
			case Entity::SHAPE_TYPE_POLYGON: {
				static ftVector2 s_vertexes[20];
				static int s_vertexCount = 0;
				if (mouseInput.isReleased[MouseInput::RIGHT_BUTTON]) {
					if (s_vertexCount == 0) {
						int refColliderID = -1;
						if (PointQueryCollider(&context->database, mouseInput.worldPos, &refColliderID)) {
							Entity::Collider* refCollider = Entity::GetCollider(&context->database, refColliderID);
							s_refBody = refCollider->body;
						}
						else {
							s_refBody = nullptr;
						}
					}
					else {
						ftPolygon *shape = ftPolygon::create(s_vertexCount, s_vertexes);
						ftMassProperty massProperty = ftMassComputer::computeForPolygon(*shape, 1, ftVector2(0, 0));

						if (s_refBody == nullptr) {
							int bodyID = Entity::AddBody(&context->database,
								ftTransform(massProperty.centerOfMass, ftRotation::Angle(0)));
							s_refBody = Entity::GetBody(&context->database, bodyID);
							s_refBody->bodyType = context->toolsConfig.bodyType;
							s_refBody->mass = 1;
						}

						for (int i = 0; i < shape->numVertex; ++i) {
							shape->vertices[i] -= massProperty.centerOfMass;
						}

						ftVector2 localPos = massProperty.centerOfMass - s_refBody->transform.center;
						localPos = s_refBody->transform.rotation.invRotate(localPos);
						real angle = -1 * s_refBody->transform.rotation.angle;

						int colliderID = Entity::AddCollider(&context->database,
							s_refBody->bodyID,
							ftTransform(localPos, angle),
							shape);
						s_vertexCount = 0;
					}
				}
				if (mouseInput.isReleased[MouseInput::LEFT_BUTTON]) {
					s_vertexes[s_vertexCount] = { mouseInput.worldPos.x, mouseInput.worldPos.y };
					++s_vertexCount;
				}
				Render::DrawOutlinePolygon(&context->renderContext,
					s_vertexes,
					s_vertexCount,
					context->style.bodyColors[context->toolsConfig.bodyType]);
				break;
			}
			}
			break;
		}
		case EDITOR_STATE_EDIT_JOINT: {
			int hoveredJointID = -1;
			if (PointQueryJoint(&context->database,
				mouseInput.worldPos,
				10 / context->camera.scale,
				&hoveredJointID)) {
				Entity::Joint* joint = Entity::GetJoint(&context->database, hoveredJointID);
				HighlightBody(context,
					joint->body[0]->bodyID,
					context->style.hightlightColors[Style::HIGHLIGHT_COLOR_JOINT_BODY_A]);
				HighlightBody(context,
					joint->body[1]->bodyID,
					context->style.hightlightColors[Style::HIGHLIGHT_COLOR_JOINT_BODY_B]);
			}

			if (mouseInput.isReleased[MouseInput::LEFT_BUTTON]) {
				if (!PointQueryJoint(&context->database,
					mouseInput.worldPos,
					10 /context->camera.scale,
					&context->selectedJoint)) {
					context->selectedJoint = -1;
				} else {
					Entity::Joint* joint = Entity::GetJoint(&context->database, context->selectedJoint);
					switch (joint->jointType) {
						case Entity::JOINT_TYPE_DISTANCE:
							context->editorState = EDITOR_STATE_EDIT_JOINT_DISTANCE;
							break;
						case Entity::JOINT_TYPE_DYNAMO:
							context->editorState = EDITOR_STATE_EDIT_JOINT_DYNAMO;
							break;
						case Entity::JOINT_TYPE_HINGE:
							context->editorState = EDITOR_STATE_EDIT_JOINT_HINGE;
							break;
						case Entity::JOINT_TYPE_PISTON:
							context->editorState = EDITOR_STATE_EDIT_JOINT_PISTON;
							break;
						case Entity::JOINT_TYPE_SPRING:
							context->editorState = EDITOR_STATE_EDIT_JOINT_SPRING;
							break;
					}
				}
			}

			break;
		}
		case EDITOR_STATE_EDIT_JOINT_HINGE: {
			if (mouseInput.isReleased[MouseInput::LEFT_BUTTON]) {

			}
			break;
		}
		case EDITOR_STATE_EDIT_JOINT_PISTON: {
			Entity::Joint* joint = Entity::GetJoint(&context->database, context->selectedJoint);
			ftVector2 screenAnchor = WorldToScreenSpace(camera, joint->body[0]->transform * joint->piston.localAnchorA);
			ftVector2 screenBodyA = WorldToScreenSpace(camera, joint->body[0]->transform.center);
			ftVector2 screenBodyB = WorldToScreenSpace(camera, joint->body[1]->transform.center);
			Overlay::PistonAxis(screenAnchor, {joint->piston.axis.x, -joint->piston.axis.y});
			Overlay::JointToBodyLinkA(screenAnchor, screenBodyA);
			Overlay::JointToBodyLinkA(screenAnchor, screenBodyB);
			if (mouseInput.isReleased[MouseInput::LEFT_BUTTON]) {

			}
			break;
		}
		case EDITOR_STATE_ADD_JOINT: {
			if (mouseInput.isReleased[MouseInput::LEFT_BUTTON]) {
				int bodyID = -1;
				if (PointQueryBody(&context->database, 
					mouseInput.worldPos, 
					10 / context->camera.scale,
					&bodyID)) {
					s_body[s_bodyCount] = Entity::GetBody(&context->database, bodyID);
					++s_bodyCount;
				}
			}
			if (s_bodyCount == 2) {
				switch (context->toolsConfig.jointType) {
				case Entity::JOINT_TYPE_HINGE:
					context->editorState = EDITOR_STATE_ADD_JOINT_HINGE;
					break;
				case Entity::JOINT_TYPE_DISTANCE:
					context->editorState = EDITOR_STATE_ADD_JOINT_DISTANCE;
					break;
				case Entity::JOINT_TYPE_DYNAMO:
					context->editorState = EDITOR_STATE_ADD_JOINT_DYNAMO;
					break;
				case Entity::JOINT_TYPE_PISTON:
					context->editorState = EDITOR_STATE_ADD_JOINT_PISTON;
					break;
				case Entity::JOINT_TYPE_SPRING:
					context->editorState = EDITOR_STATE_ADD_JOINT_SPRING;
					break;
				}
			}
			break;
		}
		case EDITOR_STATE_ADD_JOINT_HINGE: {
			if (mouseInput.isReleased[MouseInput::LEFT_BUTTON]) {
				Entity::CreateHingeJoint(&context->database,
					s_body[0]->bodyID,
					s_body[1]->bodyID,
					mouseInput.worldPos);
				s_bodyCount = 0;
				context->editorState = EDITOR_STATE_ADD_JOINT;
			}
			break;
		}
		case EDITOR_STATE_ADD_JOINT_DISTANCE: {
			static ftVector2 s_anchor[2];
			static int s_anchorCount = 0;
			if (mouseInput.isReleased[MouseInput::LEFT_BUTTON]) {
				s_anchor[s_anchorCount] = mouseInput.worldPos;
				++s_anchorCount;
			}
			if (s_anchorCount == 2) {
				ftVector2 localAnchorA = s_body[0]->transform.invTransform(s_anchor[0]);
				ftVector2 localAnchorB = s_body[1]->transform.invTransform(s_anchor[1]);
				Entity::CreateDistanceJoint(&context->database,
					s_body[0]->bodyID,
					s_body[1]->bodyID,
					localAnchorA,
					localAnchorB);
				s_anchorCount = 0;
				s_bodyCount = 0;
				context->editorState = EDITOR_STATE_ADD_JOINT;
			}
			break;
		}
		case EDITOR_STATE_ADD_JOINT_DYNAMO: {
			Entity::CreateDynamoJoint(&context->database,
				s_body[0]->bodyID,
				s_body[1]->bodyID,
				0,
				0);
			s_bodyCount = 0;
			context->editorState = EDITOR_STATE_ADD_JOINT;
			break;
		}
		case EDITOR_STATE_ADD_JOINT_PISTON: {
			if (mouseInput.isReleased[MouseInput::LEFT_BUTTON]) {
				ftVector2 localAnchorA = s_body[0]->transform.invTransform(mouseInput.worldPos);
				ftVector2 localAnchorB = s_body[1]->transform.invTransform(mouseInput.worldPos);
				Entity::CreatePistonJoint(&context->database, 
					s_body[0]->bodyID,
					s_body[1]->bodyID,
					localAnchorA,
					localAnchorB,
					ftVector2(1, 1).unit());
				s_bodyCount = 0;
				context->editorState = EDITOR_STATE_ADD_JOINT;
			}
			break;
		}
		case EDITOR_STATE_ADD_JOINT_SPRING: {
			static ftVector2 s_anchor[2];
			static int s_anchorCount = 0;
			if (mouseInput.isReleased[MouseInput::LEFT_BUTTON]) {
				s_anchor[s_anchorCount] = mouseInput.worldPos;
				++s_anchorCount;
			}
			if (s_anchorCount == 2) {
				ftVector2 localAnchorA = s_body[0]->transform.invTransform(s_anchor[0]);
				ftVector2 localAnchorB = s_body[1]->transform.invTransform(s_anchor[1]);
				int jointID = Entity::CreateSpringJoint(&context->database,
					s_body[0]->bodyID,
					s_body[1]->bodyID,
					localAnchorA,
					localAnchorB);
				Entity::Joint* joint = Entity::GetJoint(&context->database, jointID);
				joint->spring.restLength = -1;
				joint->spring.stiffness = 1;
				s_anchorCount = 0;
				s_bodyCount = 0;
				context->editorState = EDITOR_STATE_ADD_JOINT;
			}
			break;
		}
		}
		
		Render::Render(&context->renderContext);
		Overlay::Render();

    }
}