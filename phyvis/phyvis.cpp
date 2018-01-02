#include "phyvis.h"
#include <IMGUI/imgui.h>
#include <iostream>
#include <cmath>
#include <cstring>
#include <cstdio>

namespace Phyvis {

    void Camera::MoveUp() {
        center.y += halfHeight / 100;
    }
    
    void Camera::MoveDown() {
        center.y -= halfHeight / 100;
    }
    
    void Camera::MoveRight() {
        center.x += halfWidth / 100;
    }
    
    void Camera::MoveLeft() {
        center.x -= halfWidth / 100;
    }
    
    void Camera::ZoomIn() {
        halfWidth *= 0.95f;
        halfHeight *= 0.95f;
    }
    
    void Camera::ZoomOut() {
        halfWidth *= 1.05f;
        halfHeight *= 1.05f;
    }

    ftVector2 Camera::ScreenToWorldSpaceDir(ftVector2 screenDir) {
        float screenHalfWidth = halfWidth * scale;
        float screenHalfHeight = halfHeight * scale;
        
        float worldX = screenDir.x / screenHalfWidth * halfWidth;
        float worldY = - screenDir.y / screenHalfHeight * halfHeight;
        return {worldX, worldY};
    }

    ftVector2 Camera::ScreenToWorldSpace(ftVector2 screenCoord) {
        float screenHalfWidth = halfWidth * scale;
        float screenHalfHeight = halfHeight * scale;

        float worldX = (screenCoord.x / screenHalfWidth * halfWidth) + (center.x - halfWidth);
        float worldY = (center.y + halfHeight) - (screenCoord.y / screenHalfHeight * halfHeight);
        return {worldX, worldY};
    }

    ftVector2 Camera::WorldToScreenSpace(ftVector2 worldCoord) {
        float screenHalfWidth = halfWidth * scale;
        float screenHalfHeight = halfHeight * scale;

        float screenX = (worldCoord.x - (center.x - halfWidth)) * screenHalfWidth / halfWidth;
        float screenY = ((center.y + halfHeight) - worldCoord.y) * screenHalfHeight / halfHeight;
        return {screenX, screenY};
    }

    void Camera::GetProjectionMat(float* projectionMat) {
        float l = center.x - halfWidth;
        float r = center.x + halfWidth;
        float b = center.y - halfHeight;
        float t = center.y + halfHeight;

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
        style.Colors[ImGuiCol_Border]                = ImVec4(0.00f, 0.00f, 0.00f, 0.39f);
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
        style.Colors[ImGuiCol_Button]                = ImVec4(0.26f, 0.59f, 0.98f, 0.40f);
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

    void Init(Context* context) {
        Render::Init(&context->renderContext);
        Entity::Init(&context->database);
        strcpy(context->projectName, "NewProject");
        context->toolsConfig.toolType = TOOL_TYPE_SELECT;
        context->toolsConfig.shapeType = Entity::SHAPE_TYPE_CIRCLE;
        context->toolsConfig.bodyType = Entity::BODY_TYPE_DYNAMIC;
        context->controlsConfig.isRunning = false;
        context->controlsConfig.isPlaying = false;
        context->selectedColliders.init(128);

		context->physicsConfig.broadphaseType = BROADPHASE_TYPE_NSQUARED;
		context->editorState = EDITOR_STATE_SELECT_BODY;
		
		context->bodyColors[Entity::BODY_TYPE_STATIC] = { 1.0f, 0.0f, 0.0f };
		context->bodyColors[Entity::BODY_TYPE_KINEMATIC] = { 0.0f, 1.0f, 0.0f };
		context->bodyColors[Entity::BODY_TYPE_DYNAMIC] = { 0.0f, 0.0f, 1.0f };

		context->selectedFaltonColliders.init(128);

		context->camera.center = { 0, 0 };
		context->camera.scale = 100;

        SetupImGuiStyle(false, 0.4);
    }

    void Cleanup(Context* context) {
        Render::Cleanup(&context->renderContext);
        Entity::Cleanup(&context->database);;
        context->selectedColliders.cleanup();
		context->selectedFaltonColliders.cleanup();
    }

    void MenuBarGUI(Context* context) {
        
        bool isOpenProject = false;
        bool isSaveProject = false;

        if (ImGui::BeginMainMenuBar()) {
            if (ImGui::BeginMenu("File")) {
                ImGui::MenuItem("New");
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
        ControlsConfig::Event event;
        
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
        static const ImVec2 buttonSize = ImVec2(-1, 0);
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

    bool ToolsGUI(ToolsConfig* config) {

        bool isToolButtonClicked= false;

        ImGui::Begin("Tools");
        static const ImVec2 buttonSize = ImVec2(-1 , 0);

        if (ToolsButton("Select", config->toolType == TOOL_TYPE_SELECT)) {
            config->toolType = TOOL_TYPE_SELECT;
            config->selectMode = SELECT_MODE_SELECT;
            isToolButtonClicked = true;
        }
        if (ToolsButton("Add Body", config->toolType == TOOL_TYPE_ADD_BODY)) {
            config->toolType = TOOL_TYPE_ADD_BODY;
            isToolButtonClicked = true;
        }
		if (ToolsButton("Move Camera", config->toolType == TOOL_TYPE_MOVE_CAMERA)) {
			config->toolType = TOOL_TYPE_MOVE_CAMERA;
			isToolButtonClicked = true;
		}

        ImVec2 parentWindowPos = ImGui::GetWindowPos();
        ImVec2 parentWindowSize = ImGui::GetWindowSize();
        ImGui::End();

        ImVec2 childWindowPos = ImVec2(parentWindowPos.x + parentWindowSize.x + 10, parentWindowPos.y);

        ImGuiWindowFlags child_window_flags = 0;
        child_window_flags |= ImGuiWindowFlags_NoTitleBar;
        child_window_flags |= ImGuiWindowFlags_NoResize;
        child_window_flags |= ImGuiWindowFlags_NoMove;
        child_window_flags |= ImGuiWindowFlags_NoCollapse;

        if (config->toolType == TOOL_TYPE_ADD_BODY) {
            ImGui::Begin("Add Body", nullptr, child_window_flags);
            ImGui::SetWindowPos(childWindowPos);
            ImGui::Text("Body Type");
			Entity::BodyType* pBodyType = &config->bodyType;
            ImGui::RadioButton("Static", (int*) pBodyType, (int)Entity::BODY_TYPE_STATIC);
            ImGui::RadioButton("Kinematic", (int*) pBodyType, (int)Entity::BODY_TYPE_KINEMATIC);
            ImGui::RadioButton("Dynamic", (int*) pBodyType, (int)Entity::BODY_TYPE_DYNAMIC);
        
            ImGui::Text("Shape Type");
            if (ToolsButton("Circle", config->shapeType == Entity::SHAPE_TYPE_CIRCLE)) {
                config->shapeType = Entity::SHAPE_TYPE_CIRCLE;
            }
            if (ToolsButton("Square", config->shapeType == Entity::SHAPE_TYPE_SQUARE)) {
                config->shapeType = Entity::SHAPE_TYPE_SQUARE;
            }
            if (ToolsButton("Polygon", config->shapeType == Entity::SHAPE_TYPE_POLYGON)) {
                config->shapeType = Entity::SHAPE_TYPE_POLYGON;
            }
        
            ImGui::End();
        }
    
        return isToolButtonClicked;
    }

    void BodyInfoGUI(Context* context) {

        ImGui::Begin("Bodies");

        for (int i = 0; i < context->selectedColliders.getSize(); ++i) {
			int colliderID = context->selectedColliders[i];
			Entity::Collider* collider = Entity::GetCollider(&context->database, colliderID);
			Entity::Body* body = collider->body;
            char headerTitle[512];
            char headerLabel[512];
            strcpy(headerTitle, body->alias);
            sprintf(headerLabel, "###Body%d", i);
            strcat(headerTitle, headerLabel);
            if (ImGui::CollapsingHeader(headerTitle)) {
                ImGui::InputText("Alias", body->alias, sizeof(body->alias)/ sizeof(char));
                ImGui::InputFloat2("Position", (float*) &(body->transform.center));
                ImGui::InputFloat2("Velocity", (float*) &(body->velocity));
                ImGui::InputFloat2("Center of Mass", (float*) &(body->centerOfMass));
                ImGui::InputFloat("Mass", &(body->mass));
                ImGui::InputFloat("Moment", &(body->moment));

				if (ImGui::CollapsingHeader("Collider")) {
					ImGui::PushItemWidth(0.35f * ImGui::GetWindowWidth());
					ImGui::SliderFloat("##Friction", &collider->friction, 0.0f, 1.0f);
					ImGui::SameLine();
					ImGui::InputFloat("Friction", &collider->friction);

					ImGui::SliderFloat("##Restitution", &collider->restitution, 0.0f, 1.0f);
					ImGui::SameLine();
					ImGui::InputFloat("Restitution", &collider->restitution);
					
					ImGui::PopItemWidth();
				}
            }
        }

        ImGui::End();
    }

	void FaltonInfoGUI(ftVectorArray<ftCollider*>* selectedColliders) {
		ImGui::Begin("Falton Info");
		for (int i = 0; i < selectedColliders->getSize(); ++i) {
			ftCollider* faltonCollider = (*selectedColliders)[i];
			ftBody* body = faltonCollider->body;
			char headerLabel[128];
			sprintf(headerLabel, "Body%04x", body);
			if (ImGui::CollapsingHeader(headerLabel)) {
				ImGui::InputFloat2("Position", (float*) &(body->transform.center));
				ImGui::InputFloat2("Velocity", (float*) &(body->velocity));
				ImGui::InputFloat2("Center of Mass", (float*) &(body->centerOfMass));
			}
		}
		ImGui::End();
	}

    void PhysicsConfigGUI(PhysicsConfig* config) {
        ImGui::SetNextWindowSize(ImVec2(450,300), ImGuiSetCond_Once);
        ImGui::Begin("Physics Config");

        ImGui::PushItemWidth(ImGui::GetContentRegionAvailWidth() * 0.5f);
        ImGui::Text("General");
        ImGui::InputFloat("Time to sleep", &config->systemConfig.sleepTimeLimit);
        ImGui::InputFloat("Linear sleep limit", &config->systemConfig.sleepLinearLimit);
        ImGui::InputFloat("Angular sleep limit", &config->systemConfig.sleepAngularLimit);
        ImGui::SliderFloat("Sleep Ratio", &config->systemConfig.sleepRatio, 0.0f, 1.0f, "%.3f");
        ImGui::SliderFloat("Baumgarte Coefficient", &config->systemConfig.solverConfig.baumgarteCoef, 0.0f, 1.0f, "%.3f");
        ImGui::SliderFloat("Allowed Penetration", &config->systemConfig.solverConfig.allowedPenetration, 0.0f, 1.0f, "%.3f");
        ImGui::InputInt("Solver Iteration",(int*)&config->systemConfig.solverConfig.numIteration);
        ImGui::InputFloat2("Gravity",(float*)&config->systemConfig.gravity);
        ImGui::NewLine();
        ImGui::Separator();
        ImGui::Text("Broadphase");

        const char* broadphaseNames[5] = {"NSquared Broadphase", "Hierarchical Grid", "Dynamic BVH", "ToroidalGrid", "QuadTree"};

        ImGui::Combo("Broadphase", reinterpret_cast<int*>(&config->broadphaseType), broadphaseNames, sizeof(broadphaseNames)/ sizeof(char*));
		
        switch (config->broadphaseType) {
            case BROADPHASE_TYPE_HGRID : {

                ImGui::InputInt("Number Of Level", (int *)&config->hierarchicalConfig.nLevel);
                ImGui::InputFloat("Base Size", &config->hierarchicalConfig.baseSize);
                ImGui::InputFloat("Size Multiplier",&config->hierarchicalConfig.sizeMul);
                ImGui::InputInt("Number Of Bucket", (int*)&config->hierarchicalConfig.nBucket);
                break;
            }
            case BROADPHASE_TYPE_DYNAMICBVH : {
                ImGui::InputFloat("AABB Extension", &config->bvhConfig.aabbExtension);
                break;
            }
            case BROADPHASE_TYPE_TOROIDALGRID : {
                ImGui::InputFloat("Cell Size", &config->toroidalConfig.cellSize);
                ImGui::InputInt("nRow", (int*)&config->toroidalConfig.nRow);
                ImGui::InputInt("nColumn", (int*)&config->toroidalConfig.nCol);
                break;
            }
            case BROADPHASE_TYPE_QUADTREE : {
                ImGui::InputFloat2("min", (float*)&config->quadConfig.worldAABB.min);
                ImGui::InputFloat2("max", (float*)&config->quadConfig.worldAABB.max);
                ImGui::InputInt("Max Level", (int*)&config->quadConfig.maxLevel);
                break;
            }
        }

        ImGui::PopItemWidth();
        ImGui::End();
    }

    SelectAction SelectActionGUI(ftVector2 centerPos) {
        static const ImVec2 buttonSize = ImVec2(50.0 , 15.0);

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

        if (ImGui::Button("Move", buttonSize)) {
            selectAction = SELECT_ACTION_MOVE;
        }
        ImGui::SameLine();
        if (ImGui::Button("Rotate", buttonSize)) {
            selectAction = SELECT_ACTION_ROTATE;
        }
        ImGui::SameLine();
        if (ImGui::Button("Remove", buttonSize)) {
            selectAction = SELECT_ACTION_REMOVE;
        }
		ImGui::SameLine();
		if (ImGui::Button("Copy", buttonSize)) {
			selectAction = SELECT_ACTION_COPY;
		}
		ImGui::SameLine();
		if (ImGui::Button("Align Horizontal", buttonSize)) {
			selectAction = SELECT_ACTION_ALIGN_HORIZONTAL;
		}
		if (ImGui::Button("Even Space H", buttonSize)) {
			selectAction = SELECT_ACTION_EVEN_SPACE_HORIZONTAL;
		}

        ImGui::End();
        return selectAction;
    }

    static void ImportBodyToFalton(Entity::Body* body, void* data) {
        ftVector2 position = body->transform.center;
        real mass = body->mass;
        real moment = body->moment;

        Context* context = (Context*) data;

        ftBody* physicsBody;
        switch(body->bodyType) {
		case Entity::BODY_TYPE_STATIC:
                physicsBody = context->physicsSystem.createStaticBody(position, 0);
                break;
		case Entity::BODY_TYPE_KINEMATIC:
                physicsBody = context->physicsSystem.createKinematicBody(position, 0);
                break;
		case Entity::BODY_TYPE_DYNAMIC:
                physicsBody = context->physicsSystem.createDynamicBody(position, 0, mass, moment);
                physicsBody->centerOfMass = body->centerOfMass;
                break;
        }
		physicsBody->transform = body->transform;

        body->faltonBody = physicsBody;
    }

    static void ImportColliderToFalton(Entity::Collider* collider, void* data) {
        // TODO: Fix problem for mask category and group
        ftBody* body = collider->body->faltonBody;
        ftShape* shape = collider->spatialProxy->shape;
        Context* context = (Context*) data;
        ftCollider* faltonCollider = context->physicsSystem.createCollider(body, shape, ftVector2(0,0), 0);
		faltonCollider->transform = collider->transform;
		faltonCollider->friction = collider->friction;
		faltonCollider->restitution = collider->restitution;
		/*faltonCollider->group = collider->group;
		faltonCollider->category = collider->category;
		faltonCollider->mask = collider->mask;*/
    }

    static void DrawCollider(Entity::Collider* collider, void* data) {
        Context* context = (Context*) data;
        Entity::Body* body = collider->body;
        ftTransform transform = body->transform;
		Render::Color color = context->bodyColors[body->bodyType];

        ftShape* shape = collider->spatialProxy->shape;
        switch (shape->shapeType) {
            case SHAPE_CIRCLE: {
                ftCircle* circle = (ftCircle*) shape;
                Render::DrawOutlineCircle(&context->renderContext,
                                        transform.center,
                                        circle->radius,
                                        color);
                break;
            }
            case SHAPE_POLYGON: {
                ftPolygon* polygon = (ftPolygon*) shape;
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
        
    }

	static void DrawRuler(Context* context) {

		static const int gridSize = 1;
		static const Render::Color gridColor = { 0.4F, 0.4F, 0.4F };

		ftVector2 cameraCenter = context->camera.center;
		ftVector2 halfSpan = ftVector2(context->camera.halfWidth, context->camera.halfHeight);

		ftVector2 min = cameraCenter - halfSpan;
		ftVector2 max = cameraCenter + halfSpan;
		int minX = (int)min.x;
		int minY = (int)min.y;
		int maxX = (int)max.x;
		int maxY = (int)max.y;

		for (int i = minX; i <= maxX; i+=gridSize) {
			Render::DrawLine(&context->renderContext, { (float)i, min.y }, { (float)i, max.y }, { 0.4f, 0.4f, 0.4f });
		}

		for (int i = minY; i <= maxY; i+=gridSize) {
			Render::DrawLine(&context->renderContext, { min.x, (float)i }, { max.x, (float)i }, { 0.4f, 0.4f, 0.4f });
		}
	}

	static void AlignHorizontalSelectedBody(Context* context) {
		float yCoordinateAccum = 0;
		for (int i = 0; i < context->selectedColliders.getSize(); ++i) {
			int colliderID = context->selectedColliders[i];
			Entity::Collider* collider = Entity::GetCollider(&context->database, colliderID);
			Entity::Body* body = collider->body;
			yCoordinateAccum += body->transform.center.y;
		}
		float targetYCoordinate = yCoordinateAccum / context->selectedColliders.getSize();

		for (int i = 0; i < context->selectedColliders.getSize(); ++i) {
			int colliderID = context->selectedColliders[i];
			Entity::Collider* collider = Entity::GetCollider(&context->database, colliderID);
			Entity::Body* body = collider->body;
			ftTransform newTransform = body->transform;
			newTransform.center.y = targetYCoordinate;
			Entity::TransformBody(&context->database, body->bodyID, newTransform);
		}
		
	}

	static void EvenSpaceHorizontalSelectedBody(Context* context) {

		// TODO: Fix for body with multiple collider
		int collider0ID = context->selectedColliders[0];
		Entity::Collider* collider0 = Entity::GetCollider(&context->database, collider0ID);
		Entity::Body* body0 = collider0->body;

		float minX = body0->transform.center.x;
		float maxX = body0->transform.center.x;

		int numBody = context->selectedColliders.getSize();

		ftVectorArray<Entity::Body*> bodyBuffers;
		bodyBuffers.init(numBody);

		for (int i = 0; i < numBody; ++i) {
			int colliderID = context->selectedColliders[i];
			Entity::Collider* collider = Entity::GetCollider(&context->database, colliderID);
			Entity::Body* body = collider->body;
			if (minX > body->transform.center.x) {
				minX = body->transform.center.x;
			}
			if (maxX < body->transform.center.x) {
				maxX = body->transform.center.x;
			}
			bodyBuffers.push(body);
		}

		for (int i = 0; i < numBody; ++i) {
			float targetX = minX + (i  * (maxX - minX) / (numBody - 1));
			int leftMostBodyIdx = 0;
			for (int j = 0; j < bodyBuffers.getSize(); ++j) {
				Entity::Body* leftMostBody = bodyBuffers[leftMostBodyIdx];
				Entity::Body* curBody = bodyBuffers[j];
				if (leftMostBody->transform.center.x > curBody->transform.center.x) {
					leftMostBodyIdx = j;
				}
			}

			Entity::Body* leftMostBody = bodyBuffers[leftMostBodyIdx];
			ftTransform newTransform = leftMostBody->transform;
			newTransform.center.x = targetX;
			Entity::TransformBody(&context->database, leftMostBody->bodyID, newTransform);

			bodyBuffers[leftMostBodyIdx] = bodyBuffers[bodyBuffers.getSize() - 1];
			bodyBuffers.remove();
		}
	}

    void InitPhysics(Context* context) {

        switch (context->physicsConfig.broadphaseType) {
		case BROADPHASE_TYPE_NSQUARED: {
			context->broadphaseSystem = new ftNSquaredBroadphase();
			break;
		}
		case BROADPHASE_TYPE_HGRID: {
			ftHierarchicalGrid* hGrid = new ftHierarchicalGrid();
			hGrid->setConfiguration(context->physicsConfig.hierarchicalConfig);
			context->broadphaseSystem = hGrid;
			break;
		}
		case BROADPHASE_TYPE_DYNAMICBVH: {
			ftDynamicBVH* bvh = new ftDynamicBVH();
			bvh->setConfiguration(context->physicsConfig.bvhConfig);
			context->broadphaseSystem = bvh;
			break;
		}
		case BROADPHASE_TYPE_TOROIDALGRID: {
			ftToroidalGrid* toroidal = new ftToroidalGrid();
			toroidal->setConfiguration(context->physicsConfig.toroidalConfig);
			context->broadphaseSystem = toroidal;
			break;
		}
        }

        context->physicsSystem.installBroadphase(context->broadphaseSystem);
        context->physicsSystem.setConfiguration(context->physicsConfig.systemConfig);
        context->physicsSystem.init();

        Entity::ForEveryBody(&context->database, ImportBodyToFalton, (void*) context);
        Entity::ForEveryCollider(&context->database, ImportColliderToFalton, (void*) context);
    }

    ftAABB GetSelectedAABB(Context* context) {
		int colliderID = context->selectedColliders[0];
		Entity::Collider* collider0 = Entity::GetCollider(&context->database, colliderID);
		Entity::Body* body0 = collider0->body;
        ftShape* faltonShape0 = collider0->spatialProxy->shape;
        ftAABB sumAABB = faltonShape0->constructAABB(body0->transform);
        for (int i = 1; i < context->selectedColliders.getSize(); ++i) {
			int colliderID = context->selectedColliders[i];
			Entity::Collider* collider = Entity::GetCollider(&context->database, colliderID);
			Entity::Body* body = collider->body;
            ftShape* shape = collider->spatialProxy->shape;
            ftAABB aabb = shape->constructAABB(body->transform);
            sumAABB = ftAABB::combine(sumAABB, aabb);
        }
        return sumAABB;
    }

    void Tick(Context* context, float widthPx, float heightPx) {

		context->camera.halfWidth = widthPx / (2 * context->camera.scale);
		context->camera.halfHeight = heightPx / (2 * context->camera.scale);

		DrawRuler(context);

        PhysicsConfigGUI(&(context->physicsConfig));
        BodyInfoGUI(context);
        MenuBarGUI(context);
        
        if (ToolsGUI(&(context->toolsConfig))) {
            switch(context->toolsConfig.toolType) {
			case TOOL_TYPE_SELECT:
				context->editorState = EDITOR_STATE_SELECT_BODY;
				break;
			case TOOL_TYPE_ADD_BODY:
				context->editorState = EDITOR_STATE_ADD_BODY;
				break;
			case TOOL_TYPE_MOVE_CAMERA:
				context->editorState = EDITOR_STATE_MOVE_CAMERA;
				break;
            }
        }

        ControlsConfig::Event controlEvent = ControlsGUI(&(context->controlsConfig));
        switch(controlEvent) {
            case ControlsConfig::EVENT_START:
				context->editorState = EDITOR_STATE_SIMULATION_RUNNING;
                InitPhysics(context);
                break;
            case ControlsConfig::EVENT_RESTART:
                context->physicsSystem.shutdown();
                InitPhysics(context);
                break;
            case ControlsConfig::EVENT_STOP:
				context->editorState = EDITOR_STATE_SELECT_BODY;
                context->physicsSystem.shutdown();
                delete context->broadphaseSystem;
                break;
        }

        if (context->controlsConfig.isPlaying) {
            context->physicsSystem.step(1.0/60);
        }

		for (int i = 0; i < 2; ++i) {
			context->isMouseFirstDown[i] = ImGui::IsMouseClicked(i) && !ImGui::GetIO().WantCaptureMouse;
			context->isMouseDragging[i] = ImGui::IsMouseDragging(i) && !ImGui::GetIO().WantCaptureMouse;
			context->isMouseReleased[i] = ImGui::IsMouseReleased(i) && !ImGui::GetIO().WantCaptureMouse;
		}
       

        ImGuiIO io = ImGui::GetIO();
        ImVec2 clickScreenPos = io.MouseClickedPos[0];
        ImVec2 currentScreenPos = ImGui::GetMousePos();
        float mouseWheel = io.MouseWheel;

        ftVector2 clickWorldPos = context->camera.ScreenToWorldSpace({clickScreenPos.x, clickScreenPos.y});
        ftVector2 currentWorldPos = context->camera.ScreenToWorldSpace({currentScreenPos.x, currentScreenPos.y});

        switch (context->editorState) {
        case EDITOR_STATE_SELECT_BODY: {
            if (context->isMouseDragging[0]) {
                Render::DrawOutlineRect(&context->renderContext, clickWorldPos, currentWorldPos, {1.0f, 0.0f, 0.0f});
            }
            if (context->isMouseReleased[0]) {
                static const float selectAreaThreshold = 0.1f;
                context->selectedColliders.removeAll();
                
                float diffX = fabs(io.MouseClickedPos[0].x - ImGui::GetMousePos().x);
                float diffY = fabs(io.MouseClickedPos[0].y - ImGui::GetMousePos().y);

                if (diffX > selectAreaThreshold || diffY >selectAreaThreshold) {
                    Entity::RegionQueryCollider(&context->database, clickWorldPos, currentWorldPos, &context->selectedColliders);
                }
                else {
                    Entity::PointQueryCollider(&context->database, currentWorldPos, &context->selectedColliders);
                }

                
                if (context->selectedColliders.getSize() > 0)
                    context->editorState = EDITOR_STATE_SELECT_ACTION;
            }
            
            break;
        }
        case EDITOR_STATE_SELECT_ACTION: {
            ftAABB sumAABB = GetSelectedAABB(context);
            Render::DrawOutlineRect(&context->renderContext, sumAABB.min, sumAABB.max, {1.0f, 0.0f, 0.0f});

            ftVector2 actionMenuPos = context->camera.WorldToScreenSpace(sumAABB.min);
            SelectAction action = SelectActionGUI(actionMenuPos);
            switch (action) {
            case SELECT_ACTION_MOVE:
                context->editorState = EDITOR_STATE_MOVE_BODY;
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
                EvenSpaceHorizontalSelectedBody(context);
                break;
            }
            if (context->isMouseReleased[1]) {
                context->editorState = EDITOR_STATE_SELECT_BODY;
            }
            break;
        }
        case EDITOR_STATE_MOVE_BODY: {
            if (context->isMouseDragging[0]) {
                ftVector2 dragDelta = {ImGui::GetIO().MouseDelta.x, ImGui::GetIO().MouseDelta.y};
                dragDelta = context->camera.ScreenToWorldSpaceDir(dragDelta);

                for (int i = 0; i < context->selectedColliders.getSize(); ++i) {
                    int colliderID = context->selectedColliders[i];
                    Entity::Collider* collider = Entity::GetCollider(&context->database, colliderID);
                    Entity::Body* body = collider->body;
                    ftTransform newTransform = body->transform;
                    newTransform.center += dragDelta;
                    Entity::TransformBody(&context->database, body->bodyID, newTransform);
                }

            } 
            if (context->isMouseReleased[0]) {
                context->editorState = EDITOR_STATE_SELECT_ACTION;
            }
            break;
        }
        case EDITOR_STATE_ROTATE_BODY: {

            static ftVector2 rotationCenter;
            if (context->isMouseFirstDown[0]) {
                // TODO : Search for alternative solution than static
                ftAABB selectedAABB = GetSelectedAABB(context);
                rotationCenter = (selectedAABB.min + selectedAABB.max) / 2;
            }
            if (context->isMouseDragging[0]) {
                
                ftVector2 deltaMousePos = context->camera.ScreenToWorldSpaceDir({ImGui::GetIO().MouseDelta.x, ImGui::GetIO().MouseDelta.y});
                ftVector2 curMousePos = context->camera.ScreenToWorldSpace({ImGui::GetIO().MousePos.x, ImGui::GetIO().MousePos.y});
                ftVector2 prevMousePos = curMousePos - deltaMousePos;
                ftRotation prevRot = ftRotation::Direction(prevMousePos - rotationCenter);
                ftRotation curRot = ftRotation::Direction(curMousePos - rotationCenter);
                real deltaAngle = curRot.angle - prevRot.angle;
                ftRotation deltaRot = ftRotation::Angle(deltaAngle);
                
                for (int i = 0;i < context->selectedColliders.getSize(); ++i) {
                    int colliderID = context->selectedColliders[i];
                    Entity::Collider* collider = Entity::GetCollider(&context->database, colliderID);
                    Entity::Body* body = collider->body;
                    ftTransform newTransform = body->transform;
                    newTransform.center = deltaRot * (body->transform.center - rotationCenter) + rotationCenter;
                    newTransform.rotation = deltaRot * body->transform.rotation;
                    Entity::TransformBody(&context->database, body->bodyID, newTransform);
                }
            }
            if (context->isMouseReleased[0]) {
                context->editorState = EDITOR_STATE_SELECT_ACTION;
            }
            break;
        }
        case EDITOR_STATE_ADD_BODY: {
            if (context->toolsConfig.shapeType == Entity::SHAPE_TYPE_SQUARE) {
                real width = currentWorldPos.x - clickWorldPos.x;
                real height = currentWorldPos.y - clickWorldPos.y;
                ftVector2 topLeft = {- width / 2, height / 2};
                ftVector2 bottomRight = { width / 2, - height / 2};
                ftVector2 rectCenterPos = {(currentWorldPos.x + clickWorldPos.x) / 2, 
                    (currentWorldPos.y + clickWorldPos.y) / 2};

                if (context->isMouseDragging[0]) {
                    ftVector2 center = (clickWorldPos + currentWorldPos) / 2;
                    Render::DrawOutlineRect(&context->renderContext, 
                                            topLeft + center, 
                                            bottomRight + center, 
                                            {1.0f, 0.0f, 0.0f});
                } 
                if (context->isMouseReleased[0]) {

                    ftPolygon* box = ftPolygon::createBox(topLeft, bottomRight);
                    ftMassProperty mp = ftMassComputer::computeForPolygon(*box, 1, {0,0});

                    int bodyID = Entity::AddBody(&context->database, 
                        ftTransform(rectCenterPos, ftRotation::Angle(0)));
                    Entity::Body* body = Entity::GetBody(&context->database, bodyID);
                    body->bodyType = context->toolsConfig.bodyType;
                    body->mass = mp.mass;
                    body->moment = mp.moment;
                    body->centerOfMass = mp.centerOfMass;

                    int colliderID = Entity::AddCollider(&context->database, bodyID, 
                        ftTransform({0, 0}, 0), box);
                    Entity::Collider* collider = Entity::GetCollider(&context->database, colliderID);

                }
            } 
            else if (context->toolsConfig.shapeType == Entity::SHAPE_TYPE_CIRCLE) {
                
                if (context->isMouseDragging[0]) {
        
                    ftVector2 dragDelta = {currentWorldPos.x - clickWorldPos.x, currentWorldPos.y - clickWorldPos.y};
                    real radius = sqrt(dragDelta.x * dragDelta.x + dragDelta.y * dragDelta.y);
                    Render::DrawOutlineCircle(&context->renderContext, clickWorldPos, radius, {1.0f, 0.0f, 0.0f});
                }
                if (context->isMouseReleased[0]) {

                    ftVector2 dragDelta = {currentWorldPos.x - clickWorldPos.x, currentWorldPos.y - clickWorldPos.y};
                    real radius = sqrt(dragDelta.x * dragDelta.x + dragDelta.y * dragDelta.y);
        
                    ftCircle *shape = ftCircle::create(radius);
                    ftMassProperty circlemp = ftMassComputer::computeForCircle(*shape, 1, ftVector2(0, 0));

                    int bodyID = Entity::AddBody(&context->database, 
                        ftTransform(clickWorldPos, ftRotation::Direction(dragDelta)));
                    Entity::Body* body = Entity::GetBody(&context->database, bodyID);
                    body->bodyType = context->toolsConfig.bodyType;
                    body->mass = circlemp.mass;
                    body->moment = circlemp.moment;
                    body->centerOfMass = circlemp.centerOfMass;

                    int colliderID = Entity::AddCollider(&context->database, bodyID, 
                        ftTransform({0,0}, 0), shape);
                    Entity::Collider* collider = Entity::GetCollider(&context->database, colliderID);
                }

            }
            else if (context->toolsConfig.shapeType == Entity::SHAPE_TYPE_POLYGON) {
                static ftVector2 polygonBuffer[128] = {};
                static int polygonVertexCount = 0;
                if (context->isMouseReleased[0]) {
                    polygonBuffer[polygonVertexCount] = {currentWorldPos.x, currentWorldPos.y};
                    ++polygonVertexCount;
                }
                if (context->isMouseReleased[1]) {
                    ftPolygon *shape = ftPolygon::create(polygonVertexCount, polygonBuffer);
                    ftMassProperty massProperty = ftMassComputer::computeForPolygon(*shape, 1, ftVector2(0,0));

                    int bodyID = Entity::AddBody(&context->database, 
                        ftTransform(massProperty.centerOfMass, ftRotation::Angle(0)));
                    Entity::Body* body = Entity::GetBody(&context->database, bodyID);
                    body->bodyType = context->toolsConfig.bodyType;
                    body->mass = massProperty.mass;
                    body->moment = massProperty.moment;
                    body->centerOfMass = ftVector2(0,0);

                    for (int i = 0;i < shape->numVertex; ++i) {
                        shape->vertices[i] -= massProperty.centerOfMass;
                    }
            
                    int colliderID = Entity::AddCollider(&context->database, 
                        bodyID, ftTransform({0, 0}, 0), shape);
                    Entity::Collider* collider = Entity::GetCollider(&context->database, colliderID);
                    collider->body = body;
                    polygonVertexCount = 0;
                }

                Render::DrawOutlinePolygon(&context->renderContext, 
                                            polygonBuffer, 
                                            polygonVertexCount, 
                                            {1.0f, 0.0f, 0.0f});
            }
            break;
        }
        case EDITOR_STATE_COPY_BODY: {
            context->mousePos = currentWorldPos;
            ftAABB aabb = GetSelectedAABB(context);
            ftVector2 selectCenter = (aabb.min + aabb.max) / 2;
            ftVector2 bodyDisplacement = currentWorldPos - selectCenter;
            for (int i = 0; i < context->selectedColliders.getSize(); ++i) {
                int colliderID = context->selectedColliders[i];
                Entity::Collider* collider = Entity::GetCollider(&context->database, colliderID);
                Entity::Body* body = collider->body;
                ftTransform transform = body->transform;
                transform.center += bodyDisplacement;
                ftShape* shape = collider->spatialProxy->shape;
                Render::Color color = context->bodyColors[body->bodyType];
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
            }
            if (context->isMouseReleased[0]) {
                for (int i = 0; i < context->selectedColliders.getSize(); ++i) {
                    int colliderID = context->selectedColliders[i];
                    Entity::Collider* collider = Entity::GetCollider(&context->database, colliderID);
                    Entity::Body* body = collider->body;
                    ftShape* shape = collider->spatialProxy->shape;

                    ftShape* newShape;
                    if (shape->shapeType == SHAPE_CIRCLE) {
                        ftCircle* circle = (ftCircle*)shape;
                        newShape = ftCircle::create(circle->radius);
                    } 
                    else if (shape->shapeType == SHAPE_POLYGON) {
                        ftPolygon* polygon = (ftPolygon*)shape;
                        newShape = ftPolygon::create(polygon->numVertex, polygon->vertices);
                    }
                    
                    ftTransform newTransform(body->transform.center + bodyDisplacement, body->transform.rotation);
                    int newBodyID = Entity::AddBody(&context->database, newTransform);
                    Entity::Body* newBody = Entity::GetBody(&context->database, newBodyID);
                    newBody->bodyType = body->bodyType;
                    newBody->centerOfMass = body->centerOfMass;
                    newBody->mass = body->mass;
                    newBody->moment = body->moment;
                    newBody->velocity = body->velocity;

                    int newColliderID = Entity::AddCollider(&context->database, newBodyID, collider->transform, newShape);
                    Entity::Collider* newCollider = Entity::GetCollider(&context->database, newColliderID);
                    newCollider->category = collider->category;
                    newCollider->friction = collider->friction;
                    newCollider->group = collider->group;
                    newCollider->mask = collider->mask;
                    newCollider->restitution = collider->restitution;

                }
            }
            if (context->isMouseReleased[1]) {
                context->editorState = EDITOR_STATE_SELECT_BODY;
                if (context->isMouseDragging[0]) {
                    Render::DrawOutlineRect(&context->renderContext, clickWorldPos, currentWorldPos, { 1.0f, 0.0f, 0.0f });
                }
                if (context->isMouseReleased[0]) {
                    context->selectedColliders.removeAll();
                    Entity::RegionQueryCollider(&context->database, clickWorldPos, currentWorldPos, &context->selectedColliders);
                    if (context->selectedColliders.getSize() > 0)
                        context->editorState = EDITOR_STATE_SELECT_ACTION;
                }

                break;
            }
            break;
        }
        case EDITOR_STATE_REMOVE_BODY: {
            for (int i = 0; i < context->selectedColliders.getSize(); ++i) {
                // TODO: Fix for bodies with multiple colliders. Body might be deleted twice
                int colliderID = context->selectedColliders[i];
                Entity::Collider* collider = Entity::GetCollider(&context->database, colliderID);
                Entity::Body* body = collider->body;
                Entity::DeleteBody(&context->database, body);
            }

            context->selectedColliders.removeAll();
            context->editorState = EDITOR_STATE_SELECT_BODY;
            break;
        }
        case EDITOR_STATE_MOVE_CAMERA: {
            if (context->isMouseDragging[0]) {
                ftVector2 dragDelta = { ImGui::GetIO().MouseDelta.x, ImGui::GetIO().MouseDelta.y };
                dragDelta = context->camera.ScreenToWorldSpaceDir(dragDelta);
                context->camera.center -= dragDelta;
            }
            context->camera.scale +=  (25 * mouseWheel);
            if (context->camera.scale < 25) context->camera.scale = 25;
            break;
        }
        case EDITOR_STATE_SIMULATION_RUNNING: {
            if (context->isMouseDragging[0]) {
                Render::DrawOutlineRect(&context->renderContext, clickWorldPos, currentWorldPos, { 1.0f, 0.0f, 0.0f });
            }
            if (context->isMouseReleased[0]) {
                ftAABB region = ftAABB::Create(clickWorldPos, currentWorldPos);
                context->selectedFaltonColliders.removeAll();
                context->physicsSystem.regionQuery(region, &context->selectedFaltonColliders);
            }

            FaltonInfoGUI(&context->selectedFaltonColliders);

            break;
        }

        }
        
        float projectionMat[16];
        context->camera.GetProjectionMat(projectionMat);
        Render::SetProjection(&context->renderContext, projectionMat);

        if (context->controlsConfig.isRunning) {
            Render::Context* renderContext = &(context->renderContext);

            const auto drawBody = [renderContext] (ftBody* body) {
                Render::Color color;
                switch(body->bodyType) {
                    case STATIC: {
                        color = {1.0f, 0.0f, 0.0f};
                        break;
                    }
                    case KINEMATIC: {
                        color = {0.0f, 1.0f, 0.0f};
                        break;
                    }
                    case DYNAMIC: {
                        color = {0.0f, 0.0f, 1.0f};
                        break;
                    }
                }

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
                                vertices[i] = body->transform * polygon->vertices[i];
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
            float cameraScale = context->camera.scale;
            const auto drawContact = [renderContext, cameraScale] (ftContact* contact) {
                for (int i = 0; i < contact->manifold.numContact; ++i) {
                    ftVector2 p = 0.5f * (contact->manifold.contactPoints[0].r1 + contact->manifold.contactPoints[0].r2);
                    Render::DrawSolidCircle(renderContext, p, 5.0f / cameraScale, {0.3f, 0.8f, 0.3f});
                }
            };

            context->physicsSystem.forEachContact(drawContact);
        } 
        else {
            ForEveryCollider(&context->database, DrawCollider, context);
        }
        
        Render::Render(&context->renderContext);

    }
}