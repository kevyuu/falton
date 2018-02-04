#include "phyvis_overlay.h"
#include "phyvis_render.h"
#include <chrono>
namespace Phyvis {
    namespace Overlay {

		Render::Context g_renderContext;
		
        void Init() {
            Render::Init(&g_renderContext);
            float projection[16] = {
                1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 1
            };
            Render::SetProjection(&g_renderContext, projection);
        }

        void NewFrame(int width, int height) {
            float projectionMat[16] = {
                2.0f / width, 0, 0, 0,
                0 , -2.0f / height, 0, 0,
                0, 0, 0, 0,
                -1.0f, 1.0f, 0, 1
            };
            Render::SetProjection(&g_renderContext, projectionMat);
        }

        static long int GetMillis() {
            std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
            auto duration = now.time_since_epoch();
            auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
            return millis;
        }

        void Snap(ftVector2 point) {
            long int millis = GetMillis();
            real angle = (millis * 3.14 / 10000);
            Render::PushTransform(&g_renderContext, ftTransform(point, angle));

            float scale = 800;
            for (int i = 0; i < 4; ++i) {
                float v0 = 0.013 * sin((float)i * 3.14 * 0.5 - 0.4);
                float v1 = 0.013 * cos((float)i * 3.14 * 0.5 - 0.4);
                float v2 = 0.045 * sin((float)i * 3.14 * 0.5 - 0.05);
                float v3 = 0.045 * cos((float)i * 3.14 * 0.5 - 0.05);
                float v4 = 0.045 * sin((float)i * 3.14 * 0.5 + 0.05);
                float v5 = 0.045 * cos((float)i * 3.14 * 0.5 + 0.05);
                float v6 = 0.013 * sin((float)i * 3.14 * 0.5 + 0.4);
                float v7 = 0.013 * cos((float)i * 3.14 * 0.5 + 0.4);
                
                Render::DrawOutlineQuad(
                    &g_renderContext,
                    ftVector2(v0, v1) * scale,
                    ftVector2(v2, v3) * scale,
                    ftVector2(v4, v5) * scale,
                    ftVector2(v6, v7) * scale,
                    Overlay::g_Style.colors[Overlay::COLOR_ID_SNAP]
                );
            }

            Render::PopTransform(&g_renderContext);
        }

		void SwipeLine(ftVector2 origin, ftVector2 dest) {
			Render::DrawDashedLine(&g_renderContext,
				origin,
				dest,
				g_Style.colors[COLOR_ID_SWIPE_COPY_LINE]);
		}

		void AlignXLine(ftVector2 point) {
			Render::DrawLine(&g_renderContext,
				point - ftVector2(0, 1000),
				point + ftVector2(0, 1000),
				g_Style.colors[COLOR_ID_REF_X_AXIS]);
		}

		void AlignYLine(ftVector2 point)
		{
			Render::DrawLine(&g_renderContext,
				point - ftVector2(2000, 0),
				point + ftVector2(2000, 0),
				g_Style.colors[COLOR_ID_REF_Y_AXIS]);
		}

		void BodySigil(ftVector2 point, float angle)
		{
			int axisLength = 16;
		
			Render::PushTransform(&g_renderContext, ftTransform(point, -1 * angle));
			Render::DrawLine(&g_renderContext,
				ftVector2(0, 0),
				ftVector2(axisLength, 0),
				g_Style.colors[Overlay::COLOR_ID_BODY_X_AXIS]);

			Render::DrawLine(&g_renderContext,
				ftVector2(0, 0),
				ftVector2(0, -1 * axisLength),
				g_Style.colors[Overlay::COLOR_ID_BODY_Y_AXIS]);

			Render::DrawSolidRect(&g_renderContext,
				ftVector2(4, 4),
				ftVector2(-4, -4),
				{ 0.8f, 0.8f, 0.8f, 1.0f });
			Render::PopTransform(&g_renderContext);

		}

		void ContactPoint(ftVector2 point)
		{
			Render::DrawSolidCircle(&g_renderContext,
				point,
				5,
				g_Style.colors[COLOR_ID_CONTACT]);
		}

		void HingeSigil(ftVector2 point)
		{
			Render::DrawOutlineCircle(&g_renderContext,
				point,
				15,
				g_Style.colors[COLOR_ID_JOINT_SIGIL]);

			Render::DrawSolidCircle(&g_renderContext,
				point,
				1,
				g_Style.colors[COLOR_ID_JOINT_SIGIL]);

			long int millis = GetMillis();
			real angle = (millis * 3.14 / 10000);
			Render::PushTransform(&g_renderContext, ftTransform(point, angle));

			Render::DrawOutlineRect(&g_renderContext,
				ftVector2(-1.5f, -9),
				ftVector2(1.5f, 9),
				g_Style.colors[COLOR_ID_JOINT_SIGIL]);

			Render::DrawOutlineRect(&g_renderContext,
				ftVector2(3, 3),
				ftVector2(10.5, 6),
				g_Style.colors[COLOR_ID_JOINT_SIGIL]);

			Render::DrawOutlineRect(&g_renderContext,
				ftVector2(-3, -3),
				ftVector2(-10.5, -6),
				g_Style.colors[COLOR_ID_JOINT_SIGIL]);

			Render::PopTransform(&g_renderContext);
		}

		void DistanceSigil(ftVector2 point)
		{
			long int millis = GetMillis();
			real angle = (millis * 3.14 / 5000);

			Render::PushTransform(&g_renderContext, ftTransform(point, angle));

			Render::DrawOutlineCircle(&g_renderContext,
				ftVector2(0, 0),
				20,
				Overlay::g_Style.colors[COLOR_ID_JOINT_SIGIL]);
			Render::DrawSolidCircle(&g_renderContext,
				ftVector2(6, 0),
				2,
				Overlay::g_Style.colors[COLOR_ID_JOINT_SIGIL]);

			Render::DrawSolidCircle(&g_renderContext,
				ftVector2(-6, 0),
				2,
				Overlay::g_Style.colors[COLOR_ID_JOINT_SIGIL]);

			Render::DrawSolidRect(&g_renderContext,
				ftVector2(-3, 0.5),
				ftVector2(3, 0.5),
				Overlay::g_Style.colors[COLOR_ID_JOINT_SIGIL]);
			Render::PopTransform(&g_renderContext);

		}

		void DynamoSigil(ftVector2 point) {
			long int millis = GetMillis();
			real angle = (millis * 3.14 / 5000);

			Render::PushTransform(&g_renderContext, ftTransform(point, angle));
			Render::DrawOutlineCircle(&g_renderContext,
				ftVector2(0, 0),
				20,
				g_Style.colors[COLOR_ID_JOINT_SIGIL]);

			Render::DrawOutlineCircle(&g_renderContext,
				ftVector2(0, 0),
				10,
				g_Style.colors[COLOR_ID_JOINT_SIGIL]);

			Render::DrawOutlineCircle(&g_renderContext,
				ftVector2(0, 0),
				4,
				g_Style.colors[COLOR_ID_JOINT_SIGIL]);

			for (int i = 0; i < 8; ++i) {
				float sinAngle = sin(2 * 3.14 * i / 8);
				float cosAngle = cos(2 * 3.14 * i / 8);
				Render::DrawLine(&g_renderContext,
					ftVector2(sinAngle * 4, cosAngle * 4),
					ftVector2(sinAngle * 10, cosAngle * 10),
					g_Style.colors[COLOR_ID_JOINT_SIGIL]);
			}

			Render::PopTransform(&g_renderContext);
		}

		void SpringSigil(ftVector2 point)
		{
			long int millis = GetMillis();
			real angle = (millis * 3.14 / 5000);

			Render::PushTransform(&g_renderContext, ftTransform(point, angle));
			Render::DrawOutlineCircle(&g_renderContext,
				ftVector2(0, 0),
				20,
				g_Style.colors[COLOR_ID_JOINT_SIGIL]);

			Render::DrawSolidRect(&g_renderContext,
				ftVector2(-6, -1),
				ftVector2(6, 1),
				g_Style.colors[COLOR_ID_JOINT_SIGIL]);

			Render::DrawSolidRect(&g_renderContext,
				ftVector2(-6, 3),
				ftVector2(6, 5),
				g_Style.colors[COLOR_ID_JOINT_SIGIL]);

			Render::DrawSolidRect(&g_renderContext,
				ftVector2(-6, -3),
				ftVector2(6, -5),
				g_Style.colors[COLOR_ID_JOINT_SIGIL]);
			Render::PopTransform(&g_renderContext);
		}

		void PistonSigil(ftVector2 point)
		{
			long int millis = GetMillis();
			real angle = (millis * 3.14 / 5000);

			Render::PushTransform(&g_renderContext, ftTransform(point, angle));
			Render::DrawOutlineCircle(&g_renderContext,
				ftVector2(0, 0),
				20,
				g_Style.colors[COLOR_ID_JOINT_SIGIL]);

			Render::DrawSolidRect(&g_renderContext,
				ftVector2(-6, -1),
				ftVector2(6, 1),
				g_Style.colors[COLOR_ID_JOINT_SIGIL]);

			Render::DrawSolidRect(&g_renderContext,
				ftVector2(-6, 3),
				ftVector2(6, 5),
				g_Style.colors[COLOR_ID_JOINT_SIGIL]);

			Render::DrawSolidRect(&g_renderContext,
				ftVector2(-6, -3),
				ftVector2(6, -5),
				g_Style.colors[COLOR_ID_JOINT_SIGIL]);
			Render::PopTransform(&g_renderContext);
		}

		void PistonAxis(ftVector2 point, ftVector2 direction) {

			Render::PushTransform(&g_renderContext, ftTransform(point, ftRotation::Direction(direction)));
			Render::DrawDashedLine(&g_renderContext,
				ftVector2(0, 0),
				ftVector2(1, 0) * 100,
				10,
				{1.0f, 1.0f, 1.0f, 1.0f});
			Render::DrawOutlineTriangle(&g_renderContext,
				ftVector2(100, 0),
				ftVector2(90, 5),
				ftVector2(90, -5),
				{1.0f, 1.0f, 1.0f, 1.0f});
			Render::PopTransform(&g_renderContext);
		}

		void SelectBox(ftVector2 corner1, ftVector2 corner2)
		{
			Render::DrawDashedRect(&g_renderContext,
				corner1,
				corner2,
				10,
				g_Style.colors[COLOR_ID_SELECTION]);
		}

		void ReferencePoint(ftVector2 point)
		{
			Render::DrawLine(&g_renderContext,
				point - ftVector2(6, 0),
				point + ftVector2(6, 0),
				g_Style.colors[COLOR_ID_REF_POINT]);
			Render::DrawLine(&g_renderContext,
				point - ftVector2(0, 6),
				point + ftVector2(0, 6),
				g_Style.colors[COLOR_ID_REF_POINT]);
		}

		void JointToBodyLinkA(ftVector2 jointPos, ftVector2 bodyPos)
		{
			Render::DrawDashedLine(&g_renderContext,
				jointPos,
				bodyPos,
				g_Style.colors[COLOR_ID_JOINT_LINK_A]);
		}

		void JointToBodyLinkB(ftVector2 jointPos, ftVector2 bodyPos)
		{
			Render::DrawDashedLine(&g_renderContext,
				jointPos,
				bodyPos,
				g_Style.colors[COLOR_ID_JOINT_LINK_B]);
		}

		void DistanceJointLink(ftVector2 pointA, ftVector2 pointB)
		{
			ftVector2 localPointB = pointB - pointA;
			localPointB = ftRotation::Direction(pointB - pointA).invRotate(localPointB);
			Render::PushTransform(&g_renderContext, ftTransform(pointA, ftRotation::Direction(pointB - pointA)));
			Render::DrawLine(&g_renderContext,
				ftVector2(20, 4),
				ftVector2(localPointB.x - 20, 4),
				{ 1.0f, 1.0f, 1.0f, 1.0f }
			);
			Render::DrawLine(&g_renderContext,
				ftVector2(20, -4),
				ftVector2(localPointB.x - 20, -4),
				{ 1.0f, 1.0f, 1.0f, 1.0f }
			);
			Render::PopTransform(&g_renderContext);
		}



		void SpringJointLink(ftVector2 pointA, ftVector2 pointB, real stiffness)
		{
			int waveNum = 20 * ( 1 - stiffness);
			
			ftVector2 localPointB = pointB - pointA;
			localPointB = ftRotation::Direction(pointB - pointA).invRotate(localPointB);
			ftTransform transform = ftTransform(pointA, ftRotation::Direction(pointB - pointA));
			Render::PushTransform(&g_renderContext, ftTransform(pointA, ftRotation::Direction(pointB - pointA)));
			ftVector2 waveDim = (localPointB - ftVector2(40, 0)) / (waveNum + 1);
			Render::DrawLine(&g_renderContext,
				ftVector2(20, 0),
				(waveDim / 4) + ftVector2(20, 0),
				{ 0.8f, 0.8f, 0.8f, 0.8f });
			Render::DrawLine(&g_renderContext,
				waveDim / 4 + ftVector2(20, 0),
				ftVector2((waveDim.x * 0.5) + 20, 8.0f),
				{ 1.0f, 1.0f,1.0f, 1.0f });

			for (int i = 0; i < waveNum; ++i) {
				ftVector2 peakPos1 = { waveDim.x * (i + 0.5f) + 20, 8.0f };
				ftVector2 peakPos2 = { waveDim.x * (i + 1.5f) + 20, 8.0f };
				ftVector2 bottomPos = { waveDim.x * (i + 1.0f) + 20, -8.0f };
				Render::DrawLine(&g_renderContext,
					peakPos1,
					bottomPos,
					{ 1.0f, 1.0f, 1.0f, 1.0f }
				);
				Render::DrawLine(&g_renderContext,
					bottomPos,
					peakPos2,
					{ 1.0f, 1.0f, 1.0f, 1.0f }
				);
			}

			Render::DrawLine(&g_renderContext, 
				localPointB - ftVector2(20, 0),
				localPointB - waveDim / 4 - ftVector2(20, 0),
				{ 1.0f, 1.0f, 1.0f, 1.0f }
			);
			Render::DrawLine(&g_renderContext,
				localPointB - waveDim / 4 - ftVector2(20, 0),
				ftVector2(waveDim.x * (waveNum + 0.5f) + 20 , 8.0f),
				{ 1.0f, 1.0f,1.0f, 1.0f });

			Render::PopTransform(&g_renderContext);
		}

		void Render() {
            Render::Render(&g_renderContext);
        }

        void Cleanup() {
            Render::Cleanup(&g_renderContext);
        }
    }
}