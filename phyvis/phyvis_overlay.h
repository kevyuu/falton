#pragma once

namespace Phyvis {
    namespace Overlay {

        enum ColorID {
            COLOR_ID_SELECTION,
            COLOR_ID_BODY_X_AXIS,
            COLOR_ID_BODY_Y_AXIS,
            COLOR_ID_SNAP,
            COLOR_ID_SWIPE_COPY_LINE,
            COLOR_ID_CONTACT,
            COLOR_ID_REF_X_AXIS,
            COLOR_ID_REF_Y_AXIS,
            COLOR_ID_REF_POINT,
            COLOR_ID_JOINT_SIGIL,
            COLOR_ID_JOINT_LINK_A,
            COLOR_ID_JOINT_LINK_B,
            COLOR_ID_JOINT_JOINT_LINK,
            COLOR_ID_TOTAL,
        };

        struct Style {
            Render::Color colors[COLOR_ID_TOTAL];
        } g_Style;

        void Init();
        void NewFrame(int width, int height);
        void Snap(ftVector2 point);
		void SelectBox(ftVector2 corner1, ftVector2 corner2);
		void ReferencePoint(ftVector2 point);
		void AlignXLine(ftVector2 point);
		void AlignYLine(ftVector2 point);
		void SwipeLine(ftVector2 origin, ftVector2 dest);
		void BodySigil(ftVector2 point, float angle);
		void ContactPoint(ftVector2 point);
		void HingeSigil(ftVector2 point);
		void DistanceSigil(ftVector2 point);
		void DynamoSigil(ftVector2 point);
        void SpringSigil(ftVector2 point);
        void PistonSigil(ftVector2 point);
		void JointToBodyLinkA(ftVector2 jointPos, ftVector2 bodyPos);
		void JointToBodyLinkB(ftVector2 jointPos, ftVector2 bodyPos);
		void DistanceJointLink(ftVector2 pointA, ftVector2 pointB);
        void SpringJointLink(ftVector2 pointA, ftVector2 pointB, real stiffness);
        void PistonAxis(ftVector2 point, ftVector2 axis);
        void Render();
        void Cleanup();
    }
}