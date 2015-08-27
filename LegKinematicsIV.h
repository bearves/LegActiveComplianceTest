#ifndef LEG_KINEMATICS_IV_H
#define LEG_KINEMATICS_IV_H

namespace Model
{
class Leg
{
    public:
        enum LEG_IDENTIFIER
        {
            LEG_ID_RF = 0,
            LEG_ID_MF = 1,
            LEG_ID_LF = 2,
            LEG_ID_RB = 3,
            LEG_ID_MB = 4,
            LEG_ID_LB = 5
        };
        Leg();
        Leg(int legID);
        ~Leg();

        // assign the leg ID w.r.t. leg direction
        void SetID(int legID);

        // Inverse kinematics
        int InverseSolution(double* tipPosition, double* jointLength, bool requireTransformToHip);

        // Transform the tip position w.r.t. Body Direction to the Leg Hip Direction
        int TransformTipPositionToHip(double* tipPositionRaw, double* relativeTipPositionWithHip, bool requireTransformToHip);

    private:
        int m_legID;
        double m_tempPosition[3];
};
}

#endif
