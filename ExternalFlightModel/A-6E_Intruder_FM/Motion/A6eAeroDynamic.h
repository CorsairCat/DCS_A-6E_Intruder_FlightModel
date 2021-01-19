#ifndef _A6EAERO_H_
#define _A6EAERO_H_

#include "../A6E_FM_Utility.h"
#include "A6eConstantData.h"

// use for calculate aerodynamic and aero control
// Important: this part not include the gravity, gravity is controlled by game core;
// control and force are supplied here
// atmosphere data will also transfer here
class A6eAeroDynamic
{
private:
    double getClfromAOA(double correctAOA) // correct aoa is define by common aoa + install angle need in range [-180 180]
    {
        int tempid = 0;
        double Cl = 0;
        double ClMultiplier = 1;
        if (correctAOA >= 0)
        {
            tempid = (int)correctAOA;
            Cl = A6EBaseAeroData::ClofWing[tempid] + (A6EBaseAeroData::ClofWing[tempid + 1] - A6EBaseAeroData::ClofWing[tempid]) * (correctAOA - tempid);
        }
        else
        {
            tempid = - (int)correctAOA;
            Cl = - (A6EBaseAeroData::ClofWing[tempid] + (A6EBaseAeroData::ClofWing[tempid + 1] - A6EBaseAeroData::ClofWing[tempid]) * (- correctAOA - tempid));
        }
        tempid = (int)CurrentMach * 10;
        if (tempid >= 10)
        {
            ClMultiplier = A6EBaseAeroData::CliftvsMachMulti[10];
        }
        else
        {
            ClMultiplier = A6EBaseAeroData::CliftvsMachMulti[tempid] + (A6EBaseAeroData::CliftvsMachMulti[tempid + 1] - A6EBaseAeroData::CliftvsMachMulti[tempid]) * (CurrentMach - tempid);
        }
        Cl = Cl * ClMultiplier;
        return Cl;
    }

    double getCdfromAOA(double correctAOA, double added_Cd) // correct aoa is define by common aoa + install angle need in range [-180 180]
    {
        int tempid = 0;
        double Cd = 0;
        double CdMultiplier = 1;
        if (correctAOA >= 0)
        {
            tempid = (int)correctAOA;
            Cd = A6EBaseAeroData::CdofWing[tempid] + (A6EBaseAeroData::CdofWing[tempid + 1] - A6EBaseAeroData::CdofWing[tempid]) * (correctAOA - tempid);
        }
        else
        {
            tempid = - (int)correctAOA;
            Cd = (A6EBaseAeroData::CdofWing[tempid] + (A6EBaseAeroData::CdofWing[tempid + 1] - A6EBaseAeroData::CdofWing[tempid]) * (- correctAOA - tempid));
        }
        tempid = (int)CurrentMach * 10;
        if (tempid >= 10)
        {
            /* code */
            CdMultiplier = A6EBaseAeroData::CdragvsMachMulti[tempid] * CurrentMach;
        }
        else
        {
            /* code */
            CdMultiplier = A6EBaseAeroData::CdragvsMachMulti[tempid] + (A6EBaseAeroData::CdragvsMachMulti[tempid + 1] - A6EBaseAeroData::CdragvsMachMulti[tempid]) * (CurrentMach - tempid);
        }
        Cd = - (Cd + added_Cd) * CdMultiplier;
        return Cd;
    }

public:
    const double ISAAirDensity = 1.225; // in Kg/m^3
    const double ISAAirPressure = 101325; //in Pa
    const double ISASpeedOfSound = 340.3; // in m/s
    double AirDensity = 1.225;
    double AirPressure = 101325;
    double SpeedOfSound = 340.3;
    Vec3 WindAround = {0,0,0};
    double FlowSpeed = 0;
    double AlieronPos = 0;
    double ElevatorPos = 0;
    double RudderPos = 0;
    double AngleOfAttack = 0;
    double AngleOfSlide = 0;
    double CurrentMach = 0;

    double RollSpeed = 0;

    double Angle_v_Roll = 0;
    double Angle_v_Yaw = 0;
    double Angle_v_Pitch = 0;

    double flap_states = 0;
    double gear_states = 0;
    double brake_states = 0;

    Vec3 LiftSideForceVTail = {0,0,0};
    Vec3 LiftForceWing[2] = {{0,0,0}, {0,0,0}};
    Vec3 DragForceWing[2] = {{0,0,0}, {0,0,0}};
    Vec3 LiftForceHTail = {0,0,0};

    Vec3 TotalDragWnF = {0,0,0};
    Vec3 DragVTail = {0,0,0};
    Vec3 DragHTail = {0,0,0};

    // apply one flight dynamic change
    void CalWingLift(int Pos)// pos 0 for left and 1 for right
    {
        double correctedAOAWing = AngleOfAttack + 2 + 3 * flap_states;
        if (correctedAOAWing > 180)
        {
            correctedAOAWing = correctedAOAWing - 360;
        }
        else if (correctedAOAWing < -180)
        {
            correctedAOAWing = correctedAOAWing + 360;
        }
        int temp = 0;
        double deltaAoa  = (Angle_v_Roll * 4) / WindAround.x;
        if (Angle_v_Roll < 0)
        {
            if (Pos == 0)
            {
                temp = 1;
                correctedAOAWing = correctedAOAWing - deltaAoa;
            }
            else
            {
                temp = -1;
                correctedAOAWing = correctedAOAWing + deltaAoa;
            }
        }
        else
        {
            if (Pos == 0)
            {
                temp = -1;
                correctedAOAWing = correctedAOAWing - deltaAoa;
            }
            else
            {
                temp = 1;
                correctedAOAWing = correctedAOAWing + deltaAoa;
            }
        }
        deltaAoa = (Angle_v_Pitch * 0.3) / WindAround.x;
        correctedAOAWing += deltaAoa;
        double WingLift = 0;
        if (Pos == 0)
        {
            WingLift = 0.5 * AirDensity * FlowSpeed * FlowSpeed * A6EBaseAeroData::wingArea * (getClfromAOA(correctedAOAWing) + AlieronPos * 0.02); 
        }
        else
        {
            WingLift = 0.5 * AirDensity * FlowSpeed * FlowSpeed * A6EBaseAeroData::wingArea * (getClfromAOA(correctedAOAWing) - AlieronPos * 0.02); 
        }
        LiftForceWing[Pos].z = 0; //WingLift * WindAround.z / FlowSpeed; this part is not for lift
        LiftForceWing[Pos].y = WingLift * WindAround.x / FlowSpeed + temp  * AirDensity * (4.5 * Angle_v_Roll) * (4.5 * Angle_v_Roll) * 1.5 * A6EBaseAeroData::wingArea;
        LiftForceWing[Pos].x = - WingLift * WindAround.y / FlowSpeed;
    }

    void CalHTailLift()
    {
        double correctedAOAWing = AngleOfAttack - 10 * (ElevatorPos - 0.2);
        if (correctedAOAWing > 180)
        {
            correctedAOAWing = correctedAOAWing - 360;
        }
        else if (correctedAOAWing < -180)
        {
            correctedAOAWing = correctedAOAWing + 360;
        }
        int temp = 0;
        double deltaAoa  = (Angle_v_Pitch * 8) / WindAround.x;
        if (Angle_v_Pitch > 0)
        {
            temp = 1;
            correctedAOAWing += deltaAoa;
        }
        else
        {
            temp = -1;
            correctedAOAWing += deltaAoa;
        }
        double WingLift = 0.5 * AirDensity * FlowSpeed * FlowSpeed * A6EBaseAeroData::HorizontalTailArea * getClfromAOA(correctedAOAWing) - temp * AirDensity * (8.5 * Angle_v_Pitch) * (8.5 * Angle_v_Pitch) * 280 * getCdfromAOA(90 - fabs(correctedAOAWing),0) * A6EBaseAeroData::HorizontalTailArea;
        LiftForceHTail.z = 0; //WingLift * WindAround.z / FlowSpeed; this part wont gene lift
        LiftForceHTail.y = WingLift * WindAround.x / FlowSpeed;
        LiftForceHTail.x = - WingLift * WindAround.y / FlowSpeed;
    }

    void CalVTailLift()
    {
        double correctedAOAWing = - AngleOfSlide - RudderPos * 10;
        if (correctedAOAWing > 180)
        {
            correctedAOAWing = correctedAOAWing - 360;
        }
        else if (correctedAOAWing < -180)
        {
            correctedAOAWing = correctedAOAWing + 360;
        }
        int temp = 0;
        double deltaAoa  = (Angle_v_Yaw * 8.5 - 2 * Angle_v_Roll ) / WindAround.x;
        if (Angle_v_Yaw * 8.5 - Angle_v_Roll * 2 > 0)
        {
            temp = 1;
            correctedAOAWing -= deltaAoa;
        }
        else
        {
            temp = -1;
             correctedAOAWing -= deltaAoa;
        }
        double WingLift = 0.5 * AirDensity * FlowSpeed * FlowSpeed * A6EBaseAeroData::VerticalTailArea * (getClfromAOA(correctedAOAWing) * 0.9);
        LiftSideForceVTail.y = 0; //WingLift * WindAround.y / FlowSpeed; this part wont have lift
        LiftSideForceVTail.z = WingLift * WindAround.x / FlowSpeed - temp * AirDensity * (8.5 * Angle_v_Yaw - 2 * Angle_v_Roll) * (8.5 * Angle_v_Yaw - 2 * Angle_v_Roll) * 360 * A6EBaseAeroData::VerticalTailArea;
        LiftSideForceVTail.x = - WingLift * WindAround.z / FlowSpeed;
    }

    void CalCenterDrag()
    {
        double correctedAOAWing = AngleOfAttack + 2 + 3 * flap_states;
        if (correctedAOAWing > 180)
        {
            correctedAOAWing = correctedAOAWing - 360;
        }
        else if (correctedAOAWing < -180)
        {
            correctedAOAWing = correctedAOAWing + 360;
        }
        double WingLift = 0.5 * AirDensity * FlowSpeed * FlowSpeed * A6EBaseAeroData::wingArea * 7 * getCdfromAOA(correctedAOAWing, 0.03 * gear_states) + 0.5 * brake_states * AirDensity * FlowSpeed * FlowSpeed * A6EBaseAeroData::VerticalTailArea * getCdfromAOA(90, 0);
        TotalDragWnF.z = WingLift * WindAround.z / FlowSpeed;
        TotalDragWnF.y = WingLift * WindAround.y / FlowSpeed;
        TotalDragWnF.x = WingLift * WindAround.x / FlowSpeed;
    }

    void CalVtailDrag()
    {
        double correctedAOAWing = - AngleOfSlide;
        if (correctedAOAWing > 180)
        {
            correctedAOAWing = correctedAOAWing - 360;
        }
        else if (correctedAOAWing < -180)
        {
            correctedAOAWing = correctedAOAWing + 360;
        }
        double WingLift = 0.5 * AirDensity * FlowSpeed * FlowSpeed * A6EBaseAeroData::VerticalTailArea * getCdfromAOA(correctedAOAWing, 0);
        
        DragVTail.z = WingLift * WindAround.z / FlowSpeed;
        DragVTail.y = WingLift * WindAround.y / FlowSpeed;
        DragVTail.x = WingLift * WindAround.x / FlowSpeed;
    }

    void CalWingDrag(int Pos) // 0 for left and 1 for right
    {
        double correctedAOAWing = AngleOfAttack + 2;
        if (correctedAOAWing > 180)
        {
            correctedAOAWing = correctedAOAWing - 360;
        }
        else if (correctedAOAWing < -180)
        {
            correctedAOAWing = correctedAOAWing + 360;
        }
        double WingLift = 0.5 * AirDensity * FlowSpeed * FlowSpeed * A6EBaseAeroData::wingArea * (getCdfromAOA(correctedAOAWing, 0));
        TotalDragWnF.z = WingLift * WindAround.z / FlowSpeed;
        TotalDragWnF.y = WingLift * WindAround.y / FlowSpeed;
        TotalDragWnF.x = WingLift * WindAround.x / FlowSpeed;
    }

    void initial()
    {
            AirDensity = 1.225;
            AirPressure = 101325;
            SpeedOfSound = 340.3;
            WindAround = {0,0,0};
            FlowSpeed = 0;
            AlieronPos = 0;
            ElevatorPos = 0;
            RudderPos = 0;
            AngleOfAttack = 0;
            AngleOfSlide = 0;
            CurrentMach = 0;

            RollSpeed = 0;

            Angle_v_Roll = 0;
            Angle_v_Yaw = 0;
            Angle_v_Pitch = 0;

            LiftSideForceVTail = {0,0,0};
            LiftForceWing[0] = {0,0,0};
            LiftForceWing[1] = {0,0,0};
            DragForceWing[0] = {0,0,0};
            DragForceWing[1] = {0,0,0};
            LiftForceHTail = {0,0,0};

            TotalDragWnF = {0,0,0};
            DragVTail = {0,0,0};
            DragHTail = {0,0,0};

            flap_states = 0;
            gear_states = 0;
            brake_states = 0;
    }
};

#endif