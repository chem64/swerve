#pragma once

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include <ctre/Phoenix.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "Constants.h"
#include "AHRS.h"
#include <pathplanner/lib/PathPlannerTrajectory.h>
#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/PathPoint.h>


class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  void DriveSwerve(double FWD, double STR, double RCW);
  void ConfigMotors();

  private:
    frc::XboxController m_driveController{0};
    std::shared_ptr<nt::NetworkTable> ntSwerve;
    AHRS *ahrs;
    double HeadingOffset = 0;
    double Heading = 0;
    boolean SwerveOrientationToField = false;
    
    //CAN Devices
    WPI_TalonFX m_frTurn{constants::kFrontRightTurn_ID};
    WPI_TalonFX m_frDrive{constants::kFrontRightDrive_ID};
    WPI_CANCoder m_frEncoder{constants::kFrontRightEncoder_ID};

    WPI_TalonFX m_flTurn{constants::kFrontLeftTurn_ID};
    WPI_TalonFX m_flDrive{constants::kFrontLeftDrive_ID};
    WPI_CANCoder m_flEncoder{constants::kFrontLeftEncoder_ID};
    
    WPI_TalonFX m_rlTurn{constants::kRearLeftTurn_ID};
    WPI_TalonFX m_rlDrive{constants::kRearLeftDrive_ID};
    WPI_CANCoder m_rlEncoder{constants::kRearLeftEncoder_ID};
    
    WPI_TalonFX m_rrTurn{constants::kRearRightTurn_ID};
    WPI_TalonFX m_rrDrive{constants::kRearRightDrive_ID};
    WPI_CANCoder m_rrEncoder{constants::kRearRightEncoder_ID};
    
    
    //SWERVE
    double forward;
    double strafe;
    double rotate;

    double L;
    double W;
    double R;

	  double A;
    double B;
    double C;
    double D;

    double ws1;
    double wa1;
    double ws2;
    double wa2;
    double ws3;
    double wa3;
    double ws4;
    double wa4;

  SupplyCurrentLimitConfiguration driveSCLC;
  StatorCurrentLimitConfiguration driveStatorSCLC;
  SupplyCurrentLimitConfiguration turnSCLC;
  StatorCurrentLimitConfiguration turnStatorSCLC; 

  };
