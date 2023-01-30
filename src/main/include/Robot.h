#pragma once

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include <ctre/Phoenix.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "Constants.h"
#include "Auto_2.h"
#include "Auto_16.h"
#include "AHRS.h"
#include <frc/controller/PIDController.h>
#include <frc/filter/SlewRateLimiter.h>

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

  double GetHeading();
  double CheckWrap(double pos);
  void DriveSwerve(double FWD, double STR, double RCW);
  void ConfigMotors();
  void RunAuto_1();
  bool AutoIsRunning();
  void AutoReset();
  void InitBuffer(BufferedTrajectoryPointStream *bufstrm, const double profile[][2], int totalCnt, bool reverse);
  void ZeroDistance();
  void StopAllDrives();

  private:

    frc::XboxController m_driveController{0};
    std::shared_ptr<nt::NetworkTable> ntBOSS;
    AHRS *ahrs;
    double HeadingOffset = 0;
    double Heading = 0;
    bool SwerveOrientationToField = false;
    
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
    
    frc2::PIDController m_TurnPID{constants::kTurn_KP, constants::kTurn_KI, constants::kTurn_KD};
    frc::SlewRateLimiter<units::scalar> spdFilter{2/1_s};
    
    double frFlip = 1.0;
    double flFlip = 1.0;
    double rlFlip = 1.0;
    double rrFlip = 1.0;

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

  //Auto
  int AutoState = 0;
  frc::Timer *AutoTimer;
  frc::Timer *ModeTimer;
  units::time::second_t ClockStart;
  units::time::second_t ClockNow;
  
  BufferedTrajectoryPointStream *AutoRev16_LeftBufStrm;
  BufferedTrajectoryPointStream *AutoRev16_RightBufStrm;
  BufferedTrajectoryPointStream *AutoFwd2_LeftBufStrm;
  BufferedTrajectoryPointStream *AutoFwd2_RightBufStrm;

  //dashboard variables
  bool FMSMatch = false;  //true if attached to FMS
  bool MatchStart = false;
  int dAutoSelect = 0;
  int CurMode = 0;

  };
