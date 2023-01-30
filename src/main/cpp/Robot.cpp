#include "Robot.h"

void Robot::RobotInit() 
{
  ntBOSS = nt::NetworkTableInstance::GetDefault().GetTable("dashBOSS");
  ConfigMotors();
  m_TurnPID.EnableContinuousInput(-180.0,180.0); //required for swerve
  try
  {
    ahrs = new AHRS(frc::SPI::Port::kMXP);
    printf("NAVX Initialized OK");
    HeadingOffset = ahrs->GetYaw();
  }
  catch(const std::exception e)
  {
    printf("!!! NAVX ERROR !!!");
    HeadingOffset = 0;
  }
  AutoRev16_LeftBufStrm = new BufferedTrajectoryPointStream();
  AutoRev16_RightBufStrm = new BufferedTrajectoryPointStream();
  InitBuffer(AutoRev16_LeftBufStrm,auto16_L,auto16_size,true);
  InitBuffer(AutoRev16_RightBufStrm,auto16_R,auto16_size,true);
  AutoFwd2_LeftBufStrm = new BufferedTrajectoryPointStream();
  AutoFwd2_RightBufStrm = new BufferedTrajectoryPointStream();
  InitBuffer(AutoFwd2_LeftBufStrm,auto2_L,auto2_size,true);
  InitBuffer(AutoFwd2_RightBufStrm,auto2_R,auto2_size,true);

  //FMSMatch = frc::DriverStation::IsFMSAttached(); //is this a real match with FMS
  AutoTimer = new frc::Timer();
  AutoTimer->Start();
  ModeTimer = new frc::Timer();
  ModeTimer->Start();

  ClockStart = frc::Timer::GetFPGATimestamp();

}

void Robot::RobotPeriodic() 
{
  //only update this until match starts
  if(!MatchStart)
  {
    static int counter1 = 0;
    counter1++;
    if(counter1 >= 50) //1 secs
    {
      counter1=0;
      //Read AutoSelect and write AutoSelected on BOSS dashboard
      dAutoSelect = (int) ntBOSS->GetNumber("AutoSelect", 0.0);
      ntBOSS->PutNumber("AutoSelected", dAutoSelect);
      //allow BOSS dashboard to reset sticky faults
      int ResetStickyFaults = (int) ntBOSS->GetNumber("ResetStickyFaults", 0);
      if(ResetStickyFaults == 1)
      {
        ntBOSS->PutNumber("ResetStickyFaults", 0);
      }
      ntBOSS->PutNumber("ClockMode", 0);
    }
  }
  static int counter2 = 0;
  counter2++;
  if(counter2 >= 13) //250 msecs
  {
      ClockNow = frc::Timer::GetFPGATimestamp();
      counter2 = 0;
      if(IsAutonomousEnabled())
      {
        ntBOSS->PutNumber("ClockMode", 1);
        ntBOSS->PutNumber("ClockElapsed", std::clamp(15.0 - (double)(ClockNow - ClockStart),0.0,15.0));
      }
      if(IsTeleopEnabled())
      {
        ntBOSS->PutNumber("ClockMode", 2);
        ntBOSS->PutNumber("ClockElapsed", std::clamp(75.0 - (double)(ClockNow - ClockStart),0.0,75.0));
      }
      ntBOSS->PutNumber("CurMode", CurMode);
  }

  static int counter3 = 0;
  counter3++;
  if(counter3 >= 50) //1 secs
  {
    counter3=0;
    ntBOSS->PutNumber("FR_POS",CheckWrap(m_frEncoder.GetPosition()-constants::kFrontRightOffset));
    ntBOSS->PutNumber("FR_DIST", m_frDrive.GetSelectedSensorPosition() * constants::kDriveUnitsToFeet);
    ntBOSS->PutNumber("FL_POS",CheckWrap(m_flEncoder.GetPosition()-constants::kFrontLeftOffset));
    ntBOSS->PutNumber("FL_DIST", m_flDrive.GetSelectedSensorPosition() * constants::kDriveUnitsToFeet);
    ntBOSS->PutNumber("RL_POS",CheckWrap(m_rlEncoder.GetPosition()-constants::kRearLeftOffset));
    ntBOSS->PutNumber("RL_DIST", m_rlDrive.GetSelectedSensorPosition() * constants::kDriveUnitsToFeet);
    ntBOSS->PutNumber("RR_POS",CheckWrap(m_rrEncoder.GetPosition()-constants::kRearRightOffset));
    ntBOSS->PutNumber("RR_DIST", m_rrDrive.GetSelectedSensorPosition() * constants::kDriveUnitsToFeet);
    ntBOSS->PutNumber("joy_FORWARD", forward);
    ntBOSS->PutNumber("joy_STRAFE", strafe);
    ntBOSS->PutNumber("joy_ROTATE", rotate);
    ntBOSS->PutNumber("HeadingOffset", HeadingOffset);
    ntBOSS->PutNumber("Heading", Heading);
    ntBOSS->PutNumber("SwerveOrientationToField", SwerveOrientationToField);
    try
    {
      ntBOSS->PutNumber("ahrs_PITCH", ahrs->GetPitch());
      ntBOSS->PutNumber("ahrs_ROLL", ahrs->GetRoll());
    }
    catch(const std::exception e)
    {
      ntBOSS->PutNumber("ahrs_PITCH", 0);
      ntBOSS->PutNumber("ahrs_ROLL", 0);
    }
  }
  ntBOSS->PutString("AutoStatus", "");
}

void Robot::AutonomousInit() 
{
  //last chance check for auto selection
  dAutoSelect = (int) ntBOSS->GetNumber("AutoSelect", dAutoSelect);
  AutoState = 0;
  CurMode = 1;
}

void Robot::AutonomousPeriodic() 
{
  //execute the selected auto routine
  if(dAutoSelect == 1)  RunAuto_1();
  //if(dAutoSelect == 2)  RunAuto_2();
  //if(dAutoSelect == 3)  RunAuto_2();  //uses curve left stream for forward
  //if(dAutoSelect == 4)  RunAuto_2();  //uses curve right stream for forward
}

void Robot::TeleopInit() 
{
  CurMode = 2;
}

void Robot::TeleopPeriodic() 
{
  //xbox
  forward = -(m_driveController.GetRightY());
  strafe =  m_driveController.GetRightX();
  rotate = m_driveController.GetLeftX();
  
  if(fabs(forward)<.15) {forward = 0;}
  double fforward = spdFilter.Calculate(forward);
  if(forward != 0) forward = fforward;
  if(fabs(strafe)<.15) {strafe = 0;}
  if(fabs(rotate)<.2) {rotate = 0;}

  DriveSwerve(forward, strafe, rotate);

  //toggle robot/field orientation for swerve drive
  if(m_driveController.GetRawButtonPressed(0)) {SwerveOrientationToField = !SwerveOrientationToField;}
  
}

void Robot::DisabledInit() 
{
  StopAllDrives();
  CurMode = 0;
}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() 
{
  CurMode = 3;
}
void Robot::TestPeriodic() {}

double Robot::GetHeading()
{
  //subtract the offset recorded at init
  double yaw = ahrs->GetYaw() - HeadingOffset;
  //normalize angle
  //yaw = (int)yaw % 180; 
  yaw = CheckWrap(yaw);
  return yaw;
}


#ifndef RUNNING_FRC_TESTS
int main() 
{
  return frc::StartRobot<Robot>();
}
#endif

