#include "Robot.h"

using namespace pathplanner;

enum Constants {kSlotIdx = 0, kPIDLoopIdx = 0, kTimeoutMs = 50};

//structure used for optimizing turn angles
struct TurnVecType{
  double wa;
  double ws;
}TurnVec;

void Robot::RobotInit() 
{
  ntSwerve = nt::NetworkTableInstance::GetDefault().GetTable("dashBOSS");
  driveSCLC = SupplyCurrentLimitConfiguration{true,constants::kDriveSupplyCurrentLimit,constants::kDrivePeakCurrentLimit,constants::kDrivePeakCurrentDuration};
  driveStatorSCLC = StatorCurrentLimitConfiguration{true,constants::kDriveStatorCurrentLimit,constants::kDriveStatorPeakCurrentLimit,constants::kDriveStatorPeakCurrentDuration};
  turnSCLC = SupplyCurrentLimitConfiguration{true,constants::kTurnSupplyCurrentLimit,constants::kTurnPeakCurrentLimit,constants::kTurnPeakCurrentDuration};
  turnStatorSCLC = StatorCurrentLimitConfiguration{true,constants::kTurnStatorCurrentLimit,constants::kTurnStatorPeakCurrentLimit,constants::kTurnStatorPeakCurrentDuration};
  ConfigMotors();
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
}

void Robot::RobotPeriodic() 
{
  static int tempCounter = 0;
  tempCounter++;
  if(tempCounter >= 50) //1 secs
  {
    ntSwerve->PutNumber("FR_ENC",m_frEncoder.GetPosition());
    ntSwerve->PutNumber("FR_DIST", m_frDrive.GetSelectedSensorPosition() * constants::kDriveUnitsToFeet);
    ntSwerve->PutNumber("FL_ENC",m_flEncoder.GetPosition());
    ntSwerve->PutNumber("FL_DIST", m_flDrive.GetSelectedSensorPosition() * constants::kDriveUnitsToFeet);
    ntSwerve->PutNumber("RL_ENC",m_rlEncoder.GetPosition());
    ntSwerve->PutNumber("RL_DIST", m_rlDrive.GetSelectedSensorPosition() * constants::kDriveUnitsToFeet);
    ntSwerve->PutNumber("RR_ENC",m_rrEncoder.GetPosition());
    ntSwerve->PutNumber("RR_DIST", m_rrDrive.GetSelectedSensorPosition() * constants::kDriveUnitsToFeet);
    ntSwerve->PutNumber("joy_FORWARD", forward);
    ntSwerve->PutNumber("joy_STRAFE", strafe);
    ntSwerve->PutNumber("joy_ROTATE", rotate);
    ntSwerve->PutNumber("HeadingOffset", HeadingOffset);
    ntSwerve->PutNumber("Heading", Heading);
    ntSwerve->PutNumber("SwerveOrientationToField", SwerveOrientationToField);
    try
    {
      ntSwerve->PutNumber("ahrs_PITCH", ahrs->GetPitch());
      ntSwerve->PutNumber("ahrs_ROLL", ahrs->GetRoll());
    }
    catch(const std::exception e)
    {
      ntSwerve->PutNumber("ahrs_PITCH", 0);
      ntSwerve->PutNumber("ahrs_ROLL", 0);
    }
  }
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}
void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() 
{
  //xbox
  forward = -(m_driveController.GetRightY());
  strafe =  m_driveController.GetRightX();
  rotate = m_driveController.GetLeftX();
  
  if(fabs(forward)<.2) {forward = 0;}
  if(fabs(strafe)<.2) {strafe = 0;}
  if(fabs(rotate)<.2) {rotate = 0;}
  DriveSwerve(forward, strafe, rotate);

  //toggle robot/field orientation for swerve drive
  if(m_driveController.GetRawButtonPressed(0)) {SwerveOrientationToField = !SwerveOrientationToField}
}

void Robot::DisabledInit() 
{
  m_rrTurn.StopMotor();
  m_frTurn.StopMotor();
  m_rlTurn.StopMotor();
  m_flTurn.StopMotor();
  m_rrDrive.StopMotor();
  m_frDrive.StopMotor();
  m_rlDrive.StopMotor();
  m_flDrive.StopMotor();
}

void Robot::DisabledPeriodic() {}
void Robot::TestInit() {}
void Robot::TestPeriodic() {}

double GetHeading()
{
  //subtract the offset recorded at init
  double yaw = ahrs->GetYaw() - HeadingOffset;
  //normalize angle
  yaw = (int)yaw % 180; 
}

TurnVecType AngleOptimize(double newAngle, double curAngle, double speed)
{
  TurnVecType vec;
  if(constants::kSwerveOptimizeAngles)
  {
    // Calculate the delta angle
    double delta = newAngle - curAngle;
    delta = std::clamp(delta,-180.0,180.0);  
    //if angle is too big, just use inverse angle and reverse motor
    if (fabs(delta) > constants::kSwerveAngleBreak) 
    {
      newAngle = ((int)newAngle + 180) % 180;
      speed = -speed;
    }
  }
  //convert from degrees to encoder counts
  vec.wa = newAngle *= constants::kEncoderCountsPerDegree;
  vec.ws = speed;
  return vec;
}

void Robot::DriveSwerve(double FWD, double STR, double RCW)
{
  L = constants::kWheelbase; //wheelbase (from center of front wheel to center of back wheel)
  W = constants::kWheelwidth; //wheelwidth (from center of left wheel to center of right wheel)
  R = sqrt((L * L) + (W * W));

  if(SwerveOrientationToField)
  {
    //get current heading
    double yaw = GetHeading();
    //convert to radians
    yaw = yaw / constants::kRadtoDeg;
    //recalculate joystick inputs
    double tmp = FWD * cos(yaw) + STR * sin(yaw);
    STR = -FWD * sin(yaw) + STR * cos(yaw);
    FWD = tmp;
  }

  A = STR - RCW * (L / R);
  B = STR + RCW * (L / R);
  C = FWD - RCW * (W / R);
  D = FWD + RCW * (W / R);

  ws1 = sqrt((B * B) + (C * C));
  wa1 = atan2(B, C) * 180 / M_PI;   //-180 to 180 degrees
  ws2 = sqrt((B * B) + (D * D));
  wa2 = atan2(B, D) * 180 / M_PI;
  ws3 = sqrt((A * A) + (D * D));
  wa3 = atan2(A, D) * 180 / M_PI;
  ws4 = sqrt((A * A) + (C * C));
  wa4 = atan2(A, C) * 180 / M_PI;
  
  double max = ws1;
  if (ws2 > max)max = ws2;
  if (ws3 > max)max = ws3;
  if (ws4 > max)max = ws4;
  if (max > 1) { ws1 /= max; ws2 /= max; ws3 /= max; ws4 /= max; }

  //if desired, don't return wheels to zero when releasing joystick
  if(!constants::kSwerveReturnToZero)
  {
    if (FWD == 0 && STR == 0 && RCW == 0)
    {
      m_frTurn.StopMotor();
      m_flTurn.StopMotor();
      m_rlTurn.StopMotor();
      m_rrTurn.StopMotor();
      return;
    }
  }

  wa1 -= constants::kFrontRightOffset;
  TurnVec = AngleOptimize(wa1,m_frEncoder.GetPosition(), ws1); 
  m_frTurn.Set(TalonFXControlMode::Position, TurnVec.wa); 
  m_frDrive.Set(TurnVec.ws); 

  wa2 -= constants::kFrontLeftOffset;
  TurnVec = AngleOptimize(wa2,m_flEncoder.GetPosition(), ws2); 
  m_flTurn.Set(TalonFXControlMode::Position, TurnVec.wa); 
  m_flDrive.Set(TurnVec.ws); 

  wa3 -= constants::kRearLeftOffset;
  TurnVec = AngleOptimize(wa3,m_rlEncoder.GetPosition(), ws3); 
  m_rlTurn.Set(TalonFXControlMode::Position, TurnVec.wa); 
  m_rlDrive.Set(TurnVec.ws); 

  wa4 -= constants::kRearRightOffset;
  TurnVec = AngleOptimize(wa4,m_rrEncoder.GetPosition(), ws4); 
  m_rrTurn.Set(TalonFXControlMode::Position, TurnVec.wa); 
  m_rrDrive.Set(TurnVec.ws); 

}



void Robot::ConfigMotors()
{
  m_frDrive.ConfigFactoryDefault(kTimeoutMs);
  m_frDrive.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, kTimeoutMs);
  m_frDrive.SetInverted(true);
  m_frDrive.ConfigNeutralDeadband(0.04, kTimeoutMs);
  m_frDrive.SetNeutralMode(NeutralMode::Brake);
  m_frDrive.ConfigNominalOutputForward(0, kTimeoutMs);
  m_frDrive.ConfigNominalOutputReverse(0, kTimeoutMs);
  m_frDrive.ConfigPeakOutputForward(constants::kDrivePeakOutputForward, kTimeoutMs);
  m_frDrive.ConfigPeakOutputReverse(constants::kDrivePeakOutputReverse, kTimeoutMs);
  m_frDrive.ConfigOpenloopRamp(constants::kDriveOpenLoopRamp,kTimeoutMs); 
  m_frDrive.Config_IntegralZone(0, 400, kTimeoutMs);
  m_frDrive.ConfigClosedLoopPeakOutput(0,1.0,kTimeoutMs);
  m_frDrive.ConfigSupplyCurrentLimit(driveSCLC);//CURRENT_LIMITING
  m_frDrive.ConfigStatorCurrentLimit(driveStatorSCLC);//CURRENT_LIMITING
  m_frDrive.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
  m_frDrive.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);
  m_frDrive.ConfigVoltageCompSaturation(constants::kDriveVoltageCompSaturation);
  m_frDrive.EnableVoltageCompensation(true);
  m_frDrive.SetSafetyEnabled(false);

  m_flDrive.ConfigFactoryDefault(kTimeoutMs);
  m_flDrive.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, kTimeoutMs);
  m_flDrive.SetInverted(true);
  m_flDrive.ConfigNeutralDeadband(0.04, kTimeoutMs);
  m_flDrive.SetNeutralMode(NeutralMode::Brake);
  m_flDrive.ConfigNominalOutputForward(0, kTimeoutMs);
  m_flDrive.ConfigNominalOutputReverse(0, kTimeoutMs);
  m_flDrive.ConfigPeakOutputForward(constants::kDrivePeakOutputForward, kTimeoutMs);
  m_flDrive.ConfigPeakOutputReverse(constants::kDrivePeakOutputReverse, kTimeoutMs);
  m_flDrive.ConfigOpenloopRamp(constants::kDriveOpenLoopRamp,kTimeoutMs); 
  m_flDrive.Config_IntegralZone(0, 400, kTimeoutMs);
  m_flDrive.ConfigClosedLoopPeakOutput(0,1.0,kTimeoutMs);
  m_flDrive.ConfigSupplyCurrentLimit(driveSCLC);//CURRENT_LIMITING
  m_flDrive.ConfigStatorCurrentLimit(driveStatorSCLC);//CURRENT_LIMITING
  m_flDrive.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
  m_flDrive.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);
  m_flDrive.ConfigVoltageCompSaturation(constants::kDriveVoltageCompSaturation);
  m_flDrive.EnableVoltageCompensation(true);
  m_flDrive.SetSafetyEnabled(false);

  m_rlDrive.ConfigFactoryDefault(kTimeoutMs);
  m_rlDrive.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, kTimeoutMs);
  m_rlDrive.SetInverted(true);
  m_rlDrive.ConfigNeutralDeadband(0.04, kTimeoutMs);
  m_rlDrive.SetNeutralMode(NeutralMode::Brake);
  m_rlDrive.ConfigNominalOutputForward(0, kTimeoutMs);
  m_rlDrive.ConfigNominalOutputReverse(0, kTimeoutMs);
  m_rlDrive.ConfigPeakOutputForward(constants::kDrivePeakOutputForward, kTimeoutMs);
  m_rlDrive.ConfigPeakOutputReverse(constants::kDrivePeakOutputReverse, kTimeoutMs);
  m_rlDrive.ConfigOpenloopRamp(constants::kDriveOpenLoopRamp,kTimeoutMs); 
  m_rlDrive.Config_IntegralZone(0, 400, kTimeoutMs);
  m_rlDrive.ConfigClosedLoopPeakOutput(0,1.0,kTimeoutMs);
  m_rlDrive.ConfigSupplyCurrentLimit(driveSCLC);//CURRENT_LIMITING
  m_rlDrive.ConfigStatorCurrentLimit(driveStatorSCLC);//CURRENT_LIMITING
  m_rlDrive.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
  m_rlDrive.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);
  m_rlDrive.ConfigVoltageCompSaturation(constants::kDriveVoltageCompSaturation);
  m_rlDrive.EnableVoltageCompensation(true);
  m_rlDrive.SetSafetyEnabled(false);

  m_rrDrive.ConfigFactoryDefault(kTimeoutMs);
  m_rrDrive.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, kTimeoutMs);
  m_rrDrive.SetInverted(true);
  m_rrDrive.ConfigNeutralDeadband(0.04, kTimeoutMs);
  m_rrDrive.SetNeutralMode(NeutralMode::Brake);
  m_rrDrive.ConfigNominalOutputForward(0, kTimeoutMs);
  m_rrDrive.ConfigNominalOutputReverse(0, kTimeoutMs);
  m_rrDrive.ConfigPeakOutputForward(constants::kDrivePeakOutputForward, kTimeoutMs);
  m_rrDrive.ConfigPeakOutputReverse(constants::kDrivePeakOutputReverse, kTimeoutMs);
  m_rrDrive.ConfigOpenloopRamp(constants::kDriveOpenLoopRamp,kTimeoutMs); 
  m_rrDrive.Config_IntegralZone(0, 400, kTimeoutMs);
  m_rrDrive.ConfigClosedLoopPeakOutput(0,1.0,kTimeoutMs);
  m_rrDrive.ConfigSupplyCurrentLimit(driveSCLC);//CURRENT_LIMITING
  m_rrDrive.ConfigStatorCurrentLimit(driveStatorSCLC);//CURRENT_LIMITING
  m_rrDrive.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
  m_rrDrive.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);
  m_rrDrive.ConfigVoltageCompSaturation(constants::kDriveVoltageCompSaturation);
  m_rrDrive.EnableVoltageCompensation(true);
  m_rrDrive.SetSafetyEnabled(false);

  m_frEncoder.ConfigFactoryDefault();
  m_frEncoder.ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
  m_frEncoder.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180);
  
  m_frTurn.ConfigFactoryDefault();
  m_frTurn.SelectProfileSlot(0, 0);
  m_frTurn.SetNeutralMode(NeutralMode::Brake);
  m_frTurn.ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0, 0, 0); // PIDLoop=0, timeoutMs=0
  m_frTurn.ConfigRemoteFeedbackFilter(constants::kFrontRightEncoder_ID, RemoteSensorSource::RemoteSensorSource_CANCoder, 0, 0);
  m_frTurn.SetSensorPhase(false);
  m_frTurn.Config_kP(0, constants::kTurn_KP);
  m_frTurn.Config_kI(0, constants::kTurn_KI);
  m_frTurn.Config_kD(0, constants::kTurn_KD);
  m_frTurn.Config_kF(0, constants::kTurn_KF);
  m_frTurn.Config_IntegralZone(0, constants::kTurn_IZ);
  m_frTurn.ConfigNominalOutputForward(0);
	m_frTurn.ConfigNominalOutputReverse(0);
	m_frTurn.ConfigPeakOutputForward(constants::kTurnPeakOutputForward);
	m_frTurn.ConfigPeakOutputReverse(constants::kTurnPeakOutputReverse);
  m_frTurn.SetSafetyEnabled(false);
  m_frTurn.SetInverted(true);
  m_frTurn.ConfigAllowableClosedloopError(0,1,10);
  //m_frTurn.ConfigClosedLoopRamp(constants::kTurnClosedLoopRamp);
  m_frTurn.ConfigSupplyCurrentLimit(turnSCLC);//CURRENT_LIMITING
  m_frTurn.ConfigStatorCurrentLimit(turnStatorSCLC);//CURRENT_LIMITING
  m_frTurn.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
  m_frTurn.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

  m_flEncoder.ConfigFactoryDefault();
  m_flEncoder.ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
  m_flEncoder.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180);
  
  m_flTurn.ConfigFactoryDefault();
  m_flTurn.SelectProfileSlot(0, 0);
  m_flTurn.SetNeutralMode(NeutralMode::Brake);
  m_flTurn.ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0, 0, 0); // PIDLoop=0, timeoutMs=0
  m_flTurn.ConfigRemoteFeedbackFilter(constants::kFrontLeftEncoder_ID, RemoteSensorSource::RemoteSensorSource_CANCoder, 0, 0);
  m_flTurn.SetSensorPhase(false);
  m_flTurn.Config_kP(0, constants::kTurn_KP);
  m_flTurn.Config_kI(0, constants::kTurn_KI);
  m_flTurn.Config_kD(0, constants::kTurn_KD);
  m_flTurn.Config_kF(0, constants::kTurn_KF);
  m_flTurn.Config_IntegralZone(0, constants::kTurn_IZ);
  m_flTurn.ConfigNominalOutputForward(0);
	m_flTurn.ConfigNominalOutputReverse(0);
	m_flTurn.ConfigPeakOutputForward(0.2);
	m_flTurn.ConfigPeakOutputReverse(-0.2);
  m_flTurn.SetSafetyEnabled(false);
  m_flTurn.SetInverted(true);
  m_flTurn.ConfigAllowableClosedloopError(0,1,10);
  //m_flTurn.ConfigClosedLoopRamp(constants::kTurnClosedLoopRamp);
  m_flTurn.ConfigSupplyCurrentLimit(turnSCLC);//CURRENT_LIMITING
  m_flTurn.ConfigStatorCurrentLimit(turnStatorSCLC);//CURRENT_LIMITING
  m_flTurn.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
  m_flTurn.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

  m_rlEncoder.ConfigFactoryDefault();
  m_rlEncoder.ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
  m_rlEncoder.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180);
  
  m_rlTurn.ConfigFactoryDefault();
  m_rlTurn.SelectProfileSlot(0, 0);
  m_rlTurn.SetNeutralMode(NeutralMode::Brake);
  m_rlTurn.ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0, 0, 0); // PIDLoop=0, timeoutMs=0
  m_rlTurn.ConfigRemoteFeedbackFilter(constants::kRearLeftEncoder_ID, RemoteSensorSource::RemoteSensorSource_CANCoder, 0, 0);
  m_rlTurn.SetSensorPhase(false);
  m_rlTurn.Config_kP(0, constants::kTurn_KP);
  m_rlTurn.Config_kI(0, constants::kTurn_KI);
  m_rlTurn.Config_kD(0, constants::kTurn_KD);
  m_rlTurn.Config_kF(0, constants::kTurn_KF);
  m_rlTurn.Config_IntegralZone(0, constants::kTurn_IZ);
  m_rlTurn.ConfigNominalOutputForward(0);
	m_rlTurn.ConfigNominalOutputReverse(0);
	m_rlTurn.ConfigPeakOutputForward(0.2);
	m_rlTurn.ConfigPeakOutputReverse(-0.2);
  m_rlTurn.SetSafetyEnabled(false);
  m_rlTurn.SetInverted(true);
  m_rlTurn.ConfigAllowableClosedloopError(0,1,10);
  //m_rlTurn.ConfigClosedLoopRamp(constants::kTurnClosedLoopRamp);
  m_rlTurn.ConfigSupplyCurrentLimit(turnSCLC);//CURRENT_LIMITING
  m_rlTurn.ConfigStatorCurrentLimit(turnStatorSCLC);//CURRENT_LIMITING
  m_rlTurn.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
  m_rlTurn.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

  m_rrEncoder.ConfigFactoryDefault();
  m_rrEncoder.ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
  m_rrEncoder.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180);
  
  m_rrTurn.ConfigFactoryDefault();
  m_rrTurn.SelectProfileSlot(0, 0);
  m_rrTurn.SetNeutralMode(NeutralMode::Brake);
  m_rrTurn.ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0, 0, 0); // PIDLoop=0, timeoutMs=0
  m_rrTurn.ConfigRemoteFeedbackFilter(constants::kRearRightEncoder_ID, RemoteSensorSource::RemoteSensorSource_CANCoder, 0, 0);
  m_rrTurn.SetSensorPhase(false);
  m_rrTurn.Config_kP(0, constants::kTurn_KP);
  m_rrTurn.Config_kI(0, constants::kTurn_KI);
  m_rrTurn.Config_kD(0, constants::kTurn_KD);
  m_rrTurn.Config_kF(0, constants::kTurn_KF);
  m_rrTurn.Config_IntegralZone(0, constants::kTurn_IZ);
  m_rrTurn.ConfigNominalOutputForward(0);
	m_rrTurn.ConfigNominalOutputReverse(0);
	m_rrTurn.ConfigPeakOutputForward(0.2);
	m_rrTurn.ConfigPeakOutputReverse(-0.2);
  m_rrTurn.SetSafetyEnabled(false);
  m_rrTurn.SetInverted(true);
  m_rrTurn.ConfigAllowableClosedloopError(0,1,10);
  //m_rrTurn.ConfigClosedLoopRamp(constants::kTurnClosedLoopRamp);
  m_rrTurn.ConfigSupplyCurrentLimit(turnSCLC);//CURRENT_LIMITING
  m_rrTurn.ConfigStatorCurrentLimit(turnStatorSCLC);//CURRENT_LIMITING
  m_rrTurn.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
  m_rrTurn.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);
}

#ifndef RUNNING_FRC_TESTS
int main() 
{
  return frc::StartRobot<Robot>();
}
#endif
