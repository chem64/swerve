#include "Robot.h"

enum Constants {kSlotIdx = 0, kPIDLoopIdx = 0, kTimeoutMs = 50};

//wraps -180 to/from 180
double Robot::CheckWrap(double pos)
{
  double ret = pos;
  ret -= 360. * std::floor((ret + 180.) * (1. / 360.)); 
  return ret;
}

void Robot::StopAllDrives()
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

void Robot::DriveSwerve(double FWD, double STR, double RCW)
{
  static double lastFR_SP = 0.0;
  static double lastFL_SP = 0.0;
  static double lastRL_SP = 0.0;
  static double lastRR_SP = 0.0;
  L = constants::kWheelbase; //wheelbase (from center of front wheel to center of back wheel)
  W = constants::kWheelwidth; //wheelwidth (from center of left wheel to center of right wheel)
  R = sqrt((L * L) + (W * W));

  //get current heading
  Heading = GetHeading();
  if(SwerveOrientationToField)
  {
    //convert to radians
    double yaw = Heading / constants::kRadtoDeg;
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

  if (FWD == 0 && STR == 0 && RCW == 0)
  {
    StopAllDrives();
    return;
  }
  //Front Right
  double turnPV = CheckWrap(m_frEncoder.GetPosition()-constants::kFrontRightOffset);  //cancoder position accumulates on every turn - does not automatically wrap
  double turnSP = wa1;
  
  if(constants::kSwerveOptimizeAngles)
  {
    if(frFlip < 0) (turnSP<=0.0)?turnSP+=180.0:turnSP-=180.0;
    if (fabs(turnSP - lastFR_SP) > constants::kSwerveAngleBreak) 
    {
      frFlip *= -1.0;
      (turnSP<=0.0)?turnSP+=180.0:turnSP-=180.0;
      lastFR_SP = turnSP;
    }
  }
  double turnOUT = std::clamp(m_TurnPID.Calculate(turnPV,turnSP),-1.0,1.0); 
  m_frTurn.Set(ControlMode::PercentOutput,turnOUT);
  m_frDrive.Set(ControlMode::PercentOutput,ws1 *= frFlip);

  /*static int counter2 = 0;
  counter2++;
  if(counter2 >= 13) //250 msecs
  {
      counter2 = 0;
      ntBOSS->PutNumber("FR_SP", m_TurnPID.GetSetpoint());
      ntBOSS->PutNumber("FR_ERR", m_TurnPID.GetPositionError());
  }*/

  //Front Left
  turnPV = CheckWrap(m_flEncoder.GetPosition()-constants::kFrontLeftOffset);  //cancoder position accumulates on every turn - does not automatically wrap
  turnSP = wa2;
  
  if(constants::kSwerveOptimizeAngles)
  {
    if(flFlip < 0) (turnSP<=0.0)?turnSP+=180.0:turnSP-=180.0;
    if (fabs(turnSP - lastFL_SP) > constants::kSwerveAngleBreak) 
    {
      flFlip *= -1.0;
      (turnSP<=0.0)?turnSP+=180.0:turnSP-=180.0;
      lastFL_SP = turnSP;
    }
  }
  turnOUT = std::clamp(m_TurnPID.Calculate(turnPV,turnSP),-1.0,1.0); 
  m_flTurn.Set(ControlMode::PercentOutput,turnOUT);
  m_flDrive.Set(ControlMode::PercentOutput,ws2 *= flFlip);

  //Rear Left
  turnPV = CheckWrap(m_rlEncoder.GetPosition()-constants::kRearLeftOffset);  //cancoder position accumulates on every turn - does not automatically wrap
  turnSP = wa3;
  
  if(constants::kSwerveOptimizeAngles)
  {
    if(rlFlip < 0) (turnSP<=0.0)?turnSP+=180.0:turnSP-=180.0;
    if (fabs(turnSP - lastRL_SP) > constants::kSwerveAngleBreak) 
    {
      rlFlip *= -1.0;
      (turnSP<=0.0)?turnSP+=180.0:turnSP-=180.0;
      lastRL_SP = turnSP;
    }
  }
  turnOUT = std::clamp(m_TurnPID.Calculate(turnPV,turnSP),-1.0,1.0); 
  m_rlTurn.Set(ControlMode::PercentOutput,turnOUT);
  m_rlDrive.Set(ControlMode::PercentOutput,ws3 *= rlFlip);

  //Rear Right
  turnPV = CheckWrap(m_rrEncoder.GetPosition()-constants::kRearRightOffset);  //cancoder position accumulates on every turn - does not automatically wrap
  turnSP = wa4;
  
  if(constants::kSwerveOptimizeAngles)
  {
    if(rrFlip < 0) (turnSP<=0.0)?turnSP+=180.0:turnSP-=180.0;
    if (fabs(turnSP - lastRR_SP) > constants::kSwerveAngleBreak) 
    {
      rrFlip *= -1.0;
      (turnSP<=0.0)?turnSP+=180.0:turnSP-=180.0;
      lastRR_SP = turnSP;
    }
  }
  turnOUT = std::clamp(m_TurnPID.Calculate(turnPV,turnSP),-1.0,1.0); 
  m_rrTurn.Set(ControlMode::PercentOutput,turnOUT);
  m_rrDrive.Set(ControlMode::PercentOutput,ws4 *= rrFlip);

}


void Robot::ConfigMotors()
{
  driveSCLC = SupplyCurrentLimitConfiguration{true,constants::kDriveSupplyCurrentLimit,constants::kDrivePeakCurrentLimit,constants::kDrivePeakCurrentDuration};
  driveStatorSCLC = StatorCurrentLimitConfiguration{true,constants::kDriveStatorCurrentLimit,constants::kDriveStatorPeakCurrentLimit,constants::kDriveStatorPeakCurrentDuration};
  turnSCLC = SupplyCurrentLimitConfiguration{true,constants::kTurnSupplyCurrentLimit,constants::kTurnPeakCurrentLimit,constants::kTurnPeakCurrentDuration};
  turnStatorSCLC = StatorCurrentLimitConfiguration{true,constants::kTurnStatorCurrentLimit,constants::kTurnStatorPeakCurrentLimit,constants::kTurnStatorPeakCurrentDuration};
  
  m_frDrive.ConfigFactoryDefault(kTimeoutMs);
  m_frDrive.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, kTimeoutMs);
  m_frDrive.SetInverted(true);
  m_frDrive.ConfigNeutralDeadband(0.04, kTimeoutMs);
  m_frDrive.SetNeutralMode(NeutralMode::Brake);
  m_frDrive.ConfigNominalOutputForward(0, kTimeoutMs);
  m_frDrive.ConfigNominalOutputReverse(0, kTimeoutMs);
  m_frDrive.ConfigPeakOutputForward(constants::kDrivePeakOutputForward, kTimeoutMs);
  m_frDrive.ConfigPeakOutputReverse(constants::kDrivePeakOutputReverse, kTimeoutMs);
  m_frDrive.Config_kF(0, constants::kDrive_kF, kTimeoutMs);
  m_frDrive.Config_kP(0, constants::kDrive_kP, kTimeoutMs);
  m_frDrive.Config_kI(0, constants::kDrive_kI, kTimeoutMs);
  m_frDrive.Config_kD(0, constants::kDrive_kD, kTimeoutMs);
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
  m_flDrive.Config_kF(0, constants::kDrive_kF, kTimeoutMs);
  m_flDrive.Config_kP(0, constants::kDrive_kP, kTimeoutMs);
  m_flDrive.Config_kI(0, constants::kDrive_kI, kTimeoutMs);
  m_flDrive.Config_kD(0, constants::kDrive_kD, kTimeoutMs);
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
  m_rlDrive.Config_kF(0, constants::kDrive_kF, kTimeoutMs);
  m_rlDrive.Config_kP(0, constants::kDrive_kP, kTimeoutMs);
  m_rlDrive.Config_kI(0, constants::kDrive_kI, kTimeoutMs);
  m_rlDrive.Config_kD(0, constants::kDrive_kD, kTimeoutMs);
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
  m_rrDrive.Config_kF(0, constants::kDrive_kF, kTimeoutMs);
  m_rrDrive.Config_kP(0, constants::kDrive_kP, kTimeoutMs);
  m_rrDrive.Config_kI(0, constants::kDrive_kI, kTimeoutMs);
  m_rrDrive.Config_kD(0, constants::kDrive_kD, kTimeoutMs);
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
  m_frEncoder.ConfigSensorDirection(true,10);
  m_frEncoder.ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
  m_frEncoder.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180);
  m_frEncoder.SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame::CANCoderStatusFrame_SensorData, 20);
  
  m_frTurn.ConfigFactoryDefault();
  m_frTurn.SetNeutralMode(NeutralMode::Brake);
  m_frTurn.ConfigNominalOutputForward(0);
	m_frTurn.ConfigNominalOutputReverse(0);
	m_frTurn.ConfigPeakOutputForward(constants::kTurnPeakOutputForward);
	m_frTurn.ConfigPeakOutputReverse(constants::kTurnPeakOutputReverse);
  m_frTurn.SetSafetyEnabled(false);
  m_frTurn.SetInverted(true);
  m_frTurn.ConfigSupplyCurrentLimit(turnSCLC);//CURRENT_LIMITING
  m_frTurn.ConfigStatorCurrentLimit(turnStatorSCLC);//CURRENT_LIMITING
  
  m_flEncoder.ConfigFactoryDefault();
  m_flEncoder.ConfigSensorDirection(true,10);
  m_flEncoder.ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
  m_flEncoder.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180);
  m_flEncoder.SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame::CANCoderStatusFrame_SensorData, 20);
  
  m_flTurn.ConfigFactoryDefault();
  m_flTurn.SetNeutralMode(NeutralMode::Brake);
  m_flTurn.ConfigNominalOutputForward(0);
	m_flTurn.ConfigNominalOutputReverse(0);
	m_flTurn.ConfigPeakOutputForward(constants::kTurnPeakOutputForward);
	m_flTurn.ConfigPeakOutputReverse(constants::kTurnPeakOutputReverse);
  m_flTurn.SetSafetyEnabled(false);
  m_flTurn.SetInverted(true);
  m_flTurn.ConfigSupplyCurrentLimit(turnSCLC);//CURRENT_LIMITING
  m_flTurn.ConfigStatorCurrentLimit(turnStatorSCLC);//CURRENT_LIMITING
  m_flTurn.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
  m_flTurn.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

  m_rlEncoder.ConfigFactoryDefault();
  m_rlEncoder.ConfigSensorDirection(true,10);
  m_rlEncoder.ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
  m_rlEncoder.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180);
  m_rlEncoder.SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame::CANCoderStatusFrame_SensorData, 20);
  
  m_rlTurn.ConfigFactoryDefault();
  m_rlTurn.SetNeutralMode(NeutralMode::Brake);
  m_rlTurn.ConfigNominalOutputForward(0);
	m_rlTurn.ConfigNominalOutputReverse(0);
	m_rlTurn.ConfigPeakOutputForward(constants::kTurnPeakOutputForward);
	m_rlTurn.ConfigPeakOutputReverse(constants::kTurnPeakOutputReverse);
  m_rlTurn.SetSafetyEnabled(false);
  m_rlTurn.SetInverted(true);
  m_rlTurn.ConfigSupplyCurrentLimit(turnSCLC);//CURRENT_LIMITING
  m_rlTurn.ConfigStatorCurrentLimit(turnStatorSCLC);//CURRENT_LIMITING
  m_rlTurn.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
  m_rlTurn.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

  m_rrEncoder.ConfigFactoryDefault();
  m_rrEncoder.ConfigSensorDirection(true,10);
  m_rrEncoder.ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
  m_rrEncoder.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180);
  m_rrEncoder.SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame::CANCoderStatusFrame_SensorData, 20);
  
  m_rrTurn.ConfigFactoryDefault();
  m_rrTurn.SetNeutralMode(NeutralMode::Brake);
  m_rrTurn.ConfigNominalOutputForward(0);
	m_rrTurn.ConfigNominalOutputReverse(0);
	m_rrTurn.ConfigPeakOutputForward(constants::kTurnPeakOutputForward);
	m_rrTurn.ConfigPeakOutputReverse(constants::kTurnPeakOutputReverse);
  m_rrTurn.SetSafetyEnabled(false);
  m_rrTurn.SetInverted(true);
  m_rrTurn.ConfigSupplyCurrentLimit(turnSCLC);//CURRENT_LIMITING
  m_rrTurn.ConfigStatorCurrentLimit(turnStatorSCLC);//CURRENT_LIMITING
  m_rrTurn.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
  m_rrTurn.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);
}