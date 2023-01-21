#pragma once

//Step1:  Set wheelbase and wheelwidth
//Step2:  Set all CAN ID numbers using Phoenix Tuner
//Step3:  Set each wheel to point forward (use straight-edge)
//Step4:  Set offset for each wheel (below) using absolute degrees from Tuner self-test snapshot
//Step5:  Adjust tuning numbers as necessary

namespace constants
{
  constexpr bool kSwerveOptimizeAngles = false;
  constexpr double kSwerveAngleBreak = 90;
  constexpr bool kSwerveReturnToZero = true;
  constexpr double kRadtoDeg = 57.2957795;
  constexpr double kDeg2Rad = 0.017453292519943295;

  constexpr double kWheelbase = 30.0;
  constexpr double kWheelwidth = 30.0;

  constexpr int kFrontLeftTurn_ID = 6;
  constexpr int kFrontLeftDrive_ID = 5;
  constexpr int kFrontLeftEncoder_ID = 16;

  constexpr int kFrontRightTurn_ID = 9;
  constexpr int kFrontRightDrive_ID = 10;
  constexpr int kFrontRightEncoder_ID = 11;

  constexpr int kRearLeftTurn_ID = 14;
  constexpr int kRearLeftDrive_ID = 15;
  constexpr int kRearLeftEncoder_ID = 21;

  constexpr int kRearRightTurn_ID = 19;
  constexpr int kRearRightDrive_ID = 20;
  constexpr int kRearRightEncoder_ID = 7;


  constexpr double kTurn_KP = 0.5; 
  constexpr double kTurn_KI = 0.001;
  constexpr double kTurn_KD = 0; 
  constexpr double kTurn_KF = 0;
  constexpr double kTurn_IZ = 20; 

  constexpr double kTurnPeakOutputForward = 0.2;
  constexpr double kTurnPeakOutputReverse = -0.2;
  //Supply Limiting is to prevent breakers tripping or brownouts
  constexpr double kTurnSupplyCurrentLimit = 20.0; //amps
  constexpr double kTurnPeakCurrentLimit = 30.0; //amps
  constexpr double kTurnPeakCurrentDuration = 25; //msecs
  //Stator limiting is to limit acceleration or heat
  constexpr double kTurnStatorCurrentLimit = 20.0;
  constexpr double kTurnStatorPeakCurrentLimit = 30.0; //amps
  constexpr double kTurnStatorPeakCurrentDuration = 15; //msecs
  constexpr double kTurnVoltageCompSaturation = 12.0;
  constexpr double kTurnClosedLoopRamp = 1.0;

  constexpr double kDrivePeakOutputForward = 0.8;
  constexpr double kDrivePeakOutputReverse = -0.8;
  //Supply Limiting is to prevent breakers tripping or brownouts
  constexpr double kDriveSupplyCurrentLimit = 35.0; //amps
  constexpr double kDrivePeakCurrentLimit = 40.0; //amps
  constexpr double kDrivePeakCurrentDuration = 25; //msecs
  //Stator limiting is to limit acceleration or heat
  constexpr double kDriveStatorCurrentLimit = 30.0;
  constexpr double kDriveStatorPeakCurrentLimit = 35.0; //amps
  constexpr double kDriveStatorPeakCurrentDuration = 15; //msecs
  constexpr double kDriveVoltageCompSaturation = 12.0;
  constexpr double kDriveOpenLoopRamp = 1.5;

  //read these from cancoder absolute position when cancoder is configured to -180 to 180 range
  constexpr double kFrontLeftOffset = 37.0;  
  constexpr double kFrontRightOffset = -24.6;
  constexpr double kRearLeftOffset = 65.8;
  constexpr double kRearRightOffset = 93.2;
  
  constexpr double kEncoderCountsPerDegree = 4096.0 / 360.0;

  //auto profile - drive wheel position - convert Feet to position units (encoder count)
  constexpr double kDriveCPR = 2048;
  constexpr double kDriveGearRatio = 6.75;
  constexpr double kDriveFeetPerRotation = 1.04719;   //4" wheel
  constexpr double kDriveFeetToUnits = (kDriveCPR * kDriveGearRatio) / kDriveFeetPerRotation; 
  constexpr double kDriveUnitsToFeet = 1 / kDriveFeetToUnits;  
  //auto profile - drive wheel speed - convert from Feet/Sec to  units/100ms
  constexpr double kDriveFPSToUnits = ((kDriveFeetPerRotation * kDriveGearRatio) / 10) * kDriveCPR;  
  constexpr double kDriveUnitsToFPS = 1 / kDriveFPSToUnits;    

}