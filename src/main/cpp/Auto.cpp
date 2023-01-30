#include "Robot.h"

bool Robot::AutoIsRunning()
{
  if (!m_flDrive.IsMotionProfileFinished() && !m_frDrive.IsMotionProfileFinished())
    return true;
  else return false;
}

void Robot::ZeroDistance()
{
  //set ticks equal to zero
  m_frDrive.GetSensorCollection().SetIntegratedSensorPosition(0,10);
  m_flDrive.GetSensorCollection().SetIntegratedSensorPosition(0,10);
  m_rlDrive.GetSensorCollection().SetIntegratedSensorPosition(0,10);
  m_rrDrive.GetSensorCollection().SetIntegratedSensorPosition(0,10);
}

void Robot::AutoReset()
{
  ZeroDistance();
  ntBOSS->PutString("AutoStatus", "RESET");
}

void Robot::InitBuffer(BufferedTrajectoryPointStream *bufstrm, const double profile[][2], int totalCnt, bool reverse)
{
    bool forward = !reverse; 
    TrajectoryPoint point; 

    bufstrm->Clear(); // clear the buffer, in case it was used elsewhere 
    for (int i = 0; i < totalCnt; ++i) // Insert every point into buffer
    {
        double direction = forward ? +1 : -1;
        double positionFeet = profile[i][0];
        double velocityFeetPerSec = profile[i][1];
        int durationMilliseconds = 10; 
        
        // for each point, fill our structure and pass it to API 
        point.timeDur = durationMilliseconds;
        point.position = direction * positionFeet * constants::kDriveFeetToUnits; //Convert Feet to Units
        point.velocity = direction * velocityFeetPerSec * constants::kDriveFPSToUnits; //Convert Feet/sec to Units/100ms
        point.auxiliaryPos = 0;
        point.auxiliaryVel = 0;
        point.profileSlotSelect0 = 0; 
        point.profileSlotSelect1 = 0; 
        point.zeroPos = (i == 0); // set this to true on the first point 
        point.isLastPoint = ((i + 1) == totalCnt); // set this to true on the last point 
        point.arbFeedFwd = 0; // you can add a constant offset to add to PID[0] output here 
        bufstrm->Write(point);
    }
}

//place object, reverse 16.5 ft, forward 2.5 ft, balance on table
void::Robot::RunAuto_1() 
{
    double rollDeg = 0.0;
    double driveOut = 0.0;
    switch(AutoState)
    {
        case 0:
            AutoTimer->Reset();
            AutoState = 10;
            ntBOSS->PutString("AutoStatus", "AUTO1_PLACE");
            //if(constants::kAutoLoggingEnabled) ntBOSS->PutNumber("AutoState", AutoState);
            break;
        case 10: //place object
            if(AutoTimer->AdvanceIfElapsed((units::time::second_t) 5))
            {
                AutoState = 20;
                //if(constants::kAutoLoggingEnabled) ntBOSS->PutNumber("AutoState", AutoState);
            }
            break;
        case 20: //start drive control
            ntBOSS->PutString("AutoStatus", "AUTO1_REV16");
            m_flDrive.StartMotionProfile(*AutoRev16_LeftBufStrm, 10, ControlMode::MotionProfile);
            m_rlDrive.StartMotionProfile(*AutoRev16_LeftBufStrm, 10, ControlMode::MotionProfile);
            m_frDrive.StartMotionProfile(*AutoRev16_RightBufStrm, 10, ControlMode::MotionProfile);
            m_rrDrive.StartMotionProfile(*AutoRev16_RightBufStrm, 10, ControlMode::MotionProfile);
            AutoState = 30;
            //if(constants::kAutoLoggingEnabled) ntBOSS->PutNumber("AutoState", AutoState);
            break;
        case 30: //process drive control
            //look for end of profile
            if(!AutoIsRunning())
            {
                AutoState = 40;
                //if(constants::kAutoLoggingEnabled) ntBOSS->PutNumber("AutoState", AutoState);
            }
            break;
        case 40: //start drive control
            ntBOSS->PutString("AutoStatus", "AUTO1_FWD2");
            m_flDrive.StartMotionProfile(*AutoFwd2_LeftBufStrm, 10, ControlMode::MotionProfile);
            m_rlDrive.StartMotionProfile(*AutoFwd2_LeftBufStrm, 10, ControlMode::MotionProfile);
            m_frDrive.StartMotionProfile(*AutoFwd2_RightBufStrm, 10, ControlMode::MotionProfile);
            m_rrDrive.StartMotionProfile(*AutoFwd2_RightBufStrm, 10, ControlMode::MotionProfile);
            AutoState = 50;
            //if(constants::kAutoLoggingEnabled) ntBOSS->PutNumber("AutoState", AutoState);
            break;
        case 50: //process drive control
            //look for end of profile
            if(!AutoIsRunning())
            {
                AutoState = 60;
                ntBOSS->PutString("AutoStatus", "LEVEL UP");
                //if(constants::kAutoLoggingEnabled) ntBOSS->PutNumber("AutoState", AutoState);
            }
            break;
        case 60: //level up on charge table
            rollDeg = ahrs->GetRoll();
            driveOut = sin(rollDeg) * -1.0;
            m_frDrive.Set(ControlMode::PercentOutput,driveOut);
            m_flDrive.Set(ControlMode::PercentOutput,driveOut);
            m_rlDrive.Set(ControlMode::PercentOutput,driveOut);
            m_rrDrive.Set(ControlMode::PercentOutput,driveOut);
            break;
        case 70:
            ntBOSS->PutString("AutoStatus", "AUTO1_DONE");
            StopAllDrives();
            break;
    }
   
}
