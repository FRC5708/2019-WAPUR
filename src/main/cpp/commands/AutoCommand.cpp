/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/AutoCommand.h"

#include "Robot.h"

#include <memory>


AutoCommand::AutoCommand() {
  // Use Requires() here to declare subsystem dependencies
  Requires(&Robot::m_subsystem);
}

// Called just before this Command runs the first time
void AutoCommand::Initialize() {
  //init the encoders distance calculations

  //256 pulses per rotation, wheels have a circumference of 1.57 feet
  encoderFL.SetDistancePerPulse(1.57/256.0);
  encoderRL.SetDistancePerPulse(1.57/256.0);
  encoderFR.SetDistancePerPulse(1.57/256.0);
  encoderRR.SetDistancePerPulse(1.57/256.0);

  
}

void AutoCommand::Reset() {
  
  encoderFL.Reset();
  encoderRL.Reset();
  encoderFR.Reset();
  encoderRR.Reset();

  startTime = std::chrono::steady_clock::now();
}

// Called repeatedly when this Command is scheduled to run
void AutoCommand::Execute() {
  if (!wasRunning) {
    Reset();
    wasRunning = true;
  }

  //drive forward if we haven't reached the proper distance
  if(!FinishedMoving()){
    Robot::instance->m_robotDrive.DriveCartesian(0.0, 1.0, 0.0); 
  } else {
    Robot::instance->m_robotDrive.DriveCartesian(0.0, 0.0, 0.0);
  }
}

bool AutoCommand::FinishedMoving() {
  if (USE_TIMER) {
    return std::chrono::steady_clock::now() - startTime >= std::chrono::seconds(3);
  }
  else {
    return (encoderRL.GetDistance() + encoderFL.GetDistance() + encoderFR.GetDistance() + encoderRR.GetDistance())
    / 4 >= targetDistance;
  }
}

// Make this return true when this Command no longer needs to run execute()
bool AutoCommand::IsFinished() {
  return false;    
}

// Called once after isFinished returns true
void AutoCommand::End() { wasRunning = false; }
