/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <memory>

#include <frc/commands/Command.h>
#include <frc/Encoder.h>
#include <chrono>


class AutoCommand : public frc::Command {
private:
	//encoders for 4 drive motors
	frc::Encoder encoderFL{0, 1};
	frc::Encoder encoderRL{2, 3};
	frc::Encoder encoderFR{4, 5};
	frc::Encoder encoderRR{6, 7};

	std::chrono::time_point<std::chrono::steady_clock> startTime;

	//target distance in feet
	static constexpr float targetDistance = 6.0;

	static constexpr bool USE_TIMER = true;
public:
	AutoCommand();
	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;

private:
	bool FinishedMoving();
	void Reset();

	bool wasRunning = false;

};
