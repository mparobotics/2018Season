package org.usfirst.frc.team3926.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team3926.robot.OI;
import org.usfirst.frc.team3926.robot.Robot;

/**
 *
 */
public class DriveCommand extends Command {


	public DriveCommand() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.DrivingSubsystem);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {


	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		
		Robot.DrivingSubsystem.TankDriveMethod(OI.lJoystick.getY(), OI.rJoystick.getY());
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {

		return false;

	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		
		Robot.DrivingSubsystem.TankDriveMethod(0, 0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}
