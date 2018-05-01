package frc.team3926.robot.command.auto;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3926.robot.Robot;

/**
 *
 */
public class MoreAuto extends Command {

    public MoreAuto() {

        requires(Robot.driveSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {

        Robot.sensorSubsystem.setClosedEncoders();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

        Robot.driveSubsystem.setSpeed(0,Robot.driveSubsystem.rightPI());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {

        return false;
    }

    // Called once after isFinished returns true
    protected void end() {

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {

    }

}
