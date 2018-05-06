package frc.team3926.robot.command.auto;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3926.robot.Robot;

import static frc.team3926.robot.Robot.driveSubsystem;

/**
 *
 */
public class MoreAuto extends Command {

    public MoreAuto() {

        requires(driveSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {

        Robot.sensorSubsystem.setClosedEncoders();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {


        driveSubsystem.setSpeed(0, driveSubsystem.rightPI(driveSubsystem.uniRightControl(6, 0)));
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
