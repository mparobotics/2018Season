package frc.team3926.robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutonomousForwardCommand extends Command {

    private double distance;

    public AutonomousForwardCommand(double desiredDistance) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.driveSubsystem);
        distance = desiredDistance;

    }

    // Called just before this Command runs the first time
    protected void initialize() {

        Robot.driveSubsystem.resetErrorsAndIntegralsAndEncoders();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

        Robot.driveSubsystem.driveForward();

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {

        return Robot.driveSubsystem.drivenDistance(distance);

    }

    // Called once after isFinished returns true
    protected void end() {

        //resets the heap values for the preError's and integral's
        Robot.driveSubsystem.resetErrorsAndIntegralsAndEncoders();
        Robot.driveSubsystem.turnOffMotors();

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {

    }

}
