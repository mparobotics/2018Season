package frc.team3926.robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutonomousTurnCommand extends Command {

    double angle;

    public AutonomousTurnCommand(double desiredAngle) {

        angle = desiredAngle;
        requires(Robot.driveSubsystem);
        requires(Robot.sensorSubsystem);

        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {

        Robot.sensorSubsystem.ResetGyro();

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

        Robot.driveSubsystem.turn(angle);

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {

        return Robot.sensorSubsystem.turnedAngle(angle);

    }

    // Called once after isFinished returns true
    protected void end() {

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {

    }

}
