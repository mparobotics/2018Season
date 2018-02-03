package frc.team3926.robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class EncoderCommand extends Command {

    public EncoderCommand() {

        requires(Robot.sensorSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {

        Robot.sensorSubsystem.enc.reset();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

        SmartDashboard.putNumber("distance", Robot.sensorSubsystem.Encoder("Distance"));
        //SmartDashboard.putNumber("raw value", Robot.sensorSubsystem.Encoder("Raw Value"));
        //SmartDashboard.putNumber("rate", Robot.sensorSubsystem.Encoder("Rate"));
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
