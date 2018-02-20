package frc.team3926.robot.command.teleop;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.team3926.robot.Robot;

/**
 *
 */
public class DeployWingsCommand extends Command {

    private double time;

    public DeployWingsCommand() {

        requires(Robot.sensorSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {

        time = Timer.getFPGATimestamp();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

        Robot.sensorSubsystem.deployWings();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {

        if(Timer.getFPGATimestamp() - time >= 2) {

            return true;
        } else {

            return false;
        }
    }

    // Called once after isFinished returns true
    protected void end() {

        Robot.sensorSubsystem.resetWings();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {

    }

}
