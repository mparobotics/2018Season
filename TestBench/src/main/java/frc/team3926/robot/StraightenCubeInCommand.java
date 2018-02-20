package frc.team3926.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class StraightenCubeInCommand extends Command {

    double time;
    public StraightenCubeInCommand() {

       requires(Robot.intakeArmSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {

        Robot.intakeArmSubsystem.setIntakeSpeed(0, 0);

        time = Timer.getFPGATimestamp();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

        Robot.intakeArmSubsystem.setIntakeSpeed(.75, -.75);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {

        if(Timer.getFPGATimestamp() - time >= .25) {

            return true;
        } else {

            return false;
        }
    }

    // Called once after isFinished returns true
    protected void end() {

        Robot.intakeArmSubsystem.setIntakeSpeed(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {

    }

}
