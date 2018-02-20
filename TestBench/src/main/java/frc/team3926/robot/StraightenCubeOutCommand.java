package frc.team3926.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class StraightenCubeOutCommand extends Command {

    double time;
    public StraightenCubeOutCommand() {

        requires(Robot.intakeArmSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {

        Robot.intakeArmSubsystem.setIntakeSpeed(0, 0);

        //time = new Timer();
        time = Timer.getFPGATimestamp();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

        //SmartDashboard.putNumber("timer: ", );

        Robot.intakeArmSubsystem.setIntakeSpeed(-.5, .5);

        /*if(Timer.getFPGATimestamp() - time >= .5) {

            Robot.intakeArmSubsystem.setIntakeSpeed(0, 0);
        } else {

            Robot.intakeArmSubsystem.setIntakeSpeed(-.5, .5);
        }*/
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {

        if(Timer.getFPGATimestamp() - time >= .08) {

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
