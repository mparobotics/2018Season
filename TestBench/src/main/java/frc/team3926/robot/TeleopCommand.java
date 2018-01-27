
package frc.team3926.robot;


import edu.wpi.first.wpilibj.command.Command;

import static frc.team3926.robot.Robot.driveSubsystem;

/**
 *
 */


public class TeleopCommand extends Command {

    boolean hittingSomething;

    public TeleopCommand() {

       requires(driveSubsystem);

    }

    // Called just before this Command runs the first time
    protected void initialize() {


    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

        driveSubsystem.teleopDrive();

        //SmartDashboard.putBoolean("Hitting Something", Robot.OI.limitSwitch.get()); //.get() is not working

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
