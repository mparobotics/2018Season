
/*
package frc.team3926.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

*/
/**
 *
 *//*


public class AutonomousCommand extends Command {

    DriverStation ds;
    double time;
    int position;
    public AutonomousCommand() {

        requires(Robot.driveSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {

        */
/*ds = DriverStation.getInstance();
        position = DriverStation.getInstance().getLocation();*//*


        SmartDashboard.putNumber("Position: ", position);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

        Robot.driveSubsystem.sketchyAuto();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {

        return true;
    }

    // Called once after isFinished returns true
    protected void end() {

        Robot.driveSubsystem.setSpeed(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {

    }

}

*/
