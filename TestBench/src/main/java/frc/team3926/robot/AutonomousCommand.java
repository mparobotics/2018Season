
package frc.team3926.robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */

public class AutonomousCommand extends Command {

    boolean hittingSomething;
    public AutonomousCommand() {

        requires(Robot.driveSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

        //Robot.driveSubsystem.autonomous();

        //SmartDashboard.putBoolean("Hitting Something", Robot.OI.limitSwitch.get()); //.get() is not working
        if(hittingSomething) {

            Robot.driveSubsystem.hitSomething();
            //then needs to do something else
        }


        switch(RobotMap.STARTING_POSITION){

            case "right":
                switch(RobotMap.FREINDLY_SCALE_SIDE){

                    case "left":
                        break;

                    case "right":
                        break;

                    case "notGettingScale":
                        break;

                }
                break;

            case "left":
                switch(RobotMap.FREINDLY_SCALE_SIDE){

                    case "left":
                        break;

                    case "right":
                        break;

                    case "notGettingScale":
                        break;

                }
                break;

            case "middle":
                switch(RobotMap.FREINDLY_SCALE_SIDE){

                    case "left":
                        break;

                    case "right":
                        break;

                    case "notGettingScale":
                        break;

                }
                break;

        }




    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {


        return false;
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
