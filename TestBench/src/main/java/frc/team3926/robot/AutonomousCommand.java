
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

        switch(RobotMap.AUTONOMOUS_MODE){
            case 1:
                // go to auto line.
                break;

            case 2:
                // go to auto line
                //go to place to prepare for teleop
                break;

            case 3:
                // travel to switch
                switch(RobotMap.STARTING_POSITION){

                    case "right":
                        switch(RobotMap.FREINDLY_SWITCH_SIDE){

                            case "left":
                                //DriveSubsystem.autonomous(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
                                break;

                            case "right":
                                //DriveSubsystem.autonomous(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
                                break;
                        }

                        break;

                    case "left":
                        switch(RobotMap.FREINDLY_SWITCH_SIDE){

                            case "left":
                                //DriveSubsystem.autonomous(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
                                break;

                            case "right":
                                //DriveSubsystem.autonomous(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
                                break;
                        }

                        break;

                    case "middle":
                        switch(RobotMap.FREINDLY_SWITCH_SIDE){

                            case "left":
                                //DriveSubsystem.autonomous(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
                                break;

                            case "right":
                                //DriveSubsystem.autonomous(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
                                break;
                        }
                        break;

                }
                // place power cube on switch
                break;

            case 4:
                // travel to switch
                switch(RobotMap.STARTING_POSITION){

                    case "right":
                        switch(RobotMap.FREINDLY_SWITCH_SIDE){

                            case "left":
                                //DriveSubsystem.autonomous(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
                                break;

                            case "right":
                                //DriveSubsystem.autonomous(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
                                break;
                        }

                        break;

                    case "left":
                        switch(RobotMap.FREINDLY_SWITCH_SIDE){

                            case "left":
                                //DriveSubsystem.autonomous(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
                                break;

                            case "right":
                                //DriveSubsystem.autonomous(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
                                break;
                        }

                        break;

                    case "middle":
                        switch(RobotMap.FREINDLY_SWITCH_SIDE){

                            case "left":
                                //DriveSubsystem.autonomous(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
                                break;

                            case "right":

                                break;
                        }
                        break;

                }
                // place power cube on switch
                // go to place to prepare for teleop
                break;

            case 5:
                //travel to scale
                switch(RobotMap.STARTING_POSITION){

                    case "right":
                        switch(RobotMap.FREINDLY_SCALE_SIDE){

                            case "left":

                                break;

                            case "right":

                                break;
                        }

                        break;

                    case "left":
                        switch(RobotMap.FREINDLY_SCALE_SIDE){

                            case "left":

                                break;

                            case "right":

                                break;
                        }

                        break;

                    case "middle":
                        switch(RobotMap.FREINDLY_SCALE_SIDE){

                            case "left":

                                break;

                            case "right":

                                break;
                        }
                        break;

                }
                // place power cube on scale
                break;

            case 6:
                //travel to scale
                switch(RobotMap.STARTING_POSITION){

                    case "right":
                        switch(RobotMap.FREINDLY_SCALE_SIDE){

                            case "left":

                                break;

                            case "right":

                                break;
                        }

                        break;

                    case "left":
                        switch(RobotMap.FREINDLY_SCALE_SIDE){

                            case "left":

                                break;

                            case "right":

                                break;
                        }

                        break;

                    case "middle":
                        switch(RobotMap.FREINDLY_SCALE_SIDE){

                            case "left":

                                break;

                            case "right":

                                break;
                        }
                        break;

                }
                // place power cube on scale
                // go to place to prepare for teleop
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
