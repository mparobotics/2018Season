package frc.team3926.robot.command.auto;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3926.robot.Robot;

import static frc.team3926.robot.Robot.driveSubsystem;

/**
 *
 */
public class MoreAuto extends Command {

    double[][] square;

    double[][] leftRightNoScore;
    double[][] centerNoScore;

    double[][] rightScoreR;
    double[][] leftScoreR;
    double[][] centerScoreR;

    double[][] rightScoreL;
    double[][] leftScoreL;
    double[][] centerScoreL;

    double[][] path;

    int posistion;
    int scoreOn;
    String switchPosistion;

    public MoreAuto() {

        requires(driveSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() { //1 = driving mode, 0 = turning mode

        Robot.sensorSubsystem.setClosedEncoders();
         square = new double[][]{{60, 0, 0, 1}, {60, 0, (Math.PI * .5), 0}, {60, 60, 0, 1}, {60,60,0,1}, {60,60,0,1}};

        //no score
        leftRightNoScore = new double[][] {{168, 0, 0, 1}, {168, 0, 0, 0}, {168, 0, 0, 0}};
        /*centerNoScore = new double[][] {{60, 0, 0, 1}, {60, 0, -(Math.PI * .5), 0}, {60, -132, 0, 1}, {60, -132, -(Math.PI * .5), 0},
                                        {168, -132, -(Math.PI * .5), 1}, {168, -132, -(Math.PI * .5), 0}, {168, -132, -(Math.PI * .5), 0}};*/

        //score on right switch
        rightScoreR = new double[][] {{168, 0, 0, 1}, {168, 0, (Math.PI * .5), 0}, {168, 0, (Math.PI * .5), 0}};
        /*leftScoreR = new double[][] {{258, 0, 0, 1}, {258, 0, -(Math.PI * .5), 0}, {258, -242, 0, 1}, {258, -242, -Math.PI, 0},
                                     {168, -242, 0, 1}, {168, -242, (Math.PI * .5), 0}, *//*lift arm*//* {168, -182, 0, 1} *//*drop cube*//*}; //add indexes */
        centerScoreR = new double[][]{{130, -108, 0, 1}, {168, -108, 0, 1}, {168, -108, (Math.PI * .5), 0}};

        //score on left switch
        /*rightScoreL = new double[][] {{240, 0, 0, 1}, {240, 0, (Math.PI * .5), 0}, {240, -242, 0, 1}, {240, -242, -Math.PI, 0},
                                      {168, -242, 0, 1}, {168, -242, -(Math.PI * .5), 0}, *//*lift arm*//* {168, -182, 0, 1} *//*drop cube*//*}; //add indexes*/
        leftScoreL = new double[][] {{168, 0, 0, 1}, {168, 0, -(Math.PI * .5), 0}, {168, 0, -(Math.PI * .5), 0}};
        centerScoreL = new double[][]{{130, 108, 0, 1}, {168, 108, 0, 1}, {168, 108, -(Math.PI * .5), 0}};


        posistion = (int)Robot.driveSubsystem.robotPoisitionChooser.getSelected();
        scoreOn = (int)Robot.driveSubsystem.scoreOnSwitchChooser.getSelected();
        //switchPosistion = DriverStation.getInstance().getGameSpecificMessage();
        switchPosistion = "LRL";

        //Robot.liftSubsystem.autoLiftToSwitch(); //lift arm
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

        Robot.sensorSubsystem.getRobotPosistion();

        if (scoreOn > .5) {

            if (switchPosistion.startsWith("R")) {

                if (posistion == 3) {

                    path = rightScoreR;
                } else if (posistion == 1) {

                    path = leftRightNoScore;
                } else {

                    path = centerScoreR;
                }

            } else if (switchPosistion.startsWith("L")) {

                if (posistion == 3) {

                    path = leftRightNoScore;
                } else if (posistion == 1) {

                    path = leftScoreL;
                } else {

                    path = centerScoreL;
                }

            } else {

                if (posistion == 1 || posistion == 3) {

                    path = leftRightNoScore;
                } else {

                    if (switchPosistion.startsWith("R")) {

                        path = centerScoreR;
                    } else if (switchPosistion.startsWith("L")) {

                        path = centerScoreL;
                    }
                }
            }
        }
        Robot.driveSubsystem.followPaths(path, Robot.sensorSubsystem.X,Robot.sensorSubsystem.Y, Robot.sensorSubsystem.gyroAngle);

        SmartDashboard.putNumber("Robot Sensor X: ", Robot.sensorSubsystem.X);
        SmartDashboard.putNumber("Robot Sensor Y: ", Robot.sensorSubsystem.Y);
        SmartDashboard.putNumber("Robot Gyro Angle: ", Robot.sensorSubsystem.gyroAngle);

        SmartDashboard.putNumber("Robot Target X: ", Robot.driveSubsystem.targetX);
        SmartDashboard.putNumber("Robot Target Y: ", Robot.driveSubsystem.targetY);

        /*Robot.driveSubsystem.headingControl(30, 0, 0, true, Robot.sensorSubsystem.X,
                                            Robot.sensorSubsystem.Y, Robot.sensorSubsystem.gyroAngle);*/

        Robot.driveSubsystem.headingControl(Robot.driveSubsystem.targetX, Robot.driveSubsystem.targetY,
                                            Robot.driveSubsystem.targetHeading, Robot.driveSubsystem.targetMode,
                                            Robot.sensorSubsystem.X, Robot.sensorSubsystem.Y, Robot.sensorSubsystem.gyroAngle);

        /*driveSubsystem.setSpeed(driveSubsystem.leftPI(driveSubsystem.leftVelocity),
                                driveSubsystem.rightPI(driveSubsystem.rightVelocity));*/

        driveSubsystem.setSpeed(driveSubsystem.leftPI(0),driveSubsystem.rightPI(6));
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
