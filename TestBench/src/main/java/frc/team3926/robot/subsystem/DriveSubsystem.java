package frc.team3926.robot.subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3926.robot.Robot;
import frc.team3926.robot.RobotMap;
import frc.team3926.robot.command.teleop.DriveCommand;

/**
 *
 */
public class DriveSubsystem extends Subsystem {

    private WPI_TalonSRX FR;
    private WPI_TalonSRX BR;
    private WPI_TalonSRX FL;
    private WPI_TalonSRX BL;

    private SpeedControllerGroup m_right;
    private SpeedControllerGroup m_left;
    private DifferentialDrive m_myRobot;

    private double ESC;
    private double leftStickYaxis;
    private double rightStickYaxis;

    private double leftSpeed;
    private double rightSpeed;

    public int driveMode = 1;

    public int defaultDriveID = 1;
    public int halfDriveID = 2;
    public int straightDriveID = 3;
    public int leftID = 1;
    public int centerID = 2;
    public int rightID = 3;
    public int score = 1;
    public int noScore = 0;
    public SendableChooser driveChooser;
    public SendableChooser robotPoisitionChooser;
    public SendableChooser scoreOnSwitchChooser;

    public DriverStation ds;
    public double time;
    public int position;
    double rightIntegralError = 0;
    double leftIntegralError = 0;

    double headingIError = 0;

    double beepboop = 0;

    public double rightVelocity;
    public double leftVelocity;

    public double targetX;
    public double targetY;
    public double targetHeading;
    public boolean targetMode;

    public int pathIndex = 0;
    public double turningTime = 0;



    public void initDefaultCommand() {

        if(RobotMap.QBERT) {

            FR = new WPI_TalonSRX(RobotMap.FRONT_RIGHT);
            BR = new WPI_TalonSRX(RobotMap.BACK_RIGHT);
            FL = new WPI_TalonSRX(RobotMap.FRONT_LEFT);
            BL = new WPI_TalonSRX(RobotMap.BACK_LEFT);
            System.out.println("RowBoat");

        } else if(RobotMap.BMO) {

            FR = new WPI_TalonSRX(RobotMap.BMO_FRONT_RIGHT);
            BR = new WPI_TalonSRX(RobotMap.BMO_BACK_RIGHT);
            FL = new WPI_TalonSRX(RobotMap.BMO_FRONT_LEFT);
            BL = new WPI_TalonSRX(RobotMap.BMO_BACK_LEFT);
            System.out.println("BMO");


        } else {

            throw new IllegalStateException("Robot Map has no robot set to true");
        }

        m_right = new SpeedControllerGroup(FR, BR);
        m_left = new SpeedControllerGroup(FL, BL);
        m_myRobot = new DifferentialDrive(m_left, m_right);

        ESC = RobotMap.OI_GAIN_R;//must be kept between 0 and 1

        driveChooser = new SendableChooser();
        robotPoisitionChooser = new SendableChooser();
        scoreOnSwitchChooser = new SendableChooser();

        driveChooser.addDefault("Default Drive ", defaultDriveID);
        driveChooser.addObject("Half Drive ", halfDriveID);
        driveChooser.addObject("Straight Drive", straightDriveID);
        SmartDashboard.putData("Drive Mode: ", driveChooser);

        robotPoisitionChooser.addDefault("Left", leftID);
        robotPoisitionChooser.addObject("Center", centerID);
        robotPoisitionChooser.addObject("Right", rightID);
        SmartDashboard.putData("Robot Posistion: ", robotPoisitionChooser);

        scoreOnSwitchChooser.addDefault("Score on switch", score);
        scoreOnSwitchChooser.addObject("Don't Score on switch", noScore);
        SmartDashboard.putData("Scoring on Switch: ", scoreOnSwitchChooser);

       /* position = DriverStation.getInstance().getLocation();

        SmartDashboard.putNumber("Position: ", position);*/

        setDefaultCommand(new DriveCommand());
        //setDefaultCommand(new MoreAuto()); //TODO

    }

    //takes in data from the position of the joysticks to determine speeds for drive system. As a joystick is pushed forward, the speed goes up exponentially
    public void teleopDrive() {

        driveMode = (int) driveChooser.getSelected();

       /* if(Robot.sensorSubsystem.LimitSwitch()) { // just messing around w/ xbox rumble

            Robot.oi.xboxController.setRumble(kLeftRumble, 1.5);
            Robot.oi.xboxController.setRumble(kRightRumble, 1.5);
            Timer.delay(.075);
            Robot.oi.xboxController.setRumble(kLeftRumble, 0);
            Robot.oi.xboxController.setRumble(kRightRumble, 0);
        } */

       rightStickYaxis = Robot.oi.getJoystickRightY();
       leftStickYaxis = Robot.oi.getJoystickLeftY(); //TODO maybe add back in

        /*if (driveMode == halfDriveID) {

            if (Robot.oi.halfSpeedTrigger.get()) {

                leftSpeed = leftStickYaxis / 2;
                rightSpeed = rightStickYaxis / 2;
            } else {

                leftSpeed = leftStickYaxis;
                rightSpeed = rightStickYaxis;
            }
        }
        else if(driveMode == defaultDriveID) {

            leftSpeed = leftStickYaxis;
            rightSpeed = rightStickYaxis;

        }
        else if (driveMode == straightDriveID) {

            if (Robot.oi.straightModeTrigger.get()) {

                leftSpeed = leftStickYaxis;
                rightSpeed = leftStickYaxis;
            } else {

                leftSpeed = leftStickYaxis;
                rightSpeed = rightStickYaxis;
            }
        }*/

        rightSpeed = rightStickYaxis;
        leftSpeed = leftStickYaxis;

        m_myRobot.tankDrive(-leftSpeed, -rightSpeed);
    }

    public void autonomousStart() {

       setSpeed(-.5, -.5);
    }
    public void autonomousStop() {

       setSpeed(0, 0);
    }

    /*public void sketchyAuto() { //timer counting down

        while (ds.isAutonomous()) {

            //time = Timer.getMatchTime(); //FMS
            ds = DriverStation.getInstance();
            time = 15 - ds.getMatchTime();

            SmartDashboard.putNumber("Timer", time);

            if (position == 1 || position == 3) {

                if (time > 13.25) {

                    Robot.driveSubsystem.setAutoSpeed(.5, .5);
                } else {

                    Robot.driveSubsystem.setAutoSpeed(0, 0);
                }

            } else if (position == 2) {

                if (time > 14.5) {

                    Robot.driveSubsystem.setAutoSpeed(.5, .5);
                }
                if (14.5 > time && time > 14) {

                    Robot.driveSubsystem.setAutoSpeed(.5, - .5);
                }
                if (14 > time && time > 13.25) {

                    Robot.driveSubsystem.setAutoSpeed(.5, .5);
                }
                if (13.25 > time && time > 12.75) {

                    Robot.driveSubsystem.setAutoSpeed(- .5, .5);
                }
                if (12.75 > time && time > 11.5) {

                    Robot.driveSubsystem.setAutoSpeed(.5, .5);
                } else {

                    Robot.driveSubsystem.setAutoSpeed(0, 0);
                }
            }
        }
    }*/

    public void autoDriveStraight(double setTime) {

        time = Timer.getFPGATimestamp();

        if (Timer.getFPGATimestamp() - time < setTime) {

            setSpeed(.75, .75);
        } else {

            setSpeed(0, 0);
        }
    }

    public void autoTurn(String direction) {

        time = Timer.getFPGATimestamp();

        if (direction == "right") {

            if (Timer.getFPGATimestamp() - time < .75) {

                setSpeed(.5, -.5);
            } else {

                setSpeed(0, 0);
            }

        } else if (direction == "left") {

            if (Timer.getFPGATimestamp() - time < .75) {

                setSpeed(-.5, .5);
            } else {

                setSpeed(0, 0);
            }
        }
    }

    public double setSpeed(double leftSpeed, double rightSpeed) {

        m_myRobot.tankDrive(-leftSpeed, -rightSpeed);
        return 0;
    }
    public double setAutoSpeed(double leftSpeed, double rightSpeed) {

        m_myRobot.tankDrive(leftSpeed, rightSpeed);
        return 0;
    }

    /*public void hitSomething() {

        m_myRobot.tankDrive(0, 0); hhi
        m_myRobot.tankDrive(-.5, -.5);
        Timer.delay(1);
        m_myRobot.tankDrive(0, 0);
        Timer.delay(3);

    }*/

    public double getRightRPM() {

        return m_right.get();
    }

    public double getLeftRPM() {

        return m_left.get();
    }

    @Override
    public void periodic() {


    }

    public WPI_TalonSRX getMasterLeft() {

        return BL;
    }
    public WPI_TalonSRX getMasterRight() {

        return BR;
    }

    public WPI_TalonSRX getFollowerLeft() {

        return FL;
    }
    public WPI_TalonSRX getFollowerRight() {

        return FR;
    }

    public double rightPI(double targetSpeed) {

        double speed = Robot.sensorSubsystem.getRightWheelSpeed();
        //double targetSpeed = 6;
        double output;

        double e = targetSpeed - speed;
        rightIntegralError += e * .02;

        output = (RobotMap.RIGHT_P * e) + (RobotMap.RIGHT_I * rightIntegralError) + (RobotMap.RIGHT_F * targetSpeed);

        SmartDashboard.putNumber("actual speed: ", speed);
        SmartDashboard.putNumber("target speed: ", targetSpeed);
        SmartDashboard.putNumber("output: ", output);

        return -output;
    }

    public double leftPI(double targetSpeed) {

        double speed = Robot.sensorSubsystem.getLeftWheelSpeed();
        //double targetSpeed = 6;
        double output;

        double e = targetSpeed - speed;
        leftIntegralError += e * .02;

        output = (RobotMap.LEFT_P * e) + (RobotMap.LEFT_I * leftIntegralError) + (RobotMap.LEFT_F * targetSpeed);

        return -output;
    }

    public void uniControl(double v, double w) {

        rightVelocity = ((2 * v) + (w * RobotMap.WIDTH_BETWEEN_WHEELS)) / (2 * RobotMap.WHEEL_RADIUS);
        leftVelocity = ((2 * v) - (w * RobotMap.WIDTH_BETWEEN_WHEELS)) / (2 * RobotMap.WHEEL_RADIUS);

    }

    public void headingControl(double targetX, double targetY, double forcedHeading, boolean targetMode,
                               double actualX, double actualY, double actualHeading) {
        double targetHeading;
        double headingError;
        double w;
        double v; //linear velocity of robot
        double distance;

        double maxSpeed = 25; //in per sec
        double slowingDistance = 8;
        double stoppingDistance = 8;

        if (targetMode) {

            targetHeading = Math.atan2((targetY - actualY), (targetX - actualX));
            distance = getDistanceToPoint(targetX, targetY, actualX, actualY);

            if(distance > slowingDistance) {

                v = maxSpeed;
            } else if (distance > stoppingDistance) {

                v = (maxSpeed/ (slowingDistance - stoppingDistance)) * (distance - stoppingDistance);
            } else {

                v = 0;
            }

        } else {

            targetHeading = forcedHeading;
            v = 0;
        }

        SmartDashboard.putNumber("target heading: ", targetHeading);
        SmartDashboard.putNumber("actual heading: ", actualHeading);

        headingError = targetHeading - actualHeading;
        headingIError += headingError * .02; //.02 = excution period (20 milliseconds)

        w = (1.5 * headingError) + (1.5 * headingIError);

        uniControl(v,w);
    }

    public void followPaths(double[][] paths, double x, double y, double heading) {

         //TODO arms
         targetX = paths[pathIndex][0];
         targetY = paths[pathIndex][1];
         targetHeading = paths[pathIndex][2];
         targetMode = paths[pathIndex][3] > .5;

         int pathLenght = 2;

         /*targetX = 30;
         targetY = 0;
         targetHeading = 0;*/
         //targetMode = true;

        double distance = getDistanceToPoint(targetX, targetY, x, y);

        if (targetMode) {

            if (distance < 9 && pathIndex != pathLenght ) {

                pathIndex++;
            }
        } else {
            //move to the next point when at the right heading

            if (Math.abs(targetHeading - heading) < .1) {

                turningTime += .02;
            } else {

                turningTime = 0;
            }

            if(pathIndex != pathLenght && turningTime > .5) {

                pathIndex++;
                turningTime = 0;
            }
        }

        /*if (pathIndex == pathLenght) {   //shoot out cube in auto

            Robot.intakeArmSubsystem.autoReleaseCube();
        }*/
    }

    public double getDistanceToPoint(double targetX, double targetY, double actualX, double actualY) {
        double distance;

        distance = Math.sqrt(Math.pow((targetY - actualY), 2) + Math.pow((targetX - actualX), 2));

        return distance;
    }

}

