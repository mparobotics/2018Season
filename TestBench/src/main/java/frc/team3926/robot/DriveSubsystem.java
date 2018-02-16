package frc.team3926.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */

public class DriveSubsystem extends Subsystem {

    private WPI_TalonSRX         FR;
    private WPI_TalonSRX         BR;
    private WPI_TalonSRX         FL;
    private WPI_TalonSRX         BL;

    private SpeedControllerGroup m_right;
    private SpeedControllerGroup m_left;
    private DifferentialDrive    m_myRobot;

    private Gyro gyro;

    String currentOrientation = "forward";

    double XPosition          = RobotMap.STARTING_X_POSITION; // left side of the field is 0
    double YPosition          = RobotMap.STARTING_Y_POSITION; // starting side of the field is 0

    double rightPreError; // used to determine how much the PID should adjust. should be reset to zero after turn or
    // specific distance traveled
    double leftPreError; // used to determine how much the PID should adjust. should be reset to zero after turn or
    // specific distance traveled

    double rightIntegral;// used to determine how much the PID should adjust. should be reset to zero after turn or
    // specific distance traveled
    double leftIntegral; // used to determine how much the PID should adjust. should be reset to zero after turn or
    // specific distance traveled

    double rightDerivative; // used to determine how much the PID should adjust. should be reset to zero after turn or
    // specific distance traveled
    double leftDerivative; // used to determine how much the PID should adjust. should be reset to zero after turn or
    // specific distance traveled

    Timer rightCalcTimer;
    Timer leftCalcTimer;

    public void initDefaultCommand() {

        rightCalcTimer.start();
        leftCalcTimer.start();

        FR = new WPI_TalonSRX(RobotMap.FRONT_RIGHT);
        BR = new WPI_TalonSRX(RobotMap.BACK_RIGHT);
        FL = new WPI_TalonSRX(RobotMap.FRONT_LEFT);
        BL = new WPI_TalonSRX(RobotMap.BACK_LEFT);

        m_right = new SpeedControllerGroup(FR, BR);
        m_left = new SpeedControllerGroup(FL, BL);
        m_myRobot = new DifferentialDrive(m_left, m_right);
        SmartDashboard.putData(Robot.driveSubsystem);

        rightPreError = 0;
        leftPreError = 0;

        double rightDerivative = 0;
        double leftDerivative = 0;

        /*rightDriveEnc = new Encoder(RobotMap.RIGHT_DRIVE_ENC_PORT_ONE, RobotMap.RIGHT_DRIVE_ENC_PORT_TWO, false,
            Encoder.EncodingType.k4X);
        leftDriveEnc = new Encoder(RobotMap.LEFT_DRIVE_ENC_PORT_ONE, RobotMap.LEFT_DRIVE_ENC_PORT_TWO, false,
            Encoder.EncodingType.k4X);*/

    }

    public void teleopDrive() {

        // takes in data from the position of the joysticks to determine speeds for drive system. As a joystick is
        // pushed forward, the speed goes up exponentially (works with pushing joystick backward and going in reverse
        // in the same way
        m_myRobot.tankDrive(Robot.oi.exponentialDriveLeft(), Robot.oi.exponentialDriveRight());

        //m_myRobot.tankDrive(Robot.oi.leftStick.getY(), Robot.oi.rightStick.getY());

    }

    public double setSpeed(double leftSpeed, double rightSpeed) {

        m_myRobot.tankDrive(leftSpeed, rightSpeed);
        return 0;

    }

    /*public double goForward(int DesiredDistance) {

        SensorSubsystem.ResetRightDriveEncoder();
        SensorSubsystem.ResetLeftDriveEncoder();

        double rightOutput;
        double leftOutput;

        while (SensorSubsystem.LeftDriveEncoder("Distance") < DesiredDistance) {

            leftOutput = leftMotorPID(RobotMap.FORWARD_SPEED_SETPOINT, 0, RobotMap.FORWARD_KP, RobotMap.FORWARD_KI,
                                      RobotMap.FORWARD_KD, 0);
            rightOutput = rightMotorPID(RobotMap.FORWARD_SPEED_SETPOINT, 0, RobotMap.FORWARD_KP, RobotMap.FORWARD_KI,
                                        RobotMap.FORWARD_KD, 0);

            m_myRobot.tankDrive(leftOutput, rightOutput);

        }
        //this changes the coordinates of the robot based off the distance that the robot has just travelled
        changeCords(DesiredDistance);

        rightPreError = 0;
        leftPreError = 0;

        return 0;

    }*/

    public double driveForward(){

        double leftOutput;
        double rightOutput;

        leftOutput = leftMotorPID(RobotMap.FORWARD_SPEED_SETPOINT, RobotMap.FORWARD_KP, RobotMap.FORWARD_KI,
                                  RobotMap.FORWARD_KD); // TODO determine value of the start integral and Dt
        rightOutput = rightMotorPID(RobotMap.FORWARD_SPEED_SETPOINT,RobotMap.FORWARD_KP, RobotMap.FORWARD_KI,
                                    RobotMap.FORWARD_KD); // TODO determine value of the start integral and Dt

        m_myRobot.tankDrive(leftOutput, rightOutput);

        return 0;

    }

    //resets how much the robot thinks it is off and how much it needs to adjust.
    public double resetErrorsAndIntegrals(){

        rightPreError = 0;
        leftPreError  = 0;

        rightIntegral = 0;
        leftIntegral  = 0;

        rightDerivative = 0;
        leftDerivative = 0;

        return 0;

    }

    public double turnOffMotors(){

        m_myRobot.tankDrive(0,0);

        return 0;

    }

    // takes in a P value, I value, D value, dt ,stating Integra, and setpoint to determine the speed of the right side
    // in autonomous
    public double rightMotorPID(double setPoint,double p, double i, double d){

        double dt = rightCalcTimer.get(); //sets dt to the amount of time since the calculation was last done
        double output;
        double error;//mesures how off the robots speed is from its target speed. used to set preError. The next
        // time this method is run, the preError will be used to determine how much the signal to the motors will be
        double derivative;

        error = setPoint - SensorSubsystem.RightDriveEncoder("Rate");

        rightIntegral = rightIntegral + (error * dt);

        if(rightIntegral > RobotMap.PID_INTEGRAL_CAP){

            rightIntegral = RobotMap.PID_INTEGRAL_CAP;

        } else if (rightIntegral < RobotMap.PID_INTEGRAL_MINIMUM){

            rightIntegral = RobotMap.PID_INTEGRAL_MINIMUM;

        }

        rightDerivative = (error - rightPreError) / dt;

        output = (p * error) + (i * rightIntegral) + (d * rightDerivative);

        rightPreError = error;

        rightCalcTimer.reset();

        return output;

    }

    // takes in a P value, I value, D value, dt ,stating Integra, and setpoint to determine the speed of the left side
    // in autonomous
    public double leftMotorPID(double setPoint,double p, double i, double d){

        double dt = leftCalcTimer.get(); //sets dt to the amount of time since the calculation was last done
        double output;
        double error; //mesures how off the robots speed is from its target speed. used to set preError. The next
        // time this method is run, the preError will be used to determine how much the signal to the motors will be

        error = setPoint - SensorSubsystem.RightDriveEncoder("Rate");

        leftIntegral = leftIntegral + (error * dt);

        if(leftIntegral > RobotMap.PID_INTEGRAL_CAP){

            leftIntegral = RobotMap.PID_INTEGRAL_CAP;

        } else if (leftIntegral < RobotMap.PID_INTEGRAL_MINIMUM){

            leftIntegral = RobotMap.PID_INTEGRAL_MINIMUM;

        }

        leftDerivative = (error - leftPreError) / dt;

        output = (p * error) + (i * leftIntegral) + (d * leftDerivative);

        leftPreError = error;

        leftCalcTimer.reset();

        return output;

    }

    //turns a specific direction depending on the angle. Is run again and again by the command
    public double turn(double angle){

        double rightSetPoint;
        double leftSetPoint;

        double rightSpeed;
        double leftSpeed;

        if(angle >= 0){

            rightSetPoint = -RobotMap.TURNING_SPEED_SETPOINT;
            leftSetPoint = RobotMap.TURNING_SPEED_SETPOINT;

        } else {

            rightSetPoint = RobotMap.TURNING_SPEED_SETPOINT;
            leftSetPoint = -RobotMap.TURNING_SPEED_SETPOINT;

        }

        rightSpeed = rightMotorPID(rightSetPoint, RobotMap.TURNING_KP,  RobotMap.TURNING_KI,  RobotMap.TURNING_KD);
        leftSpeed = leftMotorPID(leftSetPoint, RobotMap.TURNING_KP,  RobotMap.TURNING_KI,  RobotMap.TURNING_KD);

        m_myRobot.tankDrive(leftSpeed,rightSpeed);

        return 0;

    }

    //this changes the coordinates of the robot based off the distance that the robot has just travelled
    /*public double changeCords(double distance){

        switch (currentOrientation){

            case "forward":
                YPosition += distance;
                break;

            case "right":
                XPosition += distance;
                break;

            case "backward":
                YPosition -= distance;
                break;

            case "left":
                XPosition -= distance;
                break;

        }

        return 0;

    }

    //turns to a specific orientation. only words for turning 90 or -90 degrees from current position
    public double turnOrientation(String desiredOrienation){

        //sees if the robot has to move right to get to its destination
        if(   (currentOrientation.equals("forward")  && desiredOrienation.equals("right")   )
           || (currentOrientation.equals("right")    && desiredOrienation.equals("backward"))
           || (currentOrientation.equals("backward") && desiredOrienation.equals("left")    )
           || (currentOrientation.equals("left")     && desiredOrienation.equals("forward") ) ){

            turnDirection("right");

            //sees if the robot has to move left to get to its destination
        } else if(   (currentOrientation.equals("forward")  && desiredOrienation.equals("left")    )
                  || (currentOrientation.equals("left")     && desiredOrienation.equals("backward"))
                  || (currentOrientation.equals("backward") && desiredOrienation.equals("right")   )
                  || (currentOrientation.equals("right")    && desiredOrienation.equals("forward") ) ){

            turnDirection("left");

        }

        return 0;

    }

    public double turnDirection(String turningDirection){

        SensorSubsystem.ResetRightDriveEncoder();
        SensorSubsystem.ResetLeftDriveEncoder();
        gyro.reset();

        double rightError;
        double rightIntegral = 0; // value tbd
        double rightDerivative;
        double rightOutput;
        double rightPreError = 0; // value tbd
        double rightDt = 0; // value tdb

        double leftError;
        double leftIntegral = 0; // value tbd
        double leftDerivative;
        double leftOutput;
        double leftPreError = 0; // value tbd
        double leftDt = 0; // value tbd

        double rightSetPoint;
        double leftSetPoint;

        double angle;

        if( turningDirection.equals("right") ){

            rightSetPoint = -RobotMap.TURNING_SPEED_SETPOINT;
            leftSetPoint = RobotMap.TURNING_SPEED_SETPOINT;

        } else {

            rightSetPoint = RobotMap.TURNING_SPEED_SETPOINT;
            leftSetPoint = -RobotMap.TURNING_SPEED_SETPOINT;

        }


        while(gyro.getAngle() < 90 && gyro.getAngle() > -90){
            //  ^ according to the documentation about getAngle
            // "The angle is based on the current accumulator value corrected by the oversampling rate, the gyro type
            // and the A/D calibration values"
            // so those values should be set

            leftError = leftSetPoint - SensorSubsystem.LeftDriveEncoder("Rate");

            leftIntegral = leftIntegral + (leftError * leftDt);

            leftDerivative = (leftError - leftPreError) / leftDt;

            leftOutput = (RobotMap.TURNING_KP * leftError) + (RobotMap.TURNING_KI * leftIntegral)
                         + (RobotMap.TURNING_KD * leftDerivative);

            leftPreError = leftError;

            rightError = rightSetPoint - SensorSubsystem.RightDriveEncoder("Rate");

            rightIntegral = rightIntegral + (rightError * rightDt);

            rightDerivative = (rightError - rightPreError) / rightDt;

            rightOutput = (RobotMap.TURNING_KP * rightError) + (RobotMap.TURNING_KI * rightIntegral)
                          + (RobotMap.TURNING_KD * rightDerivative);

            rightPreError = rightError;

            m_myRobot.tankDrive(leftOutput, rightOutput);

        }

        //this changes what directino the robot thinks it is facing after a turn. This will be used for the go to a
        // point method if and when it is implemented
        changeStoredDirection(turningDirection);

        return 0;

    }

    // this changes what direction the robot thinks it is facing after a turn. This will be used for the go to a point
    // method if and when it is implemented
    public double changeStoredDirection(String turningDirection){

        if(turningDirection.equals("right")){

            switch(turningDirection){

                case "forward":
                    currentOrientation = "right";
                    break;

                case "right":
                    currentOrientation = "backward";
                    break;

                case "backward":
                    currentOrientation = "left";
                    break;


                case "left":
                    currentOrientation = "forward";
                    break;

            }

        } else { //to be run if turning direction is left

            switch(turningDirection){

                case "forward":
                    currentOrientation = "left";
                    break;

                case "right":
                    currentOrientation = "forward";
                    break;

                case "backward":
                    currentOrientation = "right";
                    break;

                case "left":
                    currentOrientation = "backward";
                    break;

            }

        }

        return 0;

    }*/

    public void hitSomething() {

        m_myRobot.tankDrive(0, 0);
        m_myRobot.tankDrive(-.5, -.5);
        Timer.delay(1);
        m_myRobot.tankDrive(0, 0);
        Timer.delay(3);

    }

    /*public double setRightSpeed(int distance){

        rightDriveEnc.reset();

        int rightError;
        int rightPV = 0;
        int rightIntegral = 0;
        int rightDerivative;
        int rightOutput = 0;
        int rightPreError = 0;
        int rightDt = 0;

        while(rightDriveEnc.getDistance() < distance) {

            rightError = RobotMap.FORWARD_SPEED_SETPOINT - rightPV;

            rightIntegral = rightIntegral + (rightError * rightDt);

            rightDerivative = (rightError - rightPreError) / rightDt;

            rightOutput = (RobotMap.FORWARD_KP * rightError) + (RobotMap.FORWARD_KI * rightIntegral)
                          + (RobotMap.FORWARD_KD * rightDerivative);

            rightPreError = rightError;

            FR.set(rightOutput);
            BR.set(rightOutput);
        }

        return 0;

    }*/

    /*public double setLeftSpeed(int distance){

        leftDriveEnc.reset();

        int leftError;
        int leftPV = 0; // value tbd
        int leftIntegral = 0; // value tbd
        int leftDerivative;
        int leftOutput;
        int leftPreError = 0; // value tbd
        int leftDt = 0; // value tbd

        while(leftDriveEnc.getDistance() < distance){

            leftError = RobotMap.FORWARD_SPEED_SETPOINT - leftPV;

            leftIntegral = leftIntegral + (leftError * leftDt);

            leftDerivative = (leftError - leftPreError) / leftDt;

            leftOutput = (RobotMap.FORWARD_KP * leftError) + (RobotMap.FORWARD_KI * leftIntegral) + (RobotMap.FORWARD_KD * leftDerivative);

            leftPreError = leftError;

            FL.set(leftOutput);
            BL.set(leftOutput);

        }

        return 0;

    }*/

    /*public void autonomous(double speed1, double speed2, double speed3, double speed4, double speed5,  double speed6,
                           double speed7, double speed8, double speed9, double speed10,
                           double delay1, double delay2, double delay3, double delay4, double delay5) {
            //some of the things may be zero
        //if (desiredSwitchOnRight) {
            m_myRobot.tankDrive(speed1, speed2);
            Timer.delay(delay1);
            m_myRobot.tankDrive(speed3, speed4);
            Timer.delay(delay2);
            m_myRobot.tankDrive(speed5, speed6);
            Timer.delay(delay3);
            m_myRobot.tankDrive(speed7, speed8);
            Timer.delay(delay4);
            m_myRobot.tankDrive(speed9, speed10);
            Timer.delay(delay5);


        */
            /* if (centerPosition) {

            m_myRobot.tankDrive(.5, .5);
                Timer.delay(3);
                m_myRobot.tankDrive(-.5, .5);
                Timer.delay(.5);
                m_myRobot.tankDrive(.5, .5);
                Timer.delay(2);
                m_myRobot.tankDrive(.5, -.5);
                Timer.delay(.5);
                m_myRobot.tankDrive(.5, .5);
                Timer.delay(1);

            }
            else if (rightPosition) {

                m_myRobot.tankDrive(.5, .5);
                Timer.delay(6);
            }
            else if (leftPosition) {

                m_myRobot.tankDrive(.5, .5);
                Timer.delay(6);
            } */

        /*  } else {
                if (centerPosition) {

                }
                else if (rightPosition) {


                }
                else if (leftPosition) {


                }
            } */
    /*
         m_myRobot.tankDrive(0, 0);
    }*/

}
