package frc.team3926.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
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

    double leftStickYaxis;
    double rightStickYaxis;
    double leftSpeed;
    double rightSpeed;
    double ESC;
    double ESP;

    Encoder rightDriveEnc;
    Encoder leftDriveEnc;

    private Gyro gyro;

    Preferences prefs;

    public void initDefaultCommand() {

        prefs = Preferences.getInstance();

        ESC = prefs.getDouble("Exponential Speed Constant", 0.0); //must be kept between 0 and 1
        //ESC = RobotMap.EXPONENTIAL_SPEED_CONSTANT;
        ESP = prefs.getDouble("Exponential Speed Power",0.0); // must not go lower than 0
        //ESP = RobotMap.EXPONENTIAL_SPEED_POWER;
        FR = new WPI_TalonSRX(RobotMap.FRONT_RIGHT);
        BR = new WPI_TalonSRX(RobotMap.BACK_RIGHT);
        FL = new WPI_TalonSRX(RobotMap.FRONT_LEFT);
        BL = new WPI_TalonSRX(RobotMap.BACK_LEFT);

        m_right = new SpeedControllerGroup(FR, BR);
        m_left = new SpeedControllerGroup(FL, BL);
        m_myRobot = new DifferentialDrive(m_left, m_right);
        SmartDashboard.putData(Robot.driveSubsystem);

        //rightDriveEnc = new Encoder(RobotMap.RIGHT_DRIVE_ENC_PORT_ONE, RobotMap.RIGHT_DRIVE_ENC_PORT_TWO, false,
                                    //Encoder.EncodingType.k4X);
        //leftDriveEnc = new Encoder(RobotMap.LEFT_DRIVE_ENC_PORT_ONE, RobotMap.LEFT_DRIVE_ENC_PORT_TWO, false,
                                   //Encoder.EncodingType.k4X);

    }


    public void teleopDrive() {

        // takes in data from the position of the joysticks to determine speeds for drive system. As a joystick is
        // pushed forward, the speed goes up exponentially (works with pushing joystick backward and going in reverse
        // in the same way
        exponentialDrive();
        //m_myRobot.tankDrive(Robot.oi.leftStick.getY(), Robot.oi.rightStick.getY());





    }

    // takes in data from the position of the joysticks to determine speeds for drive system. As a joystick is
    // pushed forward, the speed goes up exponentially (works with pushing joystick backward and going in reverse
    // in the same way

    public double exponentialDrive(){

        /* if (ESC > 1){

            ESC = 1;

        } else if (ESC < 0){

            ESC = 0;

        }

        if(ESP < 0){

            ESP = 0;

        }*/


        ESC = RobotMap.EXPONENTIAL_SPEED_CONSTANT;
        ESP = RobotMap.EXPONENTIAL_SPEED_POWER;
        SmartDashboard.putNumber("ESC",ESC);


        leftStickYaxis = Robot.oi.leftStick.getY();
        rightStickYaxis = Robot.oi.rightStick.getY();

        // plugs the y axis of the joysticks into a cubic equation, resulting in the speed
        leftSpeed = ESC * (Math.pow(leftStickYaxis, ESP))
                    + (1 - ESC) * leftStickYaxis;
        rightSpeed = ESC * (Math.pow(rightStickYaxis, ESP))
                     + (1 - ESC) * rightStickYaxis;

        m_myRobot.tankDrive(-leftSpeed, -rightSpeed);

        return 0;

    }

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

    public double setSpeed(double leftSpeed, double rightSpeed) {

        m_myRobot.tankDrive(leftSpeed, rightSpeed);
        return 0;

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

    public double goForward(int distance){

        rightDriveEnc.reset();
        leftDriveEnc.reset();

        double rightError;
        double rightPV = 0; // value tbd
        double rightIntegral = 0; // value tbd
        double rightDerivative;
        double rightOutput;
        double rightPreError = 0; // value tbd
        double rightDt = 0; // value tdb

        double leftError;
        double leftPV = 0; // value tbd
        double leftIntegral = 0; // value tbd
        double leftDerivative;
        double leftOutput;
        double leftPreError = 0; // value tbd
        double leftDt = 0; // value tbd

        while(leftDriveEnc.getDistance() < distance){

            leftError = RobotMap.FORWARD_SPEED_SETPOINT - leftDriveEnc.getRate();

            leftIntegral = leftIntegral + (leftError * leftDt);

            leftDerivative = (leftError - leftPreError) / leftDt;

            leftOutput = (RobotMap.FORWARD_KP * leftError) + (RobotMap.FORWARD_KI * leftIntegral) + (RobotMap.FORWARD_KD * leftDerivative);

            leftPreError = leftError;

            FL.set(leftOutput);
            BL.set(leftOutput);

            rightError = RobotMap.FORWARD_SPEED_SETPOINT - rightDriveEnc.getRate();

            rightIntegral = rightIntegral + (rightError * rightDt);

            rightDerivative = (rightError - rightPreError) / rightDt;

            rightOutput = (RobotMap.FORWARD_KP * rightError) + (RobotMap.FORWARD_KI * rightIntegral)
                          + (RobotMap.FORWARD_KD * rightDerivative);

            rightPreError = rightError;

            FR.set(rightOutput);
            BR.set(rightOutput);

        }

        return 0;

    }

    public double turn(String direction){

        rightDriveEnc.reset();
        leftDriveEnc.reset();
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

        if(direction == "right"){

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

            leftError = leftSetPoint - leftDriveEnc.getRate();

            leftIntegral = leftIntegral + (leftError * leftDt);

            leftDerivative = (leftError - leftPreError) / leftDt;

            leftOutput = (RobotMap.TURNING_KP * leftError) + (RobotMap.TURNING_KI * leftIntegral)
                         + (RobotMap.TURNING_KD * leftDerivative);

            leftPreError = leftError;

            FL.set(leftOutput);
            BL.set(leftOutput);

            rightError = rightSetPoint - rightDriveEnc.getRate();

            rightIntegral = rightIntegral + (rightError * rightDt);

            rightDerivative = (rightError - rightPreError) / rightDt;

            rightOutput = (RobotMap.TURNING_KP * rightError) + (RobotMap.TURNING_KI * rightIntegral)
                          + (RobotMap.TURNING_KD * rightDerivative);

            rightPreError = rightError;

            FR.set(rightOutput);
            BR.set(rightOutput);

        }

        return 0;

    }

    public void hitSomething() {

        m_myRobot.tankDrive(0, 0);
        m_myRobot.tankDrive(-.5, -.5);
        Timer.delay(1);
        m_myRobot.tankDrive(0, 0);
        Timer.delay(3);

    }

}
