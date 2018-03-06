package frc.team3926.robot.subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3926.robot.command.teleop.DriveCommand;
import frc.team3926.robot.Robot;
import frc.team3926.robot.RobotMap;

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
    public SendableChooser driveChooser;

    public DriverStation ds;
    public double time;
    public int position;

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

        ESC = RobotMap.OI_GAIN;//must be kept between 0 and 1

        driveChooser = new SendableChooser();

        driveChooser.addDefault("Default Drive ", defaultDriveID);
        driveChooser.addObject("Half Drive ", halfDriveID);
        driveChooser.addObject("Straight Drive", straightDriveID);

        SmartDashboard.putData("Drive Mode: ", driveChooser);

        position = DriverStation.getInstance().getLocation();

        SmartDashboard.putNumber("Position: ", position);

        setDefaultCommand(new DriveCommand());
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

        leftStickYaxis = Robot.oi.getJoystickLeftY();
        rightStickYaxis = Robot.oi.getJoystickRightY();


        if (driveMode == halfDriveID) {

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
        }

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

        m_myRobot.tankDrive(0, 0);
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

}

