package frc.team3926.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3926.robot.Robot;
import frc.team3926.robot.RobotMap;

/**
 *
 */
public class Sensors extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    //Encoder enc;
    Accelerometer accel;
    ADXRS450_Gyro gyro; //ADXRS450 angular rate sensor
    private DigitalInput limitSwitch;

    private DigitalInput liftSwitchUp;
    private DigitalInput liftSwitchDown;

    private Servo wingServo;
    private double time;

    double xVal;
    double yVal;
    double zVal;

    double gyroAngle;

    public void initDefaultCommand() {

        //new EncoderCommand();
        limitSwitch = new DigitalInput(RobotMap.LIMIT_SWITCH);
        accel = new BuiltInAccelerometer();
        gyro = new ADXRS450_Gyro();

        accel = new BuiltInAccelerometer(Accelerometer.Range.k4G);
        xVal = accel.getX();
        yVal = accel.getY();
        zVal = accel.getZ();

        gyro.reset();

        //liftSwitchUp = new DigitalInput(RobotMap.UP_LIFT_LIMIT_SWITCH);
        //liftSwitchDown = new DigitalInput(RobotMap.DOWN_LIFT_LIMIT_SWITCH);

       /* enc = new Encoder(RobotMap.ENCODER_ID_1, RobotMap.ENCODER_ID_2, false, Encoder.EncodingType.k4X);
        enc.setMaxPeriod(.05);
        enc.setMinRate(10);
        enc.setDistancePerPulse(20);
        enc.setReverseDirection(false);
        enc.setSamplesToAverage(20); //TODO test to find better value*/

        wingServo = new Servo (RobotMap.WING_SERVO_ID);


            WPI_TalonSRX BR = Robot.driveSubsystem.getMasterRight();
            WPI_TalonSRX BL = Robot.driveSubsystem.getMasterLeft();
            WPI_TalonSRX FR = Robot.driveSubsystem.getFollowerRight();
            WPI_TalonSRX FL = Robot.driveSubsystem.getFollowerLeft();

            FR.set(ControlMode.Follower, RobotMap.BMO_BACK_RIGHT);
            FL.set(ControlMode.Follower, RobotMap.BMO_BACK_LEFT);

            BR.setNeutralMode(NeutralMode.Brake);
            BL.setNeutralMode(NeutralMode.Brake);

            BR.configOpenloopRamp(RobotMap.RAMP_VALUE, 0);
            BL.configOpenloopRamp(RobotMap.RAMP_VALUE, 0);

            BR.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
            BL.configSelectedFeedbackSensor( FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10 );


        gyro.calibrate();
    }

    public void setClosedEncoders() {

        WPI_TalonSRX BR = Robot.driveSubsystem.getMasterRight();
        WPI_TalonSRX BL = Robot.driveSubsystem.getMasterLeft();

        BR.configClosedloopRamp(RobotMap.RAMP_VALUE, 0);
        BL.configClosedloopRamp(RobotMap.RAMP_VALUE, 0);
    }
    public void setOpenEncoders() {

        WPI_TalonSRX BR = Robot.driveSubsystem.getMasterRight();
        WPI_TalonSRX BL = Robot.driveSubsystem.getMasterLeft();

        BR.configOpenloopRamp(RobotMap.RAMP_VALUE, 0);
        BL.configOpenloopRamp(RobotMap.RAMP_VALUE, 0);
    }

    public void deployWings() {

        wingServo.set(1);
    }
    public void resetWings() {

        wingServo.set(0);
    }
    /*public double Encoder(String output) {

        double outputValue = 0;

           switch(output) {

               case "Raw Count" :
                outputValue = enc.getRaw();

                break;

               case "Distance" :
                   outputValue = enc.getDistance() * RobotMap.RADIUS_OF_ROTATION;

                   break;

               case "Rate" :

                   outputValue = enc.getRate();
                   break;

           }

            return outputValue;
        }*/

      public boolean LimitSwitch(){

        Boolean isPressed = limitSwitch.get();

        return isPressed;
    }
    public boolean HeightLimit() {

        Boolean upIsPressed = liftSwitchUp.get();
        return upIsPressed;
    }
    public boolean DownLimit(){

        Boolean downIsPressed = liftSwitchDown.get();
        return downIsPressed;
    }

    public void printAccelerometerValues() {
        xVal = accel.getX();
        yVal = accel.getY();
        zVal = accel.getZ();

        SmartDashboard.putNumber("X Value: ", xVal);
        SmartDashboard.putNumber("Y Value: ", yVal);
        SmartDashboard.putNumber("Z Value: ", zVal);
    }

    public void printGyroAngle() {
        gyroAngle = gyro.getAngle();

        SmartDashboard.putNumber("Gyro Angle: ", gyroAngle);
    }

 }

