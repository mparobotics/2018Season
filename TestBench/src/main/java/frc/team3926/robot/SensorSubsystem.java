package frc.team3926.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Gyro;


/**
 *
 */
public class SensorSubsystem extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    Encoder enc;
    private DigitalInput limitSwitch;

    private DigitalInput liftSwitchUp;
    private DigitalInput liftSwitchDown;

    private Servo wingServo = new Servo (RobotMap.WING_SERVO_ID);
    static Encoder rightDriveEnc;
    static Encoder leftDriveEnc;

    static Gyro    gyro; // TODO must set some values for this to set the gyro up


    public void initDefaultCommand() {

        //new EncoderCommand();
        limitSwitch = new DigitalInput(RobotMap.LIMIT_SWITCH);

        gyro = new AnalogGyro(0);
        //gyro.setSensitivity(RobotMap.GYRO_VOLTS_PER_DEG_PER_SEC);
        // TODO find out why setSensitivity is not being recognized and then uncomment sensitivity setting

        //liftSwitchUp = new DigitalInput(RobotMap.UP_LIFT_LIMIT_SWITCH);
        //liftSwitchDown = new DigitalInput(RobotMap.DOWN_LIFT_LIMIT_SWITCH);

        enc = new Encoder(RobotMap.ENCODER_ID_1, RobotMap.ENCODER_ID_2, false, Encoder.EncodingType.k4X);
        enc.setMaxPeriod(.05);
        enc.setMinRate(10);
        enc.setDistancePerPulse(20);
        enc.setReverseDirection(false);
        enc.setSamplesToAverage(20); //TODO test to find better value

        // defining encoders and determining various values

        rightDriveEnc = new Encoder(RobotMap.RIGHT_DRIVE_ENC_PORT_ONE,RobotMap.RIGHT_DRIVE_ENC_PORT_TWO,false,
                                    Encoder.EncodingType.k4X);
        rightDriveEnc.setMaxPeriod(RobotMap.DRIVE_ENC_MAX_PERIOD);
        rightDriveEnc.setMinRate(RobotMap.DRIVE_ENC_MIN_RATE);
        rightDriveEnc.setDistancePerPulse(RobotMap.DRIVE_ENC_DISTANCE_PER_PULSE);
        rightDriveEnc.setReverseDirection(false);
        rightDriveEnc.setSamplesToAverage(RobotMap.DRIVE_ENC_AVERAGE_SAMPLES);

        leftDriveEnc = new Encoder(RobotMap.LEFT_DRIVE_ENC_PORT_ONE,RobotMap.LEFT_DRIVE_ENC_PORT_TWO,false,
                                   Encoder.EncodingType.k4X);
        leftDriveEnc.setMaxPeriod(RobotMap.DRIVE_ENC_MAX_PERIOD);
        leftDriveEnc.setMinRate(RobotMap.DRIVE_ENC_MIN_RATE);
        leftDriveEnc.setDistancePerPulse(RobotMap.DRIVE_ENC_DISTANCE_PER_PULSE);
        leftDriveEnc.setReverseDirection(false);
        leftDriveEnc.setSamplesToAverage(RobotMap.DRIVE_ENC_AVERAGE_SAMPLES);

    }

    public void servoWings() {

        if(Robot.oi.LB.get()) { // TODO wings

            wingServo.setAngle(180);
        }
    }
    public double Encoder(String output) {

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

    }

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

    public static double RightDriveEncoder(String output) {

        double outputValue = 0;

        switch(output) {

            case "Raw Count" :
                outputValue = rightDriveEnc.getRaw();
                break;

            case "Distance" :
                outputValue = rightDriveEnc.getDistance() * RobotMap.RADIUS_OF_ROTATION;
                break;

            case "Rate" :
                outputValue = rightDriveEnc.getRate();
                break;

        }

        return outputValue;

    }

    public static double LeftDriveEncoder(String output) {

        double outputValue = 0;

        switch(output) {

            case "Raw Count" :
                outputValue = leftDriveEnc.getRaw();
                break;

            case "Distance" :
                outputValue = leftDriveEnc.getDistance() * RobotMap.RADIUS_OF_ROTATION;
                break;

            case "Rate" :
                outputValue = leftDriveEnc.getRate();
                break;

        }

        return outputValue;

    }

    public static double ResetRightDriveEncoder(){

        rightDriveEnc.reset();

        return 0;

    }

    public static double ResetLeftDriveEncoder(){

        leftDriveEnc.reset();

        return 0;

    }

    public static double ResetGyro(){

        gyro.reset();

        return 0;

    }

    public static double Gyro(String output){

        switch(output){

            /*case "angle":
                gyro.getAngle();
                TODO ^ according to the documentation about getAngle
                TODO "The angle is based on the currentaccumulator value corrected by the oversampling rate, the gyro
                TODO type and the A/D calibration values"
                TODO so those values should be set
                break;*/

        }

        return 0;
    }

    public boolean turnedAngle(double angle){

        if(angle >= 0.0){ // tests if the robot is moving to a positive angle

            return gyro.getAngle() >= angle; //returns whether the robot has turned as much as or more than required


        } else { //runs if the robot is moving to a negative angle

            return gyro.getAngle() <= angle; //returns whether the robot has turned as much as or more than required

        }

    }

 }

