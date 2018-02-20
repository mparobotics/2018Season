package frc.team3926.robot.subsystem;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team3926.robot.RobotMap;

/**
 *
 */
public class Sensors extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    Encoder enc;
    private DigitalInput limitSwitch;

    private DigitalInput liftSwitchUp;
    private DigitalInput liftSwitchDown;

    private Servo wingServo;
    private double time;

    public void initDefaultCommand() {

        //new EncoderCommand();
        limitSwitch = new DigitalInput(RobotMap.LIMIT_SWITCH);

        //liftSwitchUp = new DigitalInput(RobotMap.UP_LIFT_LIMIT_SWITCH);
        //liftSwitchDown = new DigitalInput(RobotMap.DOWN_LIFT_LIMIT_SWITCH);

        enc = new Encoder(RobotMap.ENCODER_ID_1, RobotMap.ENCODER_ID_2, false, Encoder.EncodingType.k4X);
        enc.setMaxPeriod(.05);
        enc.setMinRate(10);
        enc.setDistancePerPulse(20);
        enc.setReverseDirection(false);
        enc.setSamplesToAverage(20); //TODO test to find better value

        wingServo = new Servo (RobotMap.WING_SERVO_ID);

    }
    public void deployWings() {

        wingServo.set(1);
    }
    public void resetWings() {

        wingServo.set(0);
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
 }

