package frc.team3926.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class SensorSubsystem extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    private Encoder enc;
    private DigitalInput limitSwitch;
    public void initDefaultCommand() {
        limitSwitch = new DigitalInput(RobotMap.LIMIT_SWITCH);
        enc = new Encoder(RobotMap.ENCODER_ID_1, RobotMap.ENCODER_ID_2, false, Encoder.EncodingType.k4X);
    }

    public double Encoder(String output) {

        double outputValue = 0;

           enc.setMaxPeriod(.1);
           enc.setMinRate(10);
           enc.setDistancePerPulse(20);
           enc.setReverseDirection(false);
           enc.setSamplesToAverage(20); //TODO test to find better value



           switch(output) {

               case "Raw Count" :
                outputValue = enc.getRaw();

               case "Distance" :
                   outputValue = enc.getDistance();

               case "Rate" :
                   outputValue = enc.getRate();
           }

            return outputValue;
        }

      public boolean LimitSwitch(){

        Boolean isPressed = limitSwitch.get();

        return isPressed;
    }
 }

