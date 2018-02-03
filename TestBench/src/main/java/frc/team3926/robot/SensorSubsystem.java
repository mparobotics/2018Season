package frc.team3926.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *import static frc.team3926.robot.Robot.*;
 * */
public class SensorSubsystem extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    private Encoder enc;
    //private AnalogInput limitSwitch;
    public void initDefaultCommand() {


    }

    public double Encoder(String output) {

        enc = new Encoder(RobotMap.ENCODER_ID_1, RobotMap.ENCODER_ID_2, false, Encoder.EncodingType.k4X);
        double outputValue = 0;

           enc.setMaxPeriod(.1);
           enc.setMinRate(10);
           enc.setDistancePerPulse(6.3); //TODO find better value
           enc.setReverseDirection(true);
           enc.setSamplesToAverage(7); //TODO test to find better value



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

   /* public boolean LimitSwitch(){
        Boolean isPressed = false;


        return isPressed;
    }*/
}

