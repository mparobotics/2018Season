package frc.team3926.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class LiftSubsystem extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    private WPI_TalonSRX liftMotor;
    private double ESC;
    private double rawAxis;

    private double leftAxis;

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.

        liftMotor = new WPI_TalonSRX(RobotMap.LIFT_MOTOR);
    }

    public void controlLiftTeleop(){

        leftAxis = Robot.oi.getXboxLeftY();

        liftMotor.set(leftAxis);

        /*if(Robot.sensorSubsystem.DownLimit()) { //TODO add back in when limit switches are added

            Robot.oi.setXboxRumble(true);
        } else {

            Robot.oi.setXboxRumble(false);
        }*/
    }

    public void autoLiftToSwitch(){

        liftMotor.set(.10);
        Timer.delay(1);
    }

    public void setLiftSpeed(double speed) {

        liftMotor.set(speed);

    }

}

