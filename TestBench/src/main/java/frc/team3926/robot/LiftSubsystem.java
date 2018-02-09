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

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.

        liftMotor = new WPI_TalonSRX(RobotMap.LIFT_MOTOR);

        ESC = RobotMap.EXPONENTIAL_SPEED_CONSTANT;
    }

    public void controlLiftTeleop(){

        rawAxis = Robot.oi.xboxController.getRawAxis(Robot.oi.xboxLeftAxis);
        liftMotor.set(ESC * (Math.pow(rawAxis, 3)) + (1 - ESC) * rawAxis);
    }

    public void autoLiftToSwitch(){ //TODO test

        liftMotor.set(.10);
        Timer.delay(1);
    }

    public void setLiftSpeed(double speed) {

        liftMotor.set(speed);
    }
}

