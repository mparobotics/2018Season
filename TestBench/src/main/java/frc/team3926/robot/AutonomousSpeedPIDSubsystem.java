package frc.team3926.robot;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;

/**
 * this class is supposed to control the speed during autonomous.
 */
public class AutonomousSpeedPIDSubsystem extends PIDSubsystem {

    Accelerometer accel;


    // Initialize your subsystem here
    public AutonomousSpeedPIDSubsystem() {

        super("AutonomousSpeedPIDSubsystem", RobotMap.AUTONOMOUS_SPEED_P, RobotMap.AUTONOMOUS_SPEED_I, RobotMap
                .AUTONOMOUS_SPEED_D);
        setAbsoluteTolerance(RobotMap.AUTONOMOUS_SPEED_TOLERANCE);
        // Use these to get going:
        // setSetpoint() -  Sets where the PID controller should move the system
        //                  to
        // enable() - Enables the PID controller.

    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());

        accel = new BuiltInAccelerometer();
        accel = new BuiltInAccelerometer(Accelerometer.Range.k4G);

    }

    protected double returnPIDInput() {
        // Return your input value for the PID loop
        // e.g. a sensor, like a potentiometer:
        // yourPot.getAverageVoltage() / kYourMaxVoltage;
        return accel.getX();

    }

    protected void usePIDOutput(double output) {
        // Use output to drive your system, like a motor
        // e.g. yourMotor.set(output);
    }

}
