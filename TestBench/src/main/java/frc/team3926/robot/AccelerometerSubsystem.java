package frc.team3926.robot;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;

/**
 *
 */
public class AccelerometerSubsystem extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    Accelerometer accel;

    double accelXVal;

    public void initDefaultCommand() {

        accel = new BuiltInAccelerometer();
        accel = new BuiltInAccelerometer(Accelerometer.Range.k4G);

        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }

    // may be need to changed to z or y acceleration depending on the oreintation of the robot and built in
    // accelerometer. The intention is that this varable being positive will mean that the robot is going forward.

    public double accelXval(){

        return accel.getX();

    }

}

