/*package frc.team3926.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; */

/**
 *
 *
public class LimitSwitchSubsystem extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    DigitalInput limitSwitch = new DigitalInput(RobotMap.LIMIT_SWITCH);

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());

        SmartDashboard.putBoolean("Limit Switch", limitSwitch.get());
    }

} */

