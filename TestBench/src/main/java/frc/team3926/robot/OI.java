
package frc.team3926.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Joystick;

public class OI {


    public Joystick rightStick;
    public Joystick leftStick;
    public Joystick xboxController;

    int         xboxRightAxis;
    int         xboxLeftAxis;

    AnalogInput limitSwitch;

    OI() {

        rightStick = new Joystick(RobotMap.RIGHT_JOYSTICK);
        leftStick = new Joystick(RobotMap.LEFT_JOYSTICK);

        limitSwitch = new AnalogInput(RobotMap.LIMIT_SWITCH);

    }
}
