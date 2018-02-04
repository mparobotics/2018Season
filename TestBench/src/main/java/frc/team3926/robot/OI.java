package frc.team3926.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class OI {

    public Joystick rightStick;
    public Joystick leftStick;
    public Joystick xboxController;

    int xboxRightAxis;
    int xboxLeftAxis; //contols the lift

    JoystickButton X; //controls winch motors
    JoystickButton LB; //spits out cube (backwards)
    JoystickButton RB; //pulls in cube (forewords)

    OI() {

        rightStick = new Joystick(RobotMap.RIGHT_JOYSTICK);
        leftStick = new Joystick(RobotMap.LEFT_JOYSTICK);
        xboxController = new Joystick(RobotMap.XBOX_CONTROLLER);

        xboxLeftAxis = 1;

        X = new JoystickButton(xboxController, 3);
        LB = new JoystickButton(xboxController, 5);
        RB = new JoystickButton(xboxController, 6);

    }
}
