package frc.team3926.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class OI {

    public Joystick rightStick;
    public Joystick leftStick;
    public Joystick xboxController;

    int xboxRightAxis;
    int xboxLeftAxis; //contols the lift

    public JoystickButton X; //controls winch motors
    public JoystickButton Y; //controls winch motors down
    public JoystickButton LB; //spits out cube (backwards)
    public JoystickButton RB; //pulls in cube (forewords)

    public JoystickButton halfSpeedTrigger; //controls half speed mode
    public JoystickButton straightModeTrigger; //controls straight mode

    OI() {

        rightStick = new Joystick(RobotMap.RIGHT_JOYSTICK);
        leftStick = new Joystick(RobotMap.LEFT_JOYSTICK);
        xboxController = new Joystick(RobotMap.XBOX_CONTROLLER);

        xboxLeftAxis = 1;

        X = new JoystickButton(xboxController, 3);
        Y = new JoystickButton(xboxController, 4);
        LB = new JoystickButton(xboxController, 5);
        RB = new JoystickButton(xboxController, 6);

        halfSpeedTrigger = new JoystickButton(rightStick, 1);
        straightModeTrigger = new JoystickButton(leftStick, 1);

    }
}
