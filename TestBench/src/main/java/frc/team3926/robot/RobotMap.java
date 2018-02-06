package frc.team3926.robot;

public class RobotMap {

    public static int LIMIT_SWITCH = 0;

    public static int ENCODER_ID_1   = 1;
    public static int ENCODER_ID_2   = 2;
    public static int ENCODER_MOTOR  = 1;

    public static double RADIUS_OF_ROTATION = 7.62; //cm

    public static int FRONT_RIGHT    = 8;
    public static int FRONT_LEFT     = 22;
    public static int BACK_RIGHT     = 7;
    public static int BACK_LEFT      = 0;

    public static int INTAKE_MOTOR_1 = 1; //change
    public static int INTAKE_MOTOR_2 = 2; //change

    public static int WINCH_MOTOR_1  = 3; //change
    public static int WINCH_MOTOR_2  = 4; //change

    public static int LIFT_MOTOR = 5; //change

    public static int RIGHT_JOYSTICK  = 0;
    public static int LEFT_JOYSTICK   = 1;
    public static int XBOX_CONTROLLER = 3;

    public static int CAMERA_RES_HIGHT = 480;
    public static int CAMERA_RES_WIDTH = 640;
    public static int FPS = 20;
    public static double EXPONENTIAL_SPEED_CONSTANT = 0.5; // a constant used for exponential speed growth as the
    // joystick goes forward
    public static double EXPONENTIAL_SPEED_POWER = 3; // must be a whole odd number


    public static double AUTONOMOUS_SPEED_TOLERANCE = 1; //tbd

    public static double AUTO_LINE_DISTANCE_METERS  = 1; //tbd
    public static String STARTING_POSITION          = "right"; //tbd
    public static String FREINDLY_SCALE_SIDE        = "right"; //tbd

    public static int FORWARD_KP                    = 1; //tbd
    public static int FORWARD_KI                    = 1; //tbd
    public static int FORWARD_KD                    = 1; //tbd
    public static int FORWARD_SPEED_SETPOINT        = 1; //tbd

    public static int TURNING_KP                    = 1; //tbd
    public static int TURNING_KI                    = 1; //tbd
    public static int TURNING_KD                    = 1; //tbd
    public static int TURNING_SPEED_SETPOINT        = 1; //tbd

    public static int LEFT_DRIVE_ENC_PORT_ONE       = 1; //tbd
    public static int LEFT_DRIVE_ENC_PORT_TWO       = 1; //tbd
    public static int RIGHT_DRIVE_ENC_PORT_ONE      = 1; //tbd
    public static int RIGHT_DRIVE_ENC_PORT_TWO      = 1; //tbd

}
