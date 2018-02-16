package frc.team3926.robot;

public class RobotMap {
////////////////////////////test sensors and motors////////////////////////////////////////////////////////////////////
    public static final int LIMIT_SWITCH = 0;

    public static final double RADIUS_OF_ROTATION = 7.62; //cm

////////////////////////////drive train motors/////////////////////////////////////////////////////////////////////////
    //ids for drive motors
    public static final int FRONT_RIGHT = 8;  //(BMO)
    public static final int FRONT_LEFT = 22;
    public static final int BACK_RIGHT = 7;
    public static final int BACK_LEFT = 0;

////////////////////////////intake/////////////////////////////////////////////////////////////////////////////////////
    //id for intake motors
    public static final int INTAKE_MOTOR_1 = 1; //TODO change id, left,   25% to 30% speed
    public static final int INTAKE_MOTOR_2 = 2; //TODO change id, right,  25% to 30% speed
    public static final double INTAKE_SPEED = .30;

////////////////////////////winch//////////////////////////////////////////////////////////////////////////////////////
    public static final int WINCH_MOTOR_1 = 3; //TODO change id,      90% speed
    public static final int WINCH_MOTOR_2 = 4; //TODO change id,      90% speed
    public static final double WINCH_SPEED = .90;

////////////////////////////lift///////////////////////////////////////////////////////////////////////////////////////
    //id for lift motor
    public static final int LIFT_MOTOR = 5; //TODO change id, adjustable up to 75% speed
    //limit switch ids
    public static final int UP_LIFT_LIMIT_SWITCH = 4; //TODO change id
    public static final int DOWN_LIFT_LIMIT_SWITCH = 2; //TODO change id

////////////////////////////joysticks////////////////////////////////////////////////////////////////////////////////..
    // joystick ids
    public static final int RIGHT_JOYSTICK = 0;
    public static final int LEFT_JOYSTICK = 1;
    public static final int XBOX_CONTROLLER = 3;
    //id for button which halfs speed
    public static final int HALF_DRIVE_BUTTON = 1; //TODO implement half drive button

////////////////////////////camera////////////////////////////////////////////////////////////////////////////////////.
    //camera resolution height
    public static final int    CAMERA_RES_HIGHT            = 480;
    //camera resolution width
    public static final int    CAMERA_RES_WIDTH            = 640;
    //camera frames per second
    public static final int    FPS                         = 20;

////////////////////////////exponential drive//////////////////////////////////////////////////////////////////////////
    //default for the constant in the exponential drive equation. overridden by robotpreferences
    public static final double EXPONENTIAL_SPEED_CONSTANT  = .5; //must be between 0 and 1
    //default for the constant in the exponential drive equation. overridden by robotpreferences
    public static final double EXPONENTIAL_SPEED_DEAD_BAND = .1; //must be between 0 and 1

////////////////////////////PID////////////////////////////////////////////////////////////////////////////////////////
    public static int    FORWARD_KP                        = 1; //TODO value tbd
    public static int    FORWARD_KI                        = 1; //TODO value tbd
    public static int    FORWARD_KD                        = 0; //TODO value tbd
    public static int    FORWARD_SPEED_SETPOINT            = 1; //TODO value tbd

    public static int    TURNING_KP                        = 1; //TODO value tbd
    public static int    TURNING_KI                        = 1; //TODO value tbd
    public static int    TURNING_KD                        = 0; //TODO value tbd
    public static int    TURNING_SPEED_SETPOINT            = 1; //TODO value tbd

    public static double PID_INTEGRAL_CAP                  = 1;
    public static double PID_INTEGRAL_MINIMUM              = -1;

    ////////////////////////////drive encoders/////////////////////////////////////////////////////////////////////////
    //ports for the encoders on the driving motors
    public static int LEFT_DRIVE_ENC_PORT_ONE              = 1; //TODO value tbd
    public static int LEFT_DRIVE_ENC_PORT_TWO              = 2; //TODO value tbd
    public static int RIGHT_DRIVE_ENC_PORT_ONE             = 3; //TODO value tbd
    public static int RIGHT_DRIVE_ENC_PORT_TWO             = 4; //TODO value tbd
    //various values for the encoders on the driving motors
    public static double DRIVE_ENC_MAX_PERIOD              = 0.5; //TODO value tbd
    public static double DRIVE_ENC_MIN_RATE                = 10; //TODO value tbd
    public static double DRIVE_ENC_DISTANCE_PER_PULSE      = 20; //TODO value tbd
    //TODO units tbd
    public static boolean DRIVE_ENC_REVERSE_DIRECTION      = false;
    public static int DRIVE_ENC_AVERAGE_SAMPLES            = 20; //TODO value tbd

    ////////////////////////////other//////////////////////////////////////////////////////////////////////////////////////
    public static double AUTONOMOUS_SPEED_TOLERANCE        = 1; //TODO value tbd

    public static double AUTO_LINE_DISTANCE_METERS         = 1; //TODO value tbd
    public static String STARTING_POSITION                 = "right"; //TODO value tbd
    public static String FRIENDLY_SCALE_SIDE               = "right"; //TODO value tbd

    public static double STARTING_X_POSITION               = 0; //TODO value tbd
    public static double STARTING_Y_POSITION               = 0; //TODO value tbd

    public static String FRIENDLY_SWITCH_SIDE              = "right";

    public static int    POWER_DISTRIBUTION_PANEL_ID       = 3;

    public static int    ENCODER_ID_1                      = 1;
    public static int    ENCODER_ID_2                      = 2;

    public static double GYRO_VOLTS_PER_DEG_PER_SEC        = 1; // determines the sensitivity of the gyro

}
