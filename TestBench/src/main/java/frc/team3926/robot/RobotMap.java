package frc.team3926.robot;

public class RobotMap {

    public static final boolean BMO = true;
    public static final boolean QBERT = false;
////////////////////////////////test sensors, motors, and values////////////////////////////////////////
    public static final int LIMIT_SWITCH = 0;
    public static final int ENCODER_ID_1 = 1;
    public static final int ENCODER_ID_2 = 2;

    public static final double RADIUS_OF_ROTATION = 7.62; //cm
    public static final int ENCODER_MOTOR = 2; //testing motor

///////////////////////////////drive train motors///////////////////////////////////////////////////////
    public static final int FRONT_RIGHT     = 2;  //(Qbert)
    public static final int FRONT_LEFT      = 6;
    public static final int BACK_RIGHT      = 5;
    public static final int BACK_LEFT       = 4;

    public static final int BMO_FRONT_RIGHT = 8; // (BMO)
    public static final int BMO_FRONT_LEFT  = 22;
    public static final int BMO_BACK_RIGHT  = 7;
    public static final int BMO_BACK_LEFT   = 2;

///////////////////////////////intake, winch, and lift motors and values////////////////////////////////
    public static final int INTAKE_MOTOR_1         = 3; //change id, left,   25% to 30% speed
    public static final int INTAKE_MOTOR_2         = 7; //change id, right,  25% to 30% speed
    public static final double INTAKE_SPEED        = .30;

    //public static int WINCH_MOTOR_1              = 8; //change id,      90% speed
    public static final int CLIMBING_WINCH_MOTOR_2 = 9; //change id,      90% speed
    public static final double WINCH_SPEED         = .90;

    public static final int LIFT_MOTOR             = 8; //change id, adjustable up to 75% speed
    public static final int UP_LIFT_LIMIT_SWITCH   = 4; // TODO change id
    public static final int DOWN_LIFT_LIMIT_SWITCH = 2; // TODO change id

//////////////////////////////joysticks/////////////////////////////////////////////////////////////////
    public static final int RIGHT_JOYSTICK = 0;
    public static final int LEFT_JOYSTICK = 1;
    public static final int XBOX_CONTROLLER = 2;

/////////////////////////////camera////////////////////////////////////////////////////////////////////
    public static final int CAMERA_RES_HEIGHT = 480;
    public static final int CAMERA_RES_WIDTH  = 640;
    public static final int FPS               = 20;
////////////////////////////other/////////////////////////////////////////////////////////////////////
    public static final double OI_GAIN      = .2;
    public static final double OI_DEAD_BAND = .1;

////////////////////////////exponential drive//////////////////////////////////////////////////////////////////////////
    //default for the constant in the exponential drive equation. overridden by robot preferences
    public static final double EXPONENTIAL_SPEED_CONSTANT  = .5; //must be between 0 and 1
    //default for the constant in the exponential drive equation. overridden by robot preferences
    public static final double EXPONENTIAL_SPEED_DEAD_BAND = .1; //must be between 0 and 1

////////////////////////////PID////////////////////////////////////////////////////////////////////////////////////////

    public static final int    FORWARD_KP             = 0; //TODO value tbd
    public static final int    FORWARD_KI             = 0; //TODO value tbd
    public static final int    FORWARD_KD             = 0; //TODO value tbd
    //set point needs to be negative
    public static final int    FORWARD_SPEED_SETPOINT   = -1; //TODO value tbd

    public static final int    TURNING_KP               = 0; //TODO value tbd
    public static final int    TURNING_KI               = 0; //TODO value tbd
    public static final int    TURNING_KD               = 0; //TODO value tbd
    //set point needs to be negative
    public static final int    TURNING_SPEED_SETPOINT   = -1; //TODO value tbd

    public static final double PID_INTEGRAL_CAP         = 1;
    public static final double PID_INTEGRAL_MINIMUM     = -1;

    public static final double DRIVE_PID_SPEED_CAP      = 0.7;
////////////////////////////drive encoders/////////////////////////////////////////////////////////////////////////

    //ports for the encoders on the driving motors
    public static final int    LEFT_DRIVE_ENC_PORT_ONE  = 1; //TODO value tbd
    public static final int    LEFT_DRIVE_ENC_PORT_TWO  = 2; //TODO value tbd
    public static final int    RIGHT_DRIVE_ENC_PORT_ONE = 3; //TODO value tbd
    public static final int    RIGHT_DRIVE_ENC_PORT_TWO = 4; //TODO value tbd
    //various values for the encoders on the driving motors
    public static final double  DRIVE_ENC_MAX_PERIOD           = 0.5; //TODO value tbd
    public static final double  DRIVE_ENC_MIN_RATE             = 10; //TODO value tbd
    public static final double  DRIVE_ENC_PULSE_PER_REVOLUTION = 20; //TODO value tbd
    public static final int     DRIVE_ENC_AVERAGE_SAMPLES      = 20; //TODO value tbd
    public static final boolean DRIVE_ENC_REVERSE_DIRECTION    = false;

////////////////////////////other//////////////////////////////////////////////////////////////////////////////////////
    public static final double  AUTONOMOUS_SPEED_TOLERANCE     = 1; //TODO value tbd

    // distance in feet from back wall line to auto line. used in conjunction with ROBOT_LENGTH_FEET to determine how
    // far the robot should go to reach the auto line
    public static final double  AUTO_LINE_DISTANCE_FEET        = 10; // value confirmed by manual
    // the robots length in feet. used in conjunction with AUTO_LINE_DISTANCE_FEET to to determine how far the robot
    // should go to reach the auto line
    public static final double ROBOT_LENGTH_FEET            = 38/12; //value confirmed by robot measurment

    public static final String STARTING_POSITION            = "right"; //TODO value tbd
    public static final String FRIENDLY_SCALE_SIDE          = "right"; //TODO value tbd

    public static final double STARTING_X_POSITION          = 0; //TODO value tbd
    public static final double STARTING_Y_POSITION          = 0; //TODO value tbd

    public static final String FRIENDLY_SWITCH_SIDE             = "right";

    public static final double WHEEL_CIRCUMFERENCE_FEET = 8/12;

    public static final int    POWER_DISTRIBUTION_PANEL_ID = 3;

    public static final double GYRO_VOLTS_PER_DEG_PER_SEC = 1; // determines the sensitivity of the gyro

    public static final double OI_XBOX_GAIN               = .05;
    public static double       OI_XBOX_DEAD_BAND          = .1;

    public static final int    WING_SERVO_ID              = 0; //TODO change

    public static final int    DRIVE_MODE_STRAIGHT        = 3;
    public static final int    DRIVE_MODE_HALF            = 2;
    public static final int    DRIVE_MODE_DEFAULT         = 1;

    public static final double FIELD_LENGTH_FEET          = 20; //TODO value tbd
    public static final double AUTO_LINE_TO_SWITCH_FEET   = 9; //TODO value tbd

    public static final int DRIVE_GYRO_ID = 0;

}
