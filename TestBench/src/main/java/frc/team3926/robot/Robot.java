package frc.team3926.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *  *
 *  */

public class Robot extends IterativeRobot {

	public final static OI oi = new OI();
	//public final static LimitSwitchSubsystem LSSubsystem = new LimitSwitchSubsystem();
	public final static DriveSubsystem driveSubsystem = new DriveSubsystem();
	public final static CameraSubsystem cameraSubsystem = new CameraSubsystem();
	public final static SensorSubsystem sensorSubsystem = new SensorSubsystem();

	public final static EncoderCommand encoderCommand = new EncoderCommand();

	//public static boolean rightPosition;
	//public static boolean leftPosition;
	//public static boolean centerPosition;
	//public static boolean desiredSwitchOnRight; //right = true    left = false



	double speed;

	WPI_TalonSRX encoderMotor;

	public void robotInit() {

		//SmartDashboard.putBoolean("Right Position", rightPosition);
		//SmartDashboard.putBoolean("Left Position", leftPosition);
		//SmartDashboard.putBoolean("Center Position", centerPosition);

		//SmartDashboard.putBoolean("Desired Switch on Right", desiredSwitchOnRight);

		Robot.cameraSubsystem.initDefaultCommand();

	}

    @Override
    public void disabledInit() { }

    @Override
    public void autonomousInit() { }

    @Override
    public void teleopInit() {

		encoderMotor = new WPI_TalonSRX(RobotMap.ENCODER_MOTOR);
	}

    public void testInit() {

	}

    @Override
    public void disabledPeriodic() {
		Scheduler.getInstance().run();


	}
    
    @Override
    public void autonomousPeriodic() {

		Scheduler.getInstance().run();

	}

    @Override
    public void teleopPeriodic() {

		driveSubsystem.teleopDrive();
		//encoderMotor.set(.1);
		SmartDashboard.putNumber("distance", sensorSubsystem.Encoder("Distance"));
		SmartDashboard.putBoolean("Limit Switch", sensorSubsystem.LimitSwitch());
	}

    @Override
    public void testPeriodic() {


	}
}
