package frc.team3926.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 *  *
 *  */

public class Robot extends IterativeRobot {

	public final static OI oi = new OI();

	public final static DriveSubsystem driveSubsystem = new DriveSubsystem();
	//public final static CameraSubsystem cameraSubsystem = new CameraSubsystem();
	public final static SensorSubsystem sensorSubsystem = new SensorSubsystem();

	public final static IntakeArmSubsystem intakeArmSubsystem = new IntakeArmSubsystem();
	public final static ClimbingSubsystem climbingSubsystem = new ClimbingSubsystem();
	public final static LiftSubsystem liftSubsystem = new LiftSubsystem();

	static Preferences smartDashPrefs;

	public PowerDistributionPanel pdp = new PowerDistributionPanel(RobotMap.POWER_DISTRIBUTION_PANEL_ID);

	public boolean week0;

	public double autoWaitTime;
	public double autoDriveTime;

	public Servo wingServo;


	//WPI_TalonSRX encoderMotor;

	public void robotInit() {

		week0 = false; //if true enables FMS for week zero competition

		//Robot.cameraSubsystem.initDefaultCommand(); //starts camera

		if (week0) {

			NetworkTableInstance offSeasonNetworkTable =
					NetworkTableInstance.create();
			offSeasonNetworkTable.startClient("10.0.100.5");
			String gameData = offSeasonNetworkTable
					.getTable("OffseasonFMSInfo")
					.getEntry("GameData")
					.getString("defaultValue");

		}

		smartDashPrefs = Preferences.getInstance();

		//SmartDashboard.putData("PowerDistributionPanel", pdp);

	}

    @Override
	public void disabledInit() {

	}

    @Override
    public void autonomousInit() {


		new AutonomousCommandGroup();
		autoWaitTime = 0;
		autoDriveTime = 2;
	}

    @Override
    public void teleopInit() {

		//encoderMotor = new WPI_TalonSRX(RobotMap.ENCODER_MOTOR);

	}

    public void testInit() {

		//encoderMotor = new WPI_TalonSRX(RobotMap.ENCODER_MOTOR); //encoder test motor
		wingServo = new Servo (RobotMap.WING_SERVO_ID);
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

		Scheduler.getInstance().run();
        driveSubsystem.teleopDrive();


		//encoderMotor.set(.1);
		//SmartDashboard.putNumber("distance", sensorSubsystem.Encoder("Distance"));
		//SmartDashboard.putBoolean("Limit Switch", sensorSubsystem.LimitSwitch());

	}

    @Override
    public void testPeriodic() {

	    Scheduler.getInstance().run();
		driveSubsystem.teleopDrive();

		//encoderMotor.set(.1);
		SmartDashboard.putNumber("distance", sensorSubsystem.Encoder("Distance"));
		SmartDashboard.putBoolean("Limit Switch", sensorSubsystem.LimitSwitch());

	}
}
