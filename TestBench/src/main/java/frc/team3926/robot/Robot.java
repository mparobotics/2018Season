package frc.team3926.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Preferences;
/**
 *  *
 *  */

public class Robot extends IterativeRobot {

	public final static OI oi = new OI();
	//public final static LimitSwitchSubsystem LSSubsystem = new LimitSwitchSubsystem();
	public final static DriveSubsystem driveSubsystem = new DriveSubsystem();
	public final static CameraSubsystem cameraSubsystem = new CameraSubsystem();
	public final static SensorSubsystem sensorSubsystem = new SensorSubsystem();

	public final static IntakeArmSubsystem intakeArmSubsystem = new IntakeArmSubsystem();
	public final static ClimbingSubsystem climbingSubsystem = new ClimbingSubsystem();
	public final static LiftSubsystem liftSubsystem = new LiftSubsystem();

	static Preferences smartDashPrefs;

	public PowerDistributionPanel pdp = new PowerDistributionPanel(RobotMap.POWER_DISTRIBUTION_PANEL_ID);

	public boolean week0;

	WPI_TalonSRX encoderMotor;

	public void robotInit() {

		week0 = false;

		Robot.cameraSubsystem.initDefaultCommand(); //starts camera

		if (week0) {

			NetworkTableInstance offSeasonNetworkTable =
					NetworkTableInstance.create();
			offSeasonNetworkTable.startClient("10.0.100.5");
			String gameData = offSeasonNetworkTable
					.getTable("OffseasonFMSInfo")
					.getEntry("GameData")
					.getString("defaultValue");

		}

		SmartDashboard.putData("PowerDistributionPanel", pdp);

	}

    @Override
	public void disabledInit() {

	}

    @Override
    public void autonomousInit() { }

    @Override
    public void teleopInit() {

		//encoderMotor = new WPI_TalonSRX(RobotMap.ENCODER_MOTOR);
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
		new AutonomousCommandGroup();

	}

    @Override
    public void teleopPeriodic() {

		Scheduler.getInstance().run();
        driveSubsystem.teleopDrive();

		encoderMotor.set(.1);
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
