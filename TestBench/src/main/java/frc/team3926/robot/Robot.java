package frc.team3926.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3926.robot.command.auto.SketchyAuto;
import frc.team3926.robot.command.teleop.DeployWingsCommand;
import frc.team3926.robot.command.teleop.StraightenCubeCommandGroup;
import frc.team3926.robot.subsystem.*;

/**
 *  *
 *  */

public class Robot extends IterativeRobot {

	public final static OI oi = new OI();

	public final static frc.team3926.robot.subsystem.DriveSubsystem driveSubsystem = new DriveSubsystem();
	public final static frc.team3926.robot.subsystem.CameraSubsystem cameraSubsystem = new CameraSubsystem();
	public final static Sensors sensorSubsystem = new Sensors();

	public final static frc.team3926.robot.subsystem.IntakeArmSubsystem intakeArmSubsystem = new IntakeArmSubsystem();
	public final static frc.team3926.robot.subsystem.ClimbingSubsystem climbingSubsystem = new ClimbingSubsystem();
	public final static frc.team3926.robot.subsystem.LiftSubsystem liftSubsystem = new LiftSubsystem();

	public final static frc.team3926.robot.command.teleop.StraightenCubeCommandGroup
			StraightenCube = new StraightenCubeCommandGroup();
	public final static DeployWingsCommand fly = new DeployWingsCommand();

	//public final static StraightenCubeOutCommand straightenOut = new StraightenCubeOutCommand();
	//public final static StraightenCubeInCommand straightenIn = new StraightenCubeInCommand();

	public PowerDistributionPanel pdp = new PowerDistributionPanel();

	public boolean week0;

	public double autoWaitTime;
	public double autoDriveTime;


	//public Servo wingServo

	//WPI_TalonSRX encoderMotor;

	public void robotInit() {

		week0 = false; //if true enables FMS for week zero competition

		Robot.cameraSubsystem.startVision(); //starts camera

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
    public void disabledInit() { }

    @Override
    public void autonomousInit() {

		if(RobotMap.QBERT) {

			Scheduler.getInstance().add(new SketchyAuto());
		}

		Robot.sensorSubsystem.setClosedEncoders();
	}

    @Override
    public void teleopInit() {

		Robot.sensorSubsystem.setOpenEncoders();
	}

    public void testInit() {

		/*encoderMotor = new WPI_TalonSRX(RobotMap.ENCODER_MOTOR); //encoder test motor
		wingServo = new Servo (RobotMap.WING_SERVO_ID);*/

	}

    @Override
    public void disabledPeriodic() {
		Scheduler.getInstance().run();


	}
    
    @Override
    public void autonomousPeriodic() {

		Scheduler.getInstance().run();
		SmartDashboard.putNumber("Time: ", DriverStation.getInstance().getMatchTime());
		//Robot.driveSubsystem.sketchyAuto();
		/*double timeElapsed = 15 - DriverStation.getInstance().getMatchTime(); // The DriverStation gives an approximate time until the end of the period

		if (timeElapsed >= autoWaitTime) {
			if (timeElapsed <= autoWaitTime + autoDriveTime) {

				driveSubsystem.setSpeed(-.5, -.5);
			}
		}*/



	}

    @Override
    public void teleopPeriodic() {

		Scheduler.getInstance().run();
		SmartDashboard.putNumber("Time: ", DriverStation.getInstance().getMatchTime());
		Robot.sensorSubsystem.printAccelerometerValues();
		Robot.sensorSubsystem.printGyroAngle();

		Robot.sensorSubsystem.getRobotPosistion();
		SmartDashboard.putNumber("Posistion X: ", Robot.sensorSubsystem.X);
		SmartDashboard.putNumber("Posistion Y: ", Robot.sensorSubsystem.Y);
		SmartDashboard.putNumber("Posistion Angle: ", Robot.sensorSubsystem.gyroAngle);

		//encoderMotor.set(.1);
		//SmartDashboard.putNumber("distance", sensorSubsystem.Encoder("Distance"));
		//SmartDashboard.putBoolean("Limit Switch", sensorSubsystem.LimitSwitch());


	}

    @Override
    public void testPeriodic() {
		Robot.sensorSubsystem.printAccelerometerValues();
		Robot.sensorSubsystem.printGyroAngle();
		Robot.sensorSubsystem.printDistanceTraveled();

		Scheduler.getInstance().run();
		/*encoderMotor.set(Robot.oi.xboxController.getRawAxis(Robot.oi.xboxLeftAxis));
		wingServo.set(Robot.oi.xboxController.getRawAxis(Robot.oi.xboxRightAxis));*/

	}
}
