package frc.team3926.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *  *
 *  */

public class Robot extends IterativeRobot {

	public final static OI oi = new OI();
	//public final static LimitSwitchSubsystem LSSubsystem = new LimitSwitchSubsystem();
	public final static DriveSubsystem driveSubsystem = new DriveSubsystem();
	//public final static CameraSubsystem cameraSubsystem = new CameraSubsystem();
	//public final static SensorSubsystem sensorSubsystem = new SensorSubsystem();

	public static boolean rightPosition;
	public static boolean leftPosition;
	public static boolean centerPosition;

	public static boolean desiredSwitchOnRight; //right = true    left = false

	Thread m_visionThread;
	WPI_TalonSRX encoderMotor;
	//Encoder enc;

	public void robotInit() {

		SmartDashboard.putBoolean("Right Position", rightPosition);
		SmartDashboard.putBoolean("Left Position", leftPosition);
		SmartDashboard.putBoolean("Center Position", centerPosition);

		SmartDashboard.putBoolean("Desired Switch on Right", desiredSwitchOnRight);

		m_visionThread = new Thread(() -> {
			// Get the UsbCamera from CameraServer
			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
			// Set the resolution
			//camera.setResolution(320, 240);

			VideoMode greyscale = new VideoMode(VideoMode.PixelFormat.kMJPEG, RobotMap.CAMERA_RES_WIDTH, RobotMap.CAMERA_RES_HIGHT, RobotMap.FPS);
			camera.setVideoMode(greyscale);

           /* // Get a CvSink. This will capture Mats from the camera
            CvSink cvSink = CameraServer.getInstance().getVideo();
            // Setup a CvSource. This will send images back to the Dashboard
            CvSource outputStream
                    = CameraServer.getInstance().putVideo("Rectangle", RobotMap.CAMERA_RES_WIDTH, RobotMap.CAMERA_RES_HIGHT);

            // Mats are very memory expensive. Lets reuse this Mat.
            Mat mat = new Mat();
            // This cannot be 'true'. The program will never exit if it is. This
            // lets the robot stop this thread when restarting robot code or
            // deploying.
            while (!Thread.interrupted()) {

                // Tell the CvSink to grab a frame from the camera and put it
                // in the source mat.  If there is an error notify the output.
                if (cvSink.grabFrame(mat) == 0) {
                    // Send the output the error.
                    outputStream.notifyError(cvSink.getError());
                    // skip the rest of the current iteration
                    continue;
                }
                // Put a rectangle on the image
                Imgproc.rectangle(mat, new Point(100, 100), new Point(400, 400),
                                  new Scalar(255, 255, 255), 5);
                // Give the output stream a new image to display
                outputStream.putFrame(mat);
            } */
		});
		/*m_visionThread.setDaemon(true);
		m_visionThread.start();

		enc = new Encoder(RobotMap.ENCODER_ID_1, RobotMap.ENCODER_ID_2, false, CounterBase.EncodingType.k4X);

		enc.setMaxPeriod(.1);
		enc.setMinRate(10);
		enc.setDistancePerPulse(6.3); //TODO find better value
		enc.setReverseDirection(true);
		enc.setSamplesToAverage(7); //TODO test to find better value */

	}

    @Override
    public void disabledInit() { }

    @Override
    public void autonomousInit() { }

    @Override
    public void teleopInit() { }

    @Override
    public void testInit() {

		encoderMotor = new WPI_TalonSRX(RobotMap.ENCODER_MOTOR);
		//SmartDashboard.putNumber("Distance", enc.getDistance());
	}

    @Override
    public void disabledPeriodic() { }
    
    @Override
    public void autonomousPeriodic() { }

    @Override
    public void teleopPeriodic() {
	}

    @Override
    public void testPeriodic() {

		encoderMotor.set(.25);
		//SmartDashboard.putNumber("distance", sensorSubsystem.Encoder("Distance"));
		//SmartDashboard.putNumber("raw value", sensorSubsystem.Encoder("Raw Value"));
		//SmartDashboard.putNumber("rate", sensorSubsystem.Encoder("Rate"));
	}
}
