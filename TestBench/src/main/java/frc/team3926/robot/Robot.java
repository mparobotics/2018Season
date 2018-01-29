package frc.team3926.robot;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 *  * This is a demo program showing the use of OpenCV to do vision processing. The
 *  * image is acquired from the USB camera, then a rectangle is put on the image
 *  * and sent to the dashboard. OpenCV has many methods for different types of
 *  * processing.
 *  */


public class Robot extends IterativeRobot {

	public final static OI OI = new OI();
	public final static LimitSwitchSubsystem LSSubsystem = new LimitSwitchSubsystem();
	public final static DriveSubsystem driveSubsystem = new DriveSubsystem();
	public final static AutonomousSpeedPIDSubsystem autonomousSpeedPIDSubsystem = new AutonomousSpeedPIDSubsystem();

	Thread m_visionThread;

	public static boolean rightPosition;
	public static boolean leftPosition;
	public static boolean centerPosition;

	public static boolean desiredSwitchOnRight; //right = true    left = false

	public void robotInit() {

		SmartDashboard.putBoolean("Right Position", rightPosition);
		SmartDashboard.putBoolean("Left Position", leftPosition);
		SmartDashboard.putBoolean("Center Position", centerPosition);

		SmartDashboard.putBoolean("Desired Switch on Right", desiredSwitchOnRight);

		m_visionThread = new Thread(() -> {
			// Get the UsbCamera from CameraServer
			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
			// Set the resolution
			camera.setResolution(320, 240);

			// Get a CvSink. This will capture Mats from the camera
			CvSink cvSink = CameraServer.getInstance().getVideo();
			// Setup a CvSource. This will send images back to the Dashboard
			CvSource outputStream
					= CameraServer.getInstance().putVideo("Rectangle", 640, 480);

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
			}
		});
		m_visionThread.setDaemon(true);
		m_visionThread.start();

	}

    @Override
    public void disabledInit() { }

    @Override
    public void autonomousInit() { }

    @Override
    public void teleopInit() { }

    @Override
    public void testInit() { }

    @Override
    public void disabledPeriodic() { }
    
    @Override
    public void autonomousPeriodic() { }

    @Override
    public void teleopPeriodic() {
	}

    @Override
    public void testPeriodic() { }
}
