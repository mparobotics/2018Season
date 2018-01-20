package frc.team3926.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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
	Thread m_visionThread;
	DigitalInput limitSwitch;

	Joystick rightStick = new Joystick(RobotMap.RIGHT_JOYSTICK);
	Joystick leftStick = new Joystick(RobotMap.LEFT_JOYSTICK);

	//CANTalon FR = new CANTalon(RobotMap.FRONT_RIGHT);

	private DifferentialDrive m_myRobot;

	public void robotInit() {


		m_myRobot = new DifferentialDrive(new WPI_TalonSRX(RobotMap.FRONT_LEFT), new WPI_TalonSRX(RobotMap.FRONT_RIGHT));

		DigitalInput limitSwitch = new DigitalInput(RobotMap.LIMIT_SWITCH);
		m_visionThread = new Thread(() -> {
			// Get the UsbCamera from CameraServer
			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
			// Set the resolution
			camera.setResolution(640, 480);

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

				SmartDashboard.putBoolean("Limit Switch", limitSwitch.get());
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

		m_myRobot.tankDrive(-leftStick.getY(), -rightStick.getY());
	}

    @Override
    public void testPeriodic() { }
}
