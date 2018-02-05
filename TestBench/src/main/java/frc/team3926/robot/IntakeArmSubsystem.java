package frc.team3926.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class IntakeArmSubsystem extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    private WPI_TalonSRX intakeMotor1;
    private WPI_TalonSRX intakeMotor2;

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());

        intakeMotor1 = new WPI_TalonSRX(RobotMap.INTAKE_MOTOR_1);
        intakeMotor2 = new WPI_TalonSRX(RobotMap.INTAKE_MOTOR_2);
    }

    public void teleopIntake() {


        if(Robot.oi.RB.get()) {

            intakeMotor1.set(RobotMap.INTAKE_SPEED);
            intakeMotor2.set(RobotMap.INTAKE_SPEED);
        }
        else if (Robot.oi.LB.get()) {

            intakeMotor1.set(-RobotMap.INTAKE_SPEED);
            intakeMotor2.set(-RobotMap.INTAKE_SPEED);
        }
    }

    public void autoReleaseCube() {

        intakeMotor1.set(-RobotMap.INTAKE_SPEED);
        intakeMotor2.set(-RobotMap.INTAKE_SPEED);

        Timer.delay(2);

        intakeMotor1.set(0);
        intakeMotor2.set(0);
    }
}

