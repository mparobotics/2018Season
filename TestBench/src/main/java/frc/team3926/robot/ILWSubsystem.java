package frc.team3926.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * intake arm, lift, winch
 */
public class ILWSubsystem extends Subsystem {

    private WPI_TalonSRX intakeMotor1;
    private WPI_TalonSRX intakeMotor2;
    private WPI_TalonSRX winchMotor1;
    private WPI_TalonSRX winchMotor2;
    private WPI_TalonSRX liftMotor;
    private double ESC;
    private double rawAxis;

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());

        intakeMotor1 = new WPI_TalonSRX(RobotMap.INTAKE_MOTOR_1);
        intakeMotor2 = new WPI_TalonSRX(RobotMap.INTAKE_MOTOR_2);
        winchMotor1 = new WPI_TalonSRX(RobotMap.WINCH_MOTOR_1);
        winchMotor2 = new WPI_TalonSRX(RobotMap.WINCH_MOTOR_2);
        liftMotor = new WPI_TalonSRX(RobotMap.LIFT_MOTOR);

        ESC = RobotMap.EXPONENTIAL_SPEED_CONSTANT;
    }

    public void IntakeArm() {

        if(Robot.oi.RB.get()) {

            intakeMotor1.set(RobotMap.INTAKE_SPEED);
            intakeMotor2.set(RobotMap.INTAKE_SPEED);
        }
        else if (Robot.oi.LB.get()) {

            intakeMotor1.set(-RobotMap.INTAKE_SPEED);
            intakeMotor2.set(-RobotMap.INTAKE_SPEED);
        }

    }

    public void Winch(){

        if(Robot.oi.X.get()) {      //TODO test to see if will run if X is held down and stop when let go

            winchMotor1.set(RobotMap.WINCH_SPEED);
            winchMotor2.set(RobotMap.WINCH_SPEED);
        }
    }

    public void Lift(){     //TODO add limit switches?

        rawAxis = Robot.oi.xboxController.getRawAxis(Robot.oi.xboxLeftAxis);
        liftMotor.set(ESC * (Math.pow(rawAxis, 3)) + (1 - ESC) * rawAxis);
    }
}

