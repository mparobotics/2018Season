package frc.team3926.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class ClimbingSubsystem extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    private WPI_TalonSRX winchMotor1;
    private WPI_TalonSRX winchMotor2;
    private boolean winchOn = false;

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());

        winchMotor1 = new WPI_TalonSRX(RobotMap.WINCH_MOTOR_1);
        winchMotor2 = new WPI_TalonSRX(RobotMap.WINCH_MOTOR_2);
    }

    public void teleopClimb() {

        if(Robot.oi.X.get()) {      //TODO test to see if will run if X is held down and stop when let go

            if(winchOn && Robot.oi.X.get()) {

                winchMotor1.set(0);
                winchMotor2.set(0);
                winchOn = false;

            } else if (winchOn == false){

                winchMotor1.set(RobotMap.WINCH_SPEED);
                winchMotor2.set(RobotMap.WINCH_SPEED);
                winchOn = true;
            }
        }
    }

    public void ClimbDown() {

        if(Robot.oi.Y.get()) {

            winchMotor1.set(-.1);
            winchMotor2.set(-.1);
        }
    }

}

