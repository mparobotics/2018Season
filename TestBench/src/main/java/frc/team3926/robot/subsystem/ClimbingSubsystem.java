package frc.team3926.robot.subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team3926.robot.Robot;
import frc.team3926.robot.RobotMap;

/**
 *
 */
public class ClimbingSubsystem extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    //private WPI_TalonSRX winchMotor1;
    private WPI_TalonSRX winchMotor2;
    private boolean winchOn = false;
    private double startingSpeed = 0;

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());

        //winchMotor1 = new WPI_TalonSRX(RobotMap.WINCH_MOTOR_1);
        winchMotor2 = new WPI_TalonSRX(RobotMap.CLIMBING_WINCH_MOTOR_2);
    }

    public void teleopClimb() {

        //if(Robot.oi.X.get()) {

            /*if(winchOn && Robot.oi.X.get()) {

               // winchMotor1.set(0);
                winchMotor2.set(0);
                winchOn = false;

            } else if (!winchOn){

                //winchMotor1.set(RobotMap.WINCH_SPEED);
                winchMotor2.set(RobotMap.WINCH_SPEED);
                winchOn = true;
            }*/

            //winchMotor2.set(-RobotMap.WINCH_SPEED);
       /* } else {

            winchMotor2.set(0);
        } */

        if(Robot.oi.Y.get()) {

            winchMotor2.set(RobotMap.WINCH_SPEED);
        } else {

            winchMotor2.set(0);
        }
    }

    public void ClimbDown() {

        if(Robot.oi.Y.get()) {

            //winchMotor1.set(-.1);
            winchMotor2.set(-.1);
        }
    }

}

