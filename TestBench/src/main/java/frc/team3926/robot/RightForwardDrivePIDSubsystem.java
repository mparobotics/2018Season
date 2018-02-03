package frc.team3926.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**
 * Created by blash20 on 2/2/18.
 *
 */
public class RightForwardDrivePIDSubsystem extends PIDSubsystem {

    Encoder rightDriveEnc = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
    WPI_TalonSRX motor;

    public RightForwardDrivePIDSubsystem(){

        super("RightForwardDrivePIDSubsystem",0,0,0);

        setAbsoluteTolerance(0);
        getPIDController().setContinuous(false);
        motor = new WPI_TalonSRX(0);

    }

    protected void initDefaultCommand() {

    }

    protected double returnPIDInput() {

        return rightDriveEnc.get();
    }

    protected void usePIDOutput(double output) {

        motor.set(output);

    }

}
