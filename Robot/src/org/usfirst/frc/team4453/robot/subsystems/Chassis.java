package org.usfirst.frc.team4453.robot.subsystems;

import org.usfirst.frc.team4453.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 *
 */
public class Chassis extends Subsystem {
    private final WPI_TalonSRX leftFront  = new WPI_TalonSRX(RobotMap.FRONT_LEFT_MOTOR);
    private final WPI_TalonSRX rightFront = new WPI_TalonSRX(RobotMap.FRONT_RIGHT_MOTOR);
    private final WPI_TalonSRX leftMid	  = new WPI_TalonSRX(RobotMap.MID_LEFT_MOTOR);
    private final WPI_TalonSRX rightMid	  = new WPI_TalonSRX(RobotMap.MID_RIGHT_MOTOR);
    private final WPI_TalonSRX leftBack	  = new WPI_TalonSRX(RobotMap.REAR_LEFT_MOTOR);
    private final WPI_TalonSRX rightBack  = new WPI_TalonSRX(RobotMap.REAR_RIGHT_MOTOR);

    private final DoubleSolenoid shifter = new DoubleSolenoid(RobotMap.SHIFTER_FWD, RobotMap.SHIFTER_REV);;

    private final DifferentialDrive drive = new DifferentialDrive(leftFront, rightFront);

    public Chassis() {
	leftMid.follow(leftFront);
	leftBack.follow(leftFront);

	rightMid.follow(rightFront);
	rightBack.follow(rightFront);

	shift(false);
    }

    public void drive(double lspeed, double rspeed) {
	drive.tankDrive(lspeed, rspeed);
    }

    @Override
    public void initDefaultCommand() {
	// Set the default command for a subsystem here.
	// setDefaultCommand(new MySpecialCommand());
    }

    public void shift(boolean highgear) {
	shifter.set(highgear ? RobotMap.SHIFTER_HIGH_GEAR : RobotMap.SHIFTER_LOW_GEAR);
    }

    public void stop() {
	drive.stopMotor();
    }
}
