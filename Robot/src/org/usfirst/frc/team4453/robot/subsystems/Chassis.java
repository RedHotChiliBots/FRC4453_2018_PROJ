package org.usfirst.frc.team4453.robot.subsystems;

import org.usfirst.frc.team4453.robot.RobotMap;
import org.usfirst.frc.team4453.robot.commands.TeleopDrive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 *
 */
public class Chassis extends Subsystem {
    private final WPI_TalonSRX	    leftFront			 = new WPI_TalonSRX(RobotMap.CHASSIS_FRONT_LEFT_MOTOR);
    private final WPI_TalonSRX	    rightFront			 = new WPI_TalonSRX(RobotMap.CHASSIS_FRONT_RIGHT_MOTOR);
    private final WPI_TalonSRX	    leftMid			 = new WPI_TalonSRX(RobotMap.CHASSIS_MID_LEFT_MOTOR);
    private final WPI_TalonSRX	    rightMid			 = new WPI_TalonSRX(RobotMap.CHASSIS_MID_RIGHT_MOTOR);
    private final WPI_TalonSRX	    leftBack			 = new WPI_TalonSRX(RobotMap.CHASSIS_REAR_LEFT_MOTOR);
    private final WPI_TalonSRX	    rightBack			 = new WPI_TalonSRX(RobotMap.CHASSIS_REAR_RIGHT_MOTOR);

    private final DoubleSolenoid    shifter			 = new DoubleSolenoid(RobotMap.SHIFTER_FWD_SOLENOID,
	    RobotMap.SHIFTER_REV_SOLENOID);

    private final DifferentialDrive drive			 = new DifferentialDrive(leftFront, rightFront);

    private final double	    PRESSURE_SENSOR_INPUTVOLTAGE = 5.0;
    private AnalogInput		    hiPressureSensor		 = new AnalogInput(RobotMap.HI_PRESSURE_SENSOR);
    private AnalogInput		    loPressureSensor		 = new AnalogInput(RobotMap.LO_PRESSURE_SENSOR);

    public Chassis() {
	leftFront.setInverted(false);
	leftMid.follow(leftFront);
	leftBack.follow(leftFront);

	rightFront.setInverted(true);
	rightMid.follow(rightFront);
	rightBack.follow(rightFront);

	shift(false);
    }

    public void drive(double lspeed, double rspeed) {
	drive.tankDrive(lspeed, rspeed);
    }

    public void curveDrive(double spdCmd, double rotCmd, boolean quickTurn) {
	drive.curvatureDrive(spdCmd, rotCmd, false);
    }

    @Override
    public void initDefaultCommand() {
	setDefaultCommand(new TeleopDrive());
    }

    public void shift(boolean highgear) {
	shifter.set(highgear ? RobotMap.SHIFTER_HIGH_GEAR : RobotMap.SHIFTER_LOW_GEAR);
    }

    public void stop() {
	drive.stopMotor();
    }
}
