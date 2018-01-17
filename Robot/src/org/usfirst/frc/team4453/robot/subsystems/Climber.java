package org.usfirst.frc.team4453.robot.subsystems;

import org.usfirst.frc.team4453.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Climber extends Subsystem {
    public final int	COUNTS_PER_REV	= 100;				  // TODO
    public final double	INCHES_PER_REV	= 3.3;				  // TODO
    public final double	COUNTS_PER_INCH	= COUNTS_PER_REV / INCHES_PER_REV;

    private final WPI_TalonSRX climberLeft  = new WPI_TalonSRX(RobotMap.CLIMBER_MOTOR_RIGHT);
    private final WPI_TalonSRX climberRight = new WPI_TalonSRX(RobotMap.CLIMBER_MOTOR_LEFT);

    @Override
    public void initDefaultCommand() {
	// Set the default command for a subsystem here.
	// setDefaultCommand(new MySpecialCommand());
    }

    public void setLeftPower(double speed) {
	climberLeft.set(speed);
    }

    public void setRightPower(double speed) {
	climberRight.set(speed);
    }

    public void stop() {
	climberLeft.set(0);
	climberRight.set(0);
    }
}
