package org.usfirst.frc.team4453.robot.subsystems;

import org.usfirst.frc.team4453.robot.RobotMap;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Lifter extends Subsystem {
    private final Solenoid left	 = new Solenoid(RobotMap.SOLENOID_LIFTER_LEFT);
    private final Solenoid right = new Solenoid(RobotMap.SOLENOID_LIFTER_RIGHT);

    public void drop() {
	left.set(true);
	right.set(true);
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    @Override
    public void initDefaultCommand() {
	// Set the default command for a subsystem here.
	// setDefaultCommand(new MySpecialCommand());
    }
}
