package org.usfirst.frc.team4453.robot.commands;

import org.usfirst.frc.team4453.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TeleopDrive extends Command {

    public TeleopDrive() {
	requires(Robot.chassis);
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
	Robot.chassis.stop();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
	Robot.chassis.drive(Robot.oi.getLAxis(), Robot.oi.getRAxis());
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
	Robot.chassis.stop();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
	return false;
    }
}
