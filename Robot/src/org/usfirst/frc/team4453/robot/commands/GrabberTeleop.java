package org.usfirst.frc.team4453.robot.commands;

import org.usfirst.frc.team4453.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class GrabberTeleop extends Command {

    private double angle  = 0;
    private double speed  = 0;
    private double diff	  = 0;
    private double lMotor = 0;
    private double rMotor = 0;

    public GrabberTeleop() {
	requires(Robot.grabber);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
	angle += Robot.oi.getTiltAxis() * .25;
	SmartDashboard.putNumber("Tilt Angle", angle);
	Robot.grabber.tilt(angle);

	speed = Robot.oi.getGrabSpeedAxis();
	diff = Robot.oi.getGrabDiffAxis();
	lMotor = speed * -diff;
	rMotor = speed * diff;

	Robot.grabber.diff(lMotor, rMotor);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
	return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}