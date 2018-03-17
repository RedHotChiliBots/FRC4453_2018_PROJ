package org.usfirst.frc.team4453.robot.commands;

import org.usfirst.frc.team4453.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class HookLower extends InstantCommand {

    public HookLower() {
        super();
        requires(Robot.hook);
    }

    // Called once when the command executes
    protected void initialize() {
	Robot.hook.lower();
    }

}
