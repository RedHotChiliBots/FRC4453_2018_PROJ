/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4453.robot;

import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

    private Joystick drive = new Joystick(0);

    public double getSpdAxis() {
	// TODO
	return drive.getY();
    }

    public double getTurnAxis() {
	// TODO
	return drive.getX();
    }

    public boolean getQuickTurn() {
	return drive.getRawButtonPressed(4) || drive.getRawButtonPressed(5);
    }
}
