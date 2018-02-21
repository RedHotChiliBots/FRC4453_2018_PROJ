/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4453.robot;

import org.usfirst.frc.team4453.robot.commands.*;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

    private Joystick	   drive    = new Joystick(RobotMap.FIRST_CONTROLLER);
    private XboxController operator = new XboxController(RobotMap.SECOND_CONTROLLER);

    private JoystickButton quickTurn1 = new JoystickButton(drive, RobotMap.BUTTON_5);
    private JoystickButton quickTurn2 = new JoystickButton(drive, RobotMap.BUTTON_4);
    private JoystickButton shiftHigh  = new JoystickButton(drive, RobotMap.BUTTON_3);
    private JoystickButton shiftLow   = new JoystickButton(drive, RobotMap.BUTTON_2);

    private JoystickButton grabberGrab	  = new JoystickButton(operator, RobotMap.A_BUTTON);
    private JoystickButton grabberRelease = new JoystickButton(operator, RobotMap.B_BUTTON);
    private JoystickButton grabberThrow	  = new JoystickButton(operator, RobotMap.X_BUTTON);

    private JoystickButton wingsLift = new JoystickButton(operator, RobotMap.LEFT_BUMPER);
    private JoystickButton wingsDrop = new JoystickButton(operator, RobotMap.RIGHT_BUMPER);

    public OI() {
	shiftHigh.whenPressed(new ChassisShiftHigh());
	shiftLow.whenPressed(new ChassisShiftLow());

	grabberGrab.whileHeld(new GrabberGrab());
	grabberGrab.whenReleased(new GrabberHold());
	grabberRelease.whileHeld(new GrabberRelease());
	grabberThrow.whileHeld(new GrabberThrow());
	grabberThrow.whenReleased(new GrabberHold());

	wingsLift.whenPressed(new WingsLift());
	wingsDrop.whenPressed(new WingsDrop());
    }

    public double getSpdAxis() {
	return drive.getY();
    }

    public double getTurnAxis() {
	return -drive.getX();
    }

    public double getTiltAxis() {
	return operator.getY(Hand.kRight);
    }

    public double getGrabDiffAxis() {
	return operator.getX(Hand.kLeft);
    }

    public double getGrabSpeedAxis() {
	return operator.getY(Hand.kLeft);
    }

    public boolean getQuickTurn() {
	return quickTurn1.get() || quickTurn2.get();
    }

    public boolean getWingsLift() {
	return operator.getBumper(Hand.kLeft);
    }

    public boolean getWingsDrop() {
	return operator.getBumper(Hand.kRight);
    }
}
