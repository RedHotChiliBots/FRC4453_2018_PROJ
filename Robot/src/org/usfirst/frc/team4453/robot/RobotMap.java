/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4453.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    public static final int FRONT_RIGHT_MOTOR	= 0; // TODO
    public static final int FRONT_LEFT_MOTOR	= 1; // TODO
    public static final int MID_RIGHT_MOTOR	= 2; // TODO
    public static final int MID_LEFT_MOTOR	= 3; // TODO
    public static final int REAR_RIGHT_MOTOR	= 4; // TODO
    public static final int REAR_LEFT_MOTOR	= 5; // TODO
    public static final int CLIMBER_MOTOR_RIGHT	= 6; // TODO
    public static final int CLIMBER_MOTOR_LEFT	= 6; // TODO

    public static final int SHIFTER_FWD	= 0; // TODO
    public static final int SHIFTER_REV	= 1; // TODO

    public static final DoubleSolenoid.Value SHIFTER_HIGH_GEAR = DoubleSolenoid.Value.kForward;
    public static final DoubleSolenoid.Value SHIFTER_LOW_GEAR  = DoubleSolenoid.Value.kReverse;
}
