package org.usfirst.frc.team4453.robot.subsystems;

import org.usfirst.frc.team4453.robot.RobotMap;
import org.usfirst.frc.team4453.robot.commands.HookStop;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Hook extends Subsystem {
    public final int	COUNTS_PER_REV	= 100;				  // TODO
    public final double	INCHES_PER_REV	= 3.3;				  // TODO
    public final double	COUNTS_PER_INCH	= COUNTS_PER_REV / INCHES_PER_REV;

    private final WPI_TalonSRX hookLift = new WPI_TalonSRX(RobotMap.CLIMBER_HOOK_MOTOR);

    public double getDistanceLifted() {
	return Math.min(hookLift.getSensorCollection().getQuadraturePosition(),
		hookLift.getSensorCollection().getQuadraturePosition()) / COUNTS_PER_INCH;
    }

    @Override
    public void initDefaultCommand() {
	setDefaultCommand(new HookStop());
    }

    public void raise() {
	hookLift.set(0.5);
    }

    public void stop() {
	hookLift.neutralOutput();
    }

    public void lower() {
	hookLift.set(-0.5);
    }

    public void resetEncoder() {
	hookLift.getSensorCollection().setQuadraturePosition(0, 100);
    }
}
