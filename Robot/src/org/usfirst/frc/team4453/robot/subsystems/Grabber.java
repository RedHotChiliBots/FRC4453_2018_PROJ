package org.usfirst.frc.team4453.robot.subsystems;

import org.usfirst.frc.team4453.robot.RobotMap;
import org.usfirst.frc.team4453.robot.commands.GrabberTeleop;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Grabber extends Subsystem {
    /**
     * Number of encoder ticks per one degree of tilt.
     */
    private static final int COUNTS_PER_REV_MOTOR   = 12;
    private static final int GEAR_RATIO		    = 100;
    private static final int COUNTS_PER_REV_GEARBOX = COUNTS_PER_REV_MOTOR * GEAR_RATIO;
    private static final int TICKS_PER_DEGREE	    = COUNTS_PER_REV_GEARBOX / 360;

    private WPI_TalonSRX     left		    = new WPI_TalonSRX(RobotMap.GRABBER_LEFT_MOTOR);
    private WPI_TalonSRX     right		    = new WPI_TalonSRX(RobotMap.GRABBER_RIGHT_MOTOR);
    private WPI_TalonSRX     tilt		    = new WPI_TalonSRX(RobotMap.GRABBER_TILT_MOTOR);
    private DoubleSolenoid   grip		    = new DoubleSolenoid(RobotMap.GRABBER_GRIP_SOLENOID,
	    RobotMap.GRABBER_RELEASE_SOLENOID);

    /**
     * Command to initialize the Grabber.
     * Sets subsystem to default state and resets encoder zeros.
     */

    private class Init extends CommandGroup {
	public Init() {
	    addSequential(new ResetCommand());
	}
    }

    private class ResetCommand extends Command {
	public ResetCommand() {
	    setInterruptible(false);
	    requires(Grabber.this);
	}

	@Override
	protected void initialize() {
	    left.setNeutralMode(NeutralMode.Brake);
	    right.setNeutralMode(NeutralMode.Brake);
	    left.neutralOutput();
	    right.neutralOutput();

	    grip.set(Value.kReverse);

	    tilt.setNeutralMode(NeutralMode.Brake);
	    tilt.set(ControlMode.PercentOutput, -.2); // TODO: Correct
						      // direction?
	}

	@Override
	protected boolean isFinished() {
	    return tilt.getSensorCollection().isRevLimitSwitchClosed(); // TODO:
									// Correct
									// direction?
	}

	@Override
	protected void end() {
	    tilt.neutralOutput();
	    tilt.getSensorCollection().setQuadraturePosition(0, 100);
	    tilt.set(ControlMode.Position, 0);

	}
    }

    private class NoOp extends Command {
	public NoOp() {
	    requires(Grabber.this);
	}

	@Override
	protected boolean isFinished() {
	    return false;
	}
    }

    @Override
    public void initDefaultCommand() {
	setDefaultCommand(new GrabberTeleop());
    }

    public void init() {
	new Init().start();
    }

    public void grab() {
	left.set(ControlMode.PercentOutput, .5);
	right.set(ControlMode.PercentOutput, -.5);
	grip.set(Value.kForward);
    }

    public void release() {
	left.neutralOutput();
	right.neutralOutput();
	grip.set(Value.kReverse);
    }

    public void toss() {
	left.set(ControlMode.PercentOutput, -1);
	right.set(ControlMode.PercentOutput, 1);
    }

    public void hold() {
	grip.set(Value.kForward);
	left.neutralOutput();
	right.neutralOutput();
    }

    public void tilt(double angle) {
	tilt.set(ControlMode.Position, angle * TICKS_PER_DEGREE);
    }

    public boolean isLimitHit() {
	return tilt.getSensorCollection().isRevLimitSwitchClosed();
    }
}
