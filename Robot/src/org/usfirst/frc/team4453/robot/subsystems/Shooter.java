package org.usfirst.frc.team4453.robot.subsystems;

import org.usfirst.frc.team4453.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 *
 */
public class Shooter extends Subsystem {
    private WPI_TalonSRX   motor       = new WPI_TalonSRX(RobotMap.SHOOTER_WIND_MOTOR);
    private DoubleSolenoid latch       = new DoubleSolenoid(RobotMap.SHOOTER_LATCH_SOLENOID,
	    RobotMap.SHOOTER_RELEASE_SOLENOID);
    private DigitalInput   limitswitch = new DigitalInput(RobotMap.SHOOTER_LIMIT_SWITCH);
    private Relay	   clutch      = new Relay(RobotMap.SHOOTER_CLUTCH_RELAY);

    public class PrepareCommand extends CommandGroup {
	public PrepareCommand() {
	    addSequential(new PrepareCommand1());
	    addSequential(new PrepareCommand2());
	}
    }

    private class PrepareCommand1 extends Command {

	public PrepareCommand1() {
	    requires(Shooter.this);
	}

	@Override
	protected void initialize() {
	    clutch.set(Relay.Value.kOn);
	    motor.set(1);
	}

	@Override
	protected boolean isFinished() {
	    // TODO Auto-generated method stub
	    return limitswitch.get();
	}

	@Override
	protected void end() {
	    latch.set(DoubleSolenoid.Value.kForward);
	    motor.neutralOutput();
	}

    }

    private class PrepareCommand2 extends TimedCommand {

	public PrepareCommand2() {
	    super(0.5);
	    requires(Shooter.this);
	}

	@Override
	protected void initialize() {

	}

	@Override
	protected void end() {
	    clutch.set(Relay.Value.kOff);
	}

    }

    public void fire() {
	latch.set(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public void initDefaultCommand() {

    }
}
