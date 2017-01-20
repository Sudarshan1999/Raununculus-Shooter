package org.usfirst.frc.team1165.robot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Shooter extends Subsystem
{

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	public CANTalon shooterWheel;

	public Shooter()
	{
		shooterWheel = new CANTalon(4);
		shooterWheel.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		shooterWheel.configEncoderCodesPerRev(20);

		shooterWheel.setP(0.75);
		shooterWheel.setI(0.01);
		shooterWheel.setD(0.75);
		shooterWheel.setCloseLoopRampRate(0.01);
	}
	
	public void enableShooter()
	{
		driveShooterAtRPM(SmartDashboard.getDouble("Shooter Wheel RPM"));
	}

	@Override
	protected void initDefaultCommand()
	{
		// TODO Auto-generated method stub

	}

	public void driveShooterAtRPM(double rpm)
	{
		shooterWheel.setControlMode(TalonControlMode.Speed.getValue());
		shooterWheel.set(rpm);
	}

	public void report()
	{
		SmartDashboard.putNumber("Display Shooter Wheel RPM", shooterWheel.getSpeed());
	}
}
