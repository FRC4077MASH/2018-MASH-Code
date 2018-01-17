package org.usfirst.frc.team4077.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends IterativeRobot {
	// Autonomous navigation array
	// 1st Column: desired type of movement: -1: end of auto, 0: move on y-axis (forwards/backwards), 1: move on x-axis (strafe for mecanum), 2: rotate on z-axis (yaw), 3: pause for seconds
	// 2nd Column: Tp, the speed at which to move
	// 3rd Column: goal value, either angle, distance in inches, or time in seconds
	private double[][] movementArray = new double[][] {
		  //{ -, --.--, ----.- },
			{ 0,  0.3,   24.0 },
			{ 3,  0.00,    3.0 },
			{ 0, -0.30,  -24.0 },
			{ 3,  0.00,    3.0 },
			{ 1,  0.30,   24.0 },
			{ 3,  0.00,    3.0 },
			{ 1, -0.30,  -24.0 },
			{ 3,  0.00,    3.0 },
			{ 2,  0.30,   90.0 },
			{ 3,  0.00,    3.0 },
			{ 2, -0.30,    0.0 },
			{-1,  0.00,    0.0 }
	};

	private double speedFactor = 0.25;
	private boolean slowSpeed;
	private boolean lastAButton;
	
	private WPI_TalonSRX frontLeft = new WPI_TalonSRX(1);
	private WPI_TalonSRX rearLeft = new WPI_TalonSRX(4);
	private WPI_TalonSRX frontRight = new WPI_TalonSRX(2);
	private WPI_TalonSRX rearRight = new WPI_TalonSRX(3);
	
	private Joystick stick = new Joystick(0);
	
	private ADXRS450_Gyro gyro = new ADXRS450_Gyro();

	private NavigationPID navigator = new NavigationPID(movementArray, frontLeft, rearLeft, frontRight, rearRight, NavigationPID.DRIVE_TYPE.MECANUM, gyro, 1000, 8.0, 0.005, 0.0001, 0.0);

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println(System.currentTimeMillis() + ": robotInit() started");
		
		frontLeft.setInverted(false);
		rearLeft.setInverted(false);
		frontRight.setInverted(true);
		rearRight.setInverted(true);

		navigator.setTurnTimeOutValue(3.0);
		
		zeroAllSensors();
	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	@Override
	public void autonomousInit() {
		System.out.println("Auto start timestamp: " + Timer.getFPGATimestamp());
		zeroAllSensors();
		navigator.initialize();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		navigator.loopNavigation();
	}

	/**
	 * This function is called once each time the robot enters tele-operated mode
	 */
	@Override
	public void teleopInit() {
		System.out.println("TeleOp start timestamp: " + Timer.getFPGATimestamp());
		zeroAllSensors();
		slowSpeed = true;
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		boolean AButton = stick.getRawButton(1);

		navigator.mecanumDrive(stick.getRawAxis(0) * (slowSpeed ? speedFactor : 1.0),
				stick.getRawAxis(1) * (slowSpeed ? speedFactor : 1.0),
				stick.getRawAxis(4) * (slowSpeed ? speedFactor : 1.0), true);

		System.out.println("Encoder Value: " + rearRight.getSelectedSensorPosition(0));

		Timer.delay(0.005); // wait 5ms to avoid hogging CPU cycles

		if (AButton & !lastAButton)
			slowSpeed = !slowSpeed;
		lastAButton = AButton;
	}
	
	public void zeroAllSensors() {
		rearRight.setSelectedSensorPosition(0, 0, 0);
		
		gyro.reset();
	}
}
