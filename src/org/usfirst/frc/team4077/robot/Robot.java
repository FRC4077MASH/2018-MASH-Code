package org.usfirst.frc.team4077.robot;

import org.usfirst.frc.team4077.robot.common.ControlInterpreter;
import org.usfirst.frc.team4077.robot.common.CrashTracker;
import org.usfirst.frc.team4077.robot.components.Drive;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends IterativeRobot {
  private Drive mDrive = Drive.getInstance();
  private ControlInterpreter mControlInterpreter =
      ControlInterpreter.getInstance();

  private final Joystick mDriveStick = new Joystick(0);

  private double mSpeedFactor = 0.5;
  private boolean mSlowSpeed;
  private boolean mLastSlowButton;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    System.out.println(System.currentTimeMillis() + ": robotInit() started");
    CameraServer.getInstance().startAutomaticCapture();
  }

  /**
   * This function is run once each time the robot enters autonomous mode
   */
  @Override
  public void autonomousInit() {
    System.out.println(System.currentTimeMillis() +
                       ": autonomousInit() started");
  }

  /**
   * This function is called periodically during autonomous
   */
  @Override
  public void autonomousPeriodic() {}

  /**
   * This function is called periodically during operator control
   */
  @Override
  public void teleopPeriodic() {
    boolean currentSlowButton = mControlInterpreter.getSlowButton();

    if (currentSlowButton == !mLastSlowButton) {
      mSlowSpeed = !mSlowSpeed;
    }

    double speed = mSlowSpeed ? mSpeedFactor : 1.0;
    mDrive.driveCartesian(mControlInterpreter.getStrafe() * speed,
                          mControlInterpreter.getThrottle() * speed,
                          mControlInterpreter.getTurn() * speed);

    // NOTE Update Toggle Variables
    mLastSlowButton = currentSlowButton;

    Timer.delay(0.005);
  }
}
