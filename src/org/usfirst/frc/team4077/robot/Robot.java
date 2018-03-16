package org.usfirst.frc.team4077.robot;

import org.usfirst.frc.team4077.robot.autonomous.automodes.AutoDoSwitch;
import org.usfirst.frc.team4077.robot.autonomous.automodes.AutoModeSelector;
import org.usfirst.frc.team4077.robot.autonomous.automodes.TestNavigationAutoMode;
import org.usfirst.frc.team4077.robot.common.ControlInterpreter;
import org.usfirst.frc.team4077.robot.components.Drive;
import org.usfirst.frc.team4077.robot.components.Lift;
import org.usfirst.frc.team4077.robot.components.Manipulator;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends IterativeRobot {

  private Drive mDrive = Drive.getInstance();
  private Lift mLift = Lift.getInstance();
  private Manipulator mHand = Manipulator.getInstance();

  private ControlInterpreter mControlInterpreter =
      ControlInterpreter.getInstance();

  // private EncoderBasePassAutoLine mPassAutoLineTask = new
  // EncoderBasePassAutoLine(mDrive);

  private AutoDoSwitch mTestAuto;

  private double mSpeedFactor = 0.75;
  private boolean mSlowSpeed;
  private boolean mLastSlowButton;

  /**
   * This function is run when the robot is first started up and should be
   * used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    mDrive.enableComponent(true);

    mDrive.resetAll();

    mTestAuto = new AutoDoSwitch(mDrive, mLift, mHand);

    UsbCamera mLifecam3000 = CameraServer.getInstance().startAutomaticCapture();
    mLifecam3000.setVideoMode(VideoMode.PixelFormat.kMJPEG, 320, 240, 15);
  }

  @Override
  public void robotPeriodic() {}

  /**
   * This function is run once each time the robot enters autonomous mode
   */
  @Override
  public void autonomousInit() {
    mTestAuto.init(Alliance.Red, "LRL",
                   AutoModeSelector.AutoStartPosition.START_LEFT, 1000);
  }

  /**
   * This function is called periodically during autonomous
   */
  @Override
  public void autonomousPeriodic() {
    System.out.println("Encoder Values: " + mDrive.getMotorDistance("L") +
                       ", " + mDrive.getMotorDistance("R"));
    mTestAuto.executeLoop();
  }

  /**
 * This function is run once each time the robot enters teleop mode
 */
  @Override
  public void teleopInit() {
    mDrive.resetEncoders();
  }

  /**
   * This function is called periodically during operator control
   */
  @Override
  public void teleopPeriodic() {
    // Drive
    boolean currentSlowButton = mControlInterpreter.getSlowButton();

    if (currentSlowButton && !mLastSlowButton) {
      mSlowSpeed = !mSlowSpeed;
    }

    double speed = mSlowSpeed ? mSpeedFactor : 1.0;
    mDrive.driveCartesian(0, mControlInterpreter.getThrottle() * speed,
                          mControlInterpreter.getTurn() * speed);

    if (mControlInterpreter.getPanicButton()) {
      mDrive.driveCartesian(0, -1.0, 0.0);
    }

    // NOTE Update Toggle Variables
    mLastSlowButton = currentSlowButton;

    Timer.delay(0.005);
  }
}
