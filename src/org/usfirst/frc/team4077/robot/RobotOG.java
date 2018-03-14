package org.usfirst.frc.team4077.robot;

import org.usfirst.frc.team4077.robot.autonomous.automodes.EncoderBasePassAutoLine;
import org.usfirst.frc.team4077.robot.common.ControlInterpreter;
import org.usfirst.frc.team4077.robot.components.Climber;
import org.usfirst.frc.team4077.robot.components.Drive;
import org.usfirst.frc.team4077.robot.components.Lift;
import org.usfirst.frc.team4077.robot.components.Manipulator;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;

public class RobotOG extends IterativeRobot {
  private Drive mDrive = Drive.getInstance();
  private Lift mLift = Lift.getInstance();
  private Climber mClimber = Climber.getInstance();
  private Manipulator mManipulator = Manipulator.getInstance();

  private ControlInterpreter mControlInterpreter =
      ControlInterpreter.getInstance();

  private EncoderBasePassAutoLine mPassAutoLineTask =
      new EncoderBasePassAutoLine(mDrive);

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
    mLift.enableComponent(true);
    mClimber.enableComponent(true);
    mManipulator.enableComponent(true);

    mDrive.resetAll();
    mLift.resetAll();

    UsbCamera mC920Cam = CameraServer.getInstance().startAutomaticCapture();
    mC920Cam.setVideoMode(VideoMode.PixelFormat.kMJPEG, 640, 480, 10);
    UsbCamera mLifecam3000 = CameraServer.getInstance().startAutomaticCapture();
    mLifecam3000.setVideoMode(VideoMode.PixelFormat.kMJPEG, 320, 240, 15);
  }

  @Override
  public void robotPeriodic() {
    mDrive.printTelemetry();
    mLift.printTelemetry();
  }

  /**
   * This function is run once each time the robot enters autonomous mode
   */
  @Override
  public void autonomousInit() {
    mPassAutoLineTask.initAutoModeFromSmartDashboard();
    mDrive.resetAll();
  }

  /**
   * This function is called periodically during autonomous
   */
  @Override
  public void autonomousPeriodic() {
    mPassAutoLineTask.executeLoop();
  }

  /**
 * This function is run once each time the robot enters teleop mode
 */
  @Override
  public void teleopInit() {
    mLift.resetEncoders();
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
    mDrive.driveCartesian(mControlInterpreter.getStrafe() * speed,
                          mControlInterpreter.getThrottle() * speed,
                          mControlInterpreter.getTurn() * speed);

    if (mControlInterpreter.getPanicButton()) {
      mDrive.driveCartesian(0, -1.0, 0.0);
    }

    // Lift
    mLift.liftWithLimitSwitchLimits(mControlInterpreter.getLift(), true);

    // Climber
    if (mControlInterpreter.getClimbUp()) {
      mClimber.climb(1.0);
    } else if (mControlInterpreter.getClimbDown()) {
      mClimber.climb(-1.0);
    } else {
      mClimber.climb(0.0);
    }

    // Manipulator
    mManipulator.intake(mControlInterpreter.getCollect() -
                        mControlInterpreter.getEjection());

    // NOTE Update Toggle Variables
    mLastSlowButton = currentSlowButton;

    Timer.delay(0.005);
  }
}
