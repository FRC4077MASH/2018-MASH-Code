package org.usfirst.frc.team4077.robot;

import org.usfirst.frc.team4077.robot.autonomous.NavigatePID;
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

public class Robot extends IterativeRobot {
  private Drive mDrive = Drive.getInstance();
  private Lift mLift = Lift.getInstance();
  private Climber mClimber = Climber.getInstance();
  private Manipulator mManipulator = Manipulator.getInstance();

  private ControlInterpreter mControlInterpreter =
      ControlInterpreter.getInstance();

  private NavigatePID mNavigator;

  private double mSpeedFactor = 0.75;
  private boolean mSlowSpeed;
  private boolean mLastSlowButton;
  
  //private long mLastTime;

  private double[][] testMovementTypes = new double[][] {
      {0, 0.5, 36.0}, {0 - 0.5, -36.0}, {1, 1.0, 72.0}, {1, -1.0, -72.0},
      {2, 0.5, 90},   {2, -0.5, -90},   {-1, 0.0, 0.0}};

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

    mNavigator = new NavigatePID(testMovementTypes, mDrive, 30);

    mDrive.resetAll();
    mLift.resetAll();

    UsbCamera mC920Cam = CameraServer.getInstance().startAutomaticCapture();
    mC920Cam.setVideoMode(VideoMode.PixelFormat.kYUYV, 640, 480, 15);
    UsbCamera mLifecam3000 = CameraServer.getInstance().startAutomaticCapture();
    mLifecam3000.setVideoMode(VideoMode.PixelFormat.kMJPEG, 320, 240, 24);
  }

  /**
   * This function is run once each time the robot enters autonomous mode
   */
  @Override
  public void autonomousInit() {
    mNavigator.initialize();
  }

  /**
   * This function is called periodically during autonomous
   */
  @Override
  public void autonomousPeriodic() {
    mNavigator.loopNavigation();
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
	  //long currentTime = System.currentTimeMillis();
	  //long timeDifference = mLastTime-currentTime;
	  
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
    if (mControlInterpreter.getLiftBottom()) {
      mLift.liftToHeight(0);
    } else if (mControlInterpreter.getLiftSwitch()) {
      mLift.liftToHeight(30);
    }

    //mLift.runPID(true);
    
    //if (timeDifference >= 1000) {
	//	System.out.println("Left: " + mLift.mLeftLift.getSensorCollection().getQuadraturePosition() + ", Right: " + mLift.mRightLift.getSensorCollection().getQuadraturePosition());
	//}

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

    // NOTE Telemetry/SmartDashboard Prints
    mDrive.printTelemetry();
    mLift.printTelemetry();
    // System.out.println("Loop Time: " + loopTime);
    // System.out.println("Slowmode: " + mSlowSpeed);
    
    //mLastTime = currentTime;

    Timer.delay(0.005);
/*	  mLift.setIndividualMotorPower("L", 0.7);
	  mLift.setIndividualMotorPower("R", 0.7);
	  //System.out.println(mLift.mLeftLift.getSensorCollection().getQuadratureVelocity() + "\t" + mLift.mRightLift.getSensorCollection().getQuadratureVelocity());
  */}
}
