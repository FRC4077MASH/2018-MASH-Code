package org.usfirst.frc.team4077.robot.autonomous.automodes.Old;

import org.usfirst.frc.team4077.robot.components.Drive;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TimeBasedPassAutoLine {
  private Drive mDrive;
  private Preferences mPrefs;

  private int mStartPosition;
  private long mDelayAutoTime;
  private long mStartTime;
  private int mNavigationStepCounter;
  private long mCurrentStepStartTime;

  public final static double MAX_NAVIGATION_SPEED = 0.75;

  public final static long CROSS_LINE_TIME_MILLIS = 3000;
  public final static long MOVE_FORWARD_TO_AVOID_SWITCH_TIME_MILLIS = 1000;
  public final static long TURN_TO_45_TIME_MILLIS = 1000;

  public TimeBasedPassAutoLine(Drive drive) {
    mDrive = drive;
    mPrefs = Preferences.getInstance();
  }

  public void initAutoModeFromSmartDashboard() {
    mStartTime = System.currentTimeMillis();
    mNavigationStepCounter = 0;
    mStartPosition = mPrefs.getInt("AutoStartPos", 0);
    mDelayAutoTime = mPrefs.getLong("AutoDelayMilliseconds", 0);
    System.out.println("mStartPosition: " + mStartPosition);
  }

  public void executeLoop() {
    switch (mStartPosition) {
    case 0: // Left
      switch (mNavigationStepCounter) {
      case 0: // Delay
        if (getAutoTime() >= mDelayAutoTime) {
          mCurrentStepStartTime = getAutoTime();
          System.out.println("Running Left");
          mNavigationStepCounter++;
        } else {
          // Do nothing, just delay
        }
        break;
      case 1: // Move forward
        if (getAutoTime() >= CROSS_LINE_TIME_MILLIS + mCurrentStepStartTime) {
          mDrive.driveCartesian(0, 0, 0);
          mCurrentStepStartTime = getAutoTime();
          mNavigationStepCounter++;
        } else {
          mDrive.driveCartesian(0, MAX_NAVIGATION_SPEED, 0);
        }
        break;
      case 2:
        // Do nothing
        break;
      }
      break;

    case 1: // Center
      switch (mNavigationStepCounter) {
      case 0: // Delay
        if (getAutoTime() >= mDelayAutoTime) {
          mCurrentStepStartTime = getAutoTime();
          System.out.println("Running Center");
          mNavigationStepCounter++;
        } else {
          // Do nothing, just delay
        }
        break;
      case 1: // Move forward
        if (getAutoTime() >= CROSS_LINE_TIME_MILLIS + mCurrentStepStartTime) {
          mDrive.driveCartesian(0, 0, 0);
          mCurrentStepStartTime = getAutoTime();
          mNavigationStepCounter++;
        } else {
          mDrive.driveCartesian(0, MAX_NAVIGATION_SPEED / 2, 0);
        }
        break;
      }
      break;

    case 2: // Right
      switch (mNavigationStepCounter) {
      case 0: // Delay
        if (getAutoTime() >= mDelayAutoTime) {
          mCurrentStepStartTime = getAutoTime();
          System.out.println("Running Right");
          mNavigationStepCounter++;
        } else {
          // Do nothing, just delay
        }
        break;
      case 1: // Move forward
        if (getAutoTime() >= CROSS_LINE_TIME_MILLIS + mCurrentStepStartTime) {
          mDrive.driveCartesian(0, 0, 0);
          mCurrentStepStartTime = getAutoTime();
          mNavigationStepCounter++;
        } else {
          mDrive.driveCartesian(0, MAX_NAVIGATION_SPEED, 0);
        }
        break;
      case 2:
        // Do nothing
        break;
      }
      break;

    default:
      break;
    }

    SmartDashboard.putString("Navigation Step: ",
                             Integer.toString(mNavigationStepCounter));
    SmartDashboard.putString("Current Time: ", Long.toString(getAutoTime()));
  }

  private long getAutoTime() { return System.currentTimeMillis() - mStartTime; }
}
