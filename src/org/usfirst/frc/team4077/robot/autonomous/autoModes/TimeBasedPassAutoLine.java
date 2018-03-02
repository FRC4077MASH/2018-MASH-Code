package org.usfirst.frc.team4077.robot.autonomous.autoModes;

import org.usfirst.frc.team4077.robot.components.Drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TimeBasedPassAutoLine {
  private Drive mDrive;

  private int mStartPosition;
  private long mDelayAutoTime;
  private long mStartTime;
  private int mNavigationStepCounter;
  private long mCurrentStepStartTime;

  public final static double MAX_NAVIGATION_SPEED = 0.75;

  public final static long CROSS_LINE_TIME_MILLIS = 2000;
  public final static long MOVE_FORWARD_TO_AVOID_SWITCH_TIME_MILLIS = 1000;
  public final static long TURN_TO_45_TIME_MILLIS = 250;

  public TimeBasedPassAutoLine(Drive mDrive) {
    this.mDrive = mDrive;
    SmartDashboard.putNumber("Auto Start Position: ", 0);
    SmartDashboard.putNumber("Auto Start Position: ", 0);
  }

  public void initAutoModeFromSmartDashboard() {
    mStartTime = System.currentTimeMillis();
    mNavigationStepCounter = 0;
    mStartPosition = (int)SmartDashboard.getNumber("Auto Start Position: ", 0);
    mDelayAutoTime = (long)SmartDashboard.getNumber("Auto Delay Millis: ", 0);
  }

  public void executeLoop() {
    switch (mStartPosition) {
    case 0: // Left
      switch (mNavigationStepCounter) {
      case 0: // Delay
        if (getAutoTime() >= mDelayAutoTime) {
          mCurrentStepStartTime = System.currentTimeMillis();
          mNavigationStepCounter++;
        } else {
          // Do nothing, just delay
        }
        break;
      case 1: // Move forward
        if (getAutoTime() >= CROSS_LINE_TIME_MILLIS + mCurrentStepStartTime) {
          mDrive.driveCartesian(0, 0, 0);
          mCurrentStepStartTime = System.currentTimeMillis();
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
          mCurrentStepStartTime = System.currentTimeMillis();
          mNavigationStepCounter++;
        } else {
          // Do nothing, just delay
        }
        break;
      case 1: // Turn to 45
        if (getAutoTime() >= TURN_TO_45_TIME_MILLIS + mCurrentStepStartTime) {
          mDrive.driveCartesian(0, 0, 0);
          mCurrentStepStartTime = System.currentTimeMillis();
          mNavigationStepCounter++;
        } else {
          mDrive.driveCartesian(0, 0, MAX_NAVIGATION_SPEED);
        }
      case 2: // Move forward
        if (getAutoTime() >=
            MOVE_FORWARD_TO_AVOID_SWITCH_TIME_MILLIS + mCurrentStepStartTime) {
          mDrive.driveCartesian(0, 0, 0);
          mCurrentStepStartTime = System.currentTimeMillis();
          mNavigationStepCounter++;
        } else {
          mDrive.driveCartesian(0, MAX_NAVIGATION_SPEED, 0);
        }
      case 3: // Turn back to 0
        if (getAutoTime() >= TURN_TO_45_TIME_MILLIS + mCurrentStepStartTime) {
          mDrive.driveCartesian(0, 0, 0);
          mCurrentStepStartTime = System.currentTimeMillis();
          mNavigationStepCounter++;
        } else {
          mDrive.driveCartesian(0, 0, -MAX_NAVIGATION_SPEED);
        }
      case 4: // Move forward
        if (getAutoTime() >= CROSS_LINE_TIME_MILLIS + mCurrentStepStartTime) {
          mDrive.driveCartesian(0, 0, 0);
          mCurrentStepStartTime = System.currentTimeMillis();
          mNavigationStepCounter++;
        } else {
          mDrive.driveCartesian(0, MAX_NAVIGATION_SPEED, 0);
        }
        break;
      case 5:
        // Do nothing
        break;
      }
      break;

    case 2: // Right
      switch (mNavigationStepCounter) {
      case 0: // Delay
        if (getAutoTime() >= mDelayAutoTime) {
          mCurrentStepStartTime = System.currentTimeMillis();
          mNavigationStepCounter++;
        } else {
          // Do nothing, just delay
        }
        break;
      case 1: // Move forward
        if (getAutoTime() >= CROSS_LINE_TIME_MILLIS + mCurrentStepStartTime) {
          mDrive.driveCartesian(0, 0, 0);
          mCurrentStepStartTime = System.currentTimeMillis();
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
  }

  private long getAutoTime() { return System.currentTimeMillis() - mStartTime; }
}
