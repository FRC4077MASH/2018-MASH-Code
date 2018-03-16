package org.usfirst.frc.team4077.robot.autonomous.automodes.Old;

import org.usfirst.frc.team4077.robot.components.Drive;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EncoderBasePassAutoLine {
  private Drive mDrive;
  private Preferences mPrefs;

  private int mStartPosition;
  private long mDelayAutoTime;
  private long mStartTime;
  private int mNavigationStepCounter;
  private double mCurrentStepEncoderVal;

  public final static double MAX_NAVIGATION_SPEED = 0.75;

  public EncoderBasePassAutoLine(Drive drive) {
    mDrive = drive;
    mPrefs = Preferences.getInstance();
  }

  public void initAutoModeFromSmartDashboard() {
    mDrive.driveCartesian(0, 0, 0);
    mDrive.resetAll();
    mStartTime = System.currentTimeMillis();
    mNavigationStepCounter = 0;
    mStartPosition = mPrefs.getInt("AutoStartPos", 0);
    mDelayAutoTime = mPrefs.getLong("AutoDelayMilliseconds", 0);
  }

  public void executeLoop() {
    switch (mStartPosition) {
    case 0: // Left
      switch (mNavigationStepCounter) {
      case 0: // Delay
        if (getAutoTime() >= mDelayAutoTime) {
          mCurrentStepEncoderVal = getEncoderDistance();
          System.out.println("Running Left");
          mNavigationStepCounter++;
        } else {
          // Do nothing, just delay
        }
        break;
      case 1: // Move forward
        if (getEncoderDistance() >= 180.0 + mCurrentStepEncoderVal) {
          mDrive.driveCartesian(0, 0, 0);
          mCurrentStepEncoderVal = getEncoderDistance();
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
          mCurrentStepEncoderVal = getEncoderDistance();
          System.out.println("Running Center");
          mNavigationStepCounter++;
        } else {
          // Do nothing, just delay
        }
        break;
      case 1: // Move forward
        if (getEncoderDistance() >= 24 + mCurrentStepEncoderVal) {
          mDrive.driveCartesian(0, 0, 0);
          mCurrentStepEncoderVal = getEncoderDistance();
          mNavigationStepCounter++;
        } else {
          mDrive.driveCartesian(0, MAX_NAVIGATION_SPEED / 2, 0);
        }
        break;
      case 2: // Turn
        if (mDrive.getMotorDistance("L") >= 24.4 + mCurrentStepEncoderVal) {
          mDrive.driveCartesian(0, 0, 0);
          mCurrentStepEncoderVal = getEncoderDistance();
          mNavigationStepCounter++;
        } else {
          mDrive.driveCartesian(0, 0, MAX_NAVIGATION_SPEED / 2);
        }
        break;
      case 3: // Move forward
        if (getEncoderDistance() >= 24 + mCurrentStepEncoderVal) {
          mDrive.driveCartesian(0, 0, 0);
          mCurrentStepEncoderVal = getEncoderDistance();
          mNavigationStepCounter++;
        } else {
          mDrive.driveCartesian(0, MAX_NAVIGATION_SPEED / 2, 0);
        }
        break;
      case 4: // Turn
        if (mDrive.getMotorDistance("L") >= 24.4 + mCurrentStepEncoderVal) {
          mDrive.driveCartesian(0, 0, 0);
          mCurrentStepEncoderVal = getEncoderDistance();
          mNavigationStepCounter++;
        } else {
          mDrive.driveCartesian(0, 0, MAX_NAVIGATION_SPEED / 2);
        }
        break;
      case 5:
        if (getEncoderDistance() >= 24 + mCurrentStepEncoderVal) {
          mDrive.driveCartesian(0, 0, 0);
          mCurrentStepEncoderVal = getEncoderDistance();
          mNavigationStepCounter++;
        } else {
          mDrive.driveCartesian(0, MAX_NAVIGATION_SPEED / 2, 0);
        }
        break;
      case 6:
        // Do nothing
        break;
      }
      break;

    case 2: // Right
      switch (mNavigationStepCounter) {
      case 0: // Delay
        if (getAutoTime() >= mDelayAutoTime) {
          mCurrentStepEncoderVal = getEncoderDistance();
          System.out.println("Running Right");
          mNavigationStepCounter++;
        } else {
          // Do nothing, just delay
        }
        break;
      case 1: // Move forward
        if (getEncoderDistance() >= 120.0 + mCurrentStepEncoderVal) {
          mDrive.driveCartesian(0, 0, 0);
          mCurrentStepEncoderVal = getEncoderDistance();
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
  private double getEncoderDistance() {
    return (mDrive.getMotorDistance("L") + mDrive.getMotorDistance("R")) / 2;
  }
}
