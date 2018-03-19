package org.usfirst.frc.team4077.robot.autonomous;

import org.usfirst.frc.team4077.robot.components.Drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team4077.robot.common.NavXSensor;

public class NavigatePID {
  // Private variables
  private PIDControllerAdvanced mPID;
  private Drive mDrive;
  private NavXSensor mNavX;
  private double[][] mMovementArray;
  private int mMovementArrayStep;
  private double mDistanceReferenceLeft;
  private double mDistanceReferenceRight;
  private double mTurnTimeOutSeconds = 3.0;
  private long mCurrentStepStartTime;
  private double mYawAngle;
  private double mCompletedHeading;

  public NavigatePID(Drive drive, NavXSensor navX, long sampleTime) {
    mPID =
        new PIDControllerAdvanced(PIDControllerAdvanced.REVERSE, sampleTime,
                                  -1.0, 1.0, PIDControllerAdvanced.AUTOMATIC);
    mDrive = drive;
    mNavX = navX;
  }

  public void setTunings(double kP, double kI, double kD) {
    mPID.setTunings(kP, kI, kD);
  }

  public void setSetpoint(double setPoint) { mPID.setSetpoint(setPoint); }

  public void setTurnTimeOutValue(double seconds) {
    mTurnTimeOutSeconds = seconds;
  }

  public int getCurrentType() {
    return (int)mMovementArray[mMovementArrayStep][0];
  }

  public int getCurrentStep() { return mMovementArrayStep; }

  public void initialize(double[][] movementArray) {
    mMovementArray = movementArray;
    mMovementArrayStep = 0;
    mNavX.zeroYaw();
    mDrive.resetAll();
    setSetpoint(0);
    mDistanceReferenceLeft = mDrive.getMotorDistance("L");
    mDistanceReferenceRight = mDrive.getMotorDistance("R");
  }

  public void loopNavigation() {
    mYawAngle = mNavX.getAngleUnlimited();
    mDrive.printTelemetry();
    SmartDashboard.putNumber("Yaw: ", mYawAngle);
    move((int)mMovementArray[mMovementArrayStep][0],
         mMovementArray[mMovementArrayStep][1],
         mMovementArray[mMovementArrayStep][2]);
  }

  private void move(int movementType, double Tp, double goalValue) {
    switch (movementType) {
    case 0:
      moveOnYAxis(goalValue, Tp);
      break;
    case 1:
      moveOnXAxis(goalValue, Tp);
      break;
    case 2:
      rotateOnZAxis(goalValue);
      break;
    case 3:
      pauseForSecondsStopped(goalValue);
      break;
    case -1:
      fullStop();
    default:
      break;
    }
  }

  private void pauseForSecondsStopped(double seconds) {
    fullStop();
    if (System.currentTimeMillis() >=
        mCurrentStepStartTime + (seconds * 1000)) {
      nextMovement();
    }
  }

  private void moveOnYAxis(double goal, double Tp) {
    if (Tp < 0) {
      if (((mDrive.getMotorDistance("L") > (goal + mDistanceReferenceLeft)) &&
           (mDrive.getMotorDistance("R") > (goal + mDistanceReferenceRight)))) {
        loopPID(Tp, 0.0);
      } else {
        nextMovement();
      }
    } else {
      if (((mDrive.getMotorDistance("L") < (goal + mDistanceReferenceLeft)) &&
           (mDrive.getMotorDistance("R") < (goal + mDistanceReferenceRight)))) {

        loopPID(Tp, 0.0);

      } else {
        nextMovement();
      }
    }
  }

  private void moveOnXAxis(double goal, double Tp) {
    if (Tp < 0) {
      if ((mDrive.getMotorDistance("L") > (goal + mDistanceReferenceLeft))) {
        loopPID(0.0, Tp);
      } else {
        nextMovement();
      }
    } else {
      if ((mDrive.getMotorDistance("R") > (goal + mDistanceReferenceLeft))) {
        loopPID(0.0, Tp);
      } else {
        nextMovement();
      }
    }
  }

  private void rotateOnZAxis(double goal) {
    mPID.setSetpoint(goal);
    if (System.currentTimeMillis() <=
        mCurrentStepStartTime + Math.abs(mTurnTimeOutSeconds * 1000)) {
      if (mCompletedHeading > mPID.getSetpoint()) {
        if (mYawAngle > mPID.getSetpoint()) {
          loopPID(0.0, 0.0);
        } else {
          nextMovement();
        }
      } else {
        if (mYawAngle < mPID.getSetpoint()) {
          loopPID(0.0, 0.0);
        } else {
          nextMovement();
        }
      }
    } else {
      nextMovement();
    }
  }

  public void fullStop() { mDrive.driveCartesian(0.0, 0.0, 0.0); }

  public void nextMovement() {
    fullStop();
    mCurrentStepStartTime = System.currentTimeMillis();
    mCompletedHeading = mYawAngle;
    mDistanceReferenceLeft = mDrive.getMotorDistance("L");
    mDistanceReferenceRight = mDrive.getMotorDistance("R");
    mMovementArrayStep++;
  }

  private void loopPID(double forwardsTp, double horizontalTp) {
    double output = mPID.compute(mYawAngle);
    mDrive.driveCartesian(horizontalTp, forwardsTp, output);
  }
}
