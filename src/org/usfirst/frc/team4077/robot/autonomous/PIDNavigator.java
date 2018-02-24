/*package org.usfirst.frc.team4077.robot.autonomous;

import org.usfirst.frc.team4077.robot.components.Drive;

import com.ctre.phoenix.motorcontrol.can.*;
import com.kauailabs.navx.frc.AHRS;

// Tested: No, fixed error with setpoint not changing in PIDControllerAdvanced.
// Will test.
public class PIDNavigator {
  // Public variables
  public double ENCODER_TICKS_PER_INCH;

  public PIDControllerAdvanced pid;

  public enum DRIVE_TYPE { TANK, MECANUM }

  // Private variables
  private Drive drive;
  private AHRS gyro;
  private double[][] movementCommandArray;
  private int movementArrayStep;
  private double encoderPositionReference;

  private double turnTimeOutSeconds;
  private long currentStepStartTime;

  private double yawAngle;
  private double completedHeading;

  public PIDNavigator(double[][] m_movementCommandArray, Drive m_drive,
                      DRIVE_TYPE m_driveType, AHRS m_gyro,
                      int m_encoderTicksPerWheelRevolution,
                      double m_wheelDiameter, double p, double i, double d,
                      long sampleTime) {

    pid = new PIDControllerAdvanced(PIDControllerAdvanced.DIRECT, sampleTime,
                                    -1, 1, PIDControllerAdvanced.AUTOMATIC);
    pid.setTunings(p, i, d, PIDControllerAdvanced.P_ON_M);
    pid.setSetpoint(0);

    ENCODER_TICKS_PER_INCH =
        m_encoderTicksPerWheelRevolution / (Math.PI * m_wheelDiameter);
    movementCommandArray = m_movementCommandArray;

    gyro = m_gyro;
  }

  public void initialize() {
    encoderPositionReference = wheelEncoderValue();
    movementArrayStep = 0;
  }

  public double wheelEncoderValue() {
    return drive.getMotorDistance("R");
  }

  public void loopNavigation() {
    yawAngle = gyro.getAngle();
    //System.out.println("Yaw , goal: " + yawAngle + ", " +
                       movementCommandArray[movementArrayStep][2]);

    move((int)movementCommandArray[movementArrayStep][0],
         movementCommandArray[movementArrayStep][1],
         movementCommandArray[movementArrayStep][2]);
  }

  public void setTurnTimeOutValue(double seconds) {
    turnTimeOutSeconds = seconds;
  }

  // Private methods
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

  private double convertInchesToTankEncoderTicks(double inches) {
    return ENCODER_TICKS_PER_INCH * inches;
  }

  private double convertInchesToStrafeEncoderTicks(double inches) {
    return (ENCODER_TICKS_PER_INCH * inches) / 2;
  }

  private void pauseForSecondsStopped(double seconds) {
    fullStop();
    if (System.currentTimeMillis() >= currentStepStartTime + (seconds * 1000)) {
      nextMovement();
    }
  }

  private void moveOnYAxis(double goal, double Tp) {
    if (Tp < 0) {
      if (wheelEncoderValue() >
          convertInchesToTankEncoderTicks(goal) + encoderPositionReference) {
        loopMecanumPID(Tp, 0);
      } else {
        nextMovement();
      }
    } else {
      if (wheelEncoderValue() <
          convertInchesToTankEncoderTicks(goal) + encoderPositionReference) {

        loopMecanumPID(Tp, 0);

      } else {
        nextMovement();
      }
    }
  }

  private void moveOnXAxis(double goal, double Tp) {

    if (Tp < 0) {
      if (wheelEncoderValue() >
          convertInchesToStrafeEncoderTicks(goal) + encoderPositionReference) {
        loopMecanumPID(0, Tp);
      } else {
        nextMovement();
      }
    } else {
      if (wheelEncoderValue() <
          convertInchesToStrafeEncoderTicks(goal) + encoderPositionReference) {
        loopMecanumPID(0, Tp);
      } else {
        nextMovement();
      }
    }
  }

  private void rotateOnZAxis(double goal) {
    pid.setSetpoint(goal);
    if (System.currentTimeMillis() <=
        currentStepStartTime + Math.abs(turnTimeOutSeconds * 1000)) {
      if (completedHeading > pid.getSetpoint()) {
        if (yawAngle > pid.getSetpoint()) {

            loopMecanumPID(0, 0);
        } else {
          nextMovement();
        }
      } else {
        if (yawAngle < pid.getSetpoint()) {
            loopMecanumPID(0, 0);
        } else {
          nextMovement();
        }
      }
    } else {
      nextMovement();
    }
  }

  private void fullStop() {
    if (driveType == DRIVE_TYPE.MECANUM) {
      mecanumDrive(0.0, 0.0, 0.0, false);
    } else {
      tankDrive(0.0, 0.0);
    }
  }

  private void nextMovement() {
    fullStop();
    currentStepStartTime = System.currentTimeMillis();
    completedHeading = yawAngle;
    encoderPositionReference = wheelEncoderValue();
    movementArrayStep++;
  }

  private void loopMecanumPID(double forwardsTp, double horizontalTp) {
    double output = pid.compute(yawAngle);
    mecanumDrive(horizontalTp, forwardsTp, output, false);
  }

  private void loopTankPID(double Tp) {
    double output = pid.compute(yawAngle);

    tankDrive(Tp + output, Tp - output);
  }

  public void mecanumDrive(double x, double y, double rotation,
                           boolean invertY) {
    double xIn = x;
    double yIn = invertY ? -y : y;

    double[] wheelSpeeds = new double[4];

    wheelSpeeds[0] = xIn + yIn + rotation;  // Front left
    wheelSpeeds[1] = -xIn + yIn - rotation; // Front right
    wheelSpeeds[2] = -xIn + yIn + rotation; // Rear left
    wheelSpeeds[3] = xIn + yIn - rotation;  // Rear right

    normalize(wheelSpeeds);
    frontLeft.set(wheelSpeeds[0]);
    frontRight.set(wheelSpeeds[1]);
    rearLeft.set(wheelSpeeds[2]);
    rearRight.set(wheelSpeeds[3]);
  }

  public void tankDrive(double leftValue, double rightValue) {
    double[] wheelSpeeds = new double[4];

    wheelSpeeds[0] = leftValue;  // Front left
    wheelSpeeds[1] = rightValue; // Front right
    wheelSpeeds[2] = leftValue;  // Rear left
    wheelSpeeds[3] = rightValue; // Rear right

    normalize(wheelSpeeds);
    frontLeft.set(wheelSpeeds[0]);
    frontRight.set(wheelSpeeds[1]);
    rearLeft.set(wheelSpeeds[2]);
    rearRight.set(wheelSpeeds[3]);
  }

  protected static double clip(double value, double min, double max) {
    if (value > max)
      return max;
    if (value < min)
      return min;
    return value;
  }

  protected static void normalize(double[] wheelSpeeds) {
    double maxMagnitude = Math.abs(wheelSpeeds[0]);
    for (int i = 1; i < 4; i++) {
      double temp = Math.abs(wheelSpeeds[i]);
      if (maxMagnitude < temp) {
        maxMagnitude = temp;
      }
    }
    if (maxMagnitude > 1.0) {
      for (int i = 0; i < 4; i++) {
        wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
      }
    }
  }
}
*/
