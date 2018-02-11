package org.usfirst.frc.team4077.robot.components;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.hal.HAL;
import edu.wpi.first.wpilibj.hal.FRCNetComm.tInstances;
import edu.wpi.first.wpilibj.hal.FRCNetComm.tResourceType;

public class Drive {
  // NOTE Private Objects
  private static Drive mInstance = new Drive();

  private WPI_TalonSRX mFrontLeft = new WPI_TalonSRX(4);
  private WPI_TalonSRX mRearLeft = new WPI_TalonSRX(3);
  private WPI_TalonSRX mFrontRight = new WPI_TalonSRX(8);
  private WPI_TalonSRX mRearRight = new WPI_TalonSRX(7);

  // NOTE Public Constants
  public static final double WHEEL_DIAMETER = 6.0;
  public static final int ENCODER_TICKS_PER_ROTATION = 1000;
  public static final double ENCODER_TO_INCH_RATIO =
      ENCODER_TICKS_PER_ROTATION / (WHEEL_DIAMETER * Math.PI);
  public static final double DEADBAND = 0.02;

  // NOTE Private Variables
  private boolean mIsEnabled = false;

  // NOTE Constructor
  private Drive() {
    mFrontLeft.setInverted(true);
    mRearLeft.setInverted(true);
    mFrontRight.setInverted(false);
    mRearRight.setInverted(false);
  }

  public static Drive getInstance() { return mInstance; }

  // NOTE Setters
  public void enableComponent(boolean enabled) { mIsEnabled = enabled; }

  public void driveCartesian(double xIn, double yIn, double rotation) {
    double x = xIn;
    double y = -yIn;
    double[] wheelSpeeds = new double[4];

    applyDeadband(x, DEADBAND);
    applyDeadband(x, DEADBAND);

    wheelSpeeds[0] = x + y + rotation;  // Front left
    wheelSpeeds[1] = -x + y - rotation; // Front right
    wheelSpeeds[2] = -x + y + rotation; // Rear left
    wheelSpeeds[3] = x + y - rotation;  // Rear right

    normalize(wheelSpeeds);
    mFrontLeft.set(wheelSpeeds[0]);
    mFrontRight.set(wheelSpeeds[1]);
    mRearLeft.set(wheelSpeeds[2]);
    mRearRight.set(wheelSpeeds[3]);
  }

  public void setIndividualMotorPower(String motorAbbreviation, double power) {
    switch (motorAbbreviation) {
    case "FL":
      mFrontLeft.set(power);
      break;
    case "RL":
      mRearLeft.set(power);
      break;
    case "FR":
      mFrontRight.set(power);
      break;
    case "RR":
      mRearRight.set(power);
      break;
    }
  }

  public void resetAll() {
    resetEncoders();
    resetSensors();
  }

  public void resetEncoders() {
    mFrontLeft.setSelectedSensorPosition(0, 0, 0);
    mRearLeft.setSelectedSensorPosition(0, 0, 0);
    mFrontRight.setSelectedSensorPosition(0, 0, 0);
    mRearRight.setSelectedSensorPosition(0, 0, 0);
  }

  public void resetSensors() {
    // Nothing yet
  }

  // NOTE Getters
  public boolean getEnableStatus() { return mIsEnabled; }

  public double getMotorDistance(String motorAbbreviation) {
    switch (motorAbbreviation) {
    case "FL":
      return rotationsToInches(mFrontLeft.getSelectedSensorPosition(0) /
                               1000.0);
    case "RL":
      return rotationsToInches(mRearLeft.getSelectedSensorPosition(0) / 1000.0);
    case "FR":
      return rotationsToInches(mFrontRight.getSelectedSensorPosition(0) /
                               1000.0);
    case "RR":
      return rotationsToInches(mRearRight.getSelectedSensorPosition(0) /
                               1000.0);
    default:
      return 0;
    }
  }

  // NOTE Private conversion Methods
  private double rotationsToInches(double rotations) {
    return rotations * (WHEEL_DIAMETER * Math.PI);
  }

  private double inchesToRotations(double inches) {
    return inches / (WHEEL_DIAMETER * Math.PI);
  }

  private void normalize(double[] wheelSpeeds) {
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

  private double applyDeadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }
}
