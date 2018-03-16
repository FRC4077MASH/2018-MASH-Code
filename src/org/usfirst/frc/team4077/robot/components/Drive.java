package org.usfirst.frc.team4077.robot.components;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive {
  // NOTE Private Objects
  private static Drive mInstance = new Drive();

  private TalonSRX mFrontLeft = new TalonSRX(4);
  private TalonSRX mRearLeft = new TalonSRX(3);
  private TalonSRX mFrontRight = new TalonSRX(8);
  private TalonSRX mRearRight = new TalonSRX(7);

  // NOTE Public Constants COMMENT OUT VALUES FOR COMP ROBOT
  public static final double WHEEL_DIAMETER = 4.0;                  // 6.0;
  public static final int ENCODER_TICKS_PER_ROTATION_LEFT = 10800;  // 1000;
  public static final int ENCODER_TICKS_PER_ROTATION_RIGHT = 10800; // 1000;
  public static final double DEADBAND = 0.02;

  // NOTE Private Variables
  private boolean mIsEnabled = false;

  // NOTE Constructor
  private Drive() {
    mFrontLeft.setInverted(true);

    mRearLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1,
                                   10);
    mRearLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    mRearLeft.setSensorPhase(true);
    mRearLeft.setInverted(true);

    mFrontRight.setInverted(false);

    mRearRight.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1,
                                    10);
    mRearRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    mRearRight.setSensorPhase(false);
    mRearRight.setInverted(false);
  }

  public static Drive getInstance() { return mInstance; }

  // NOTE Setters
  public void enableComponent(boolean enabled) { mIsEnabled = enabled; }

  public void driveCartesian(double xIn, double yIn, double rotation) {
    if (mIsEnabled) {
      double x = applyDeadband(xIn, DEADBAND);
      double y = applyDeadband(yIn, DEADBAND);
      double[] wheelSpeeds = new double[4];

      wheelSpeeds[0] = x + y + rotation;  // Front left
      wheelSpeeds[1] = -x + y - rotation; // Front right
      wheelSpeeds[2] = -x + y + rotation; // Rear left
      wheelSpeeds[3] = x + y - rotation;  // Rear right

      normalize(wheelSpeeds);
      mFrontLeft.set(ControlMode.PercentOutput, wheelSpeeds[0]);
      mFrontRight.set(ControlMode.PercentOutput, wheelSpeeds[1]);
      mRearLeft.set(ControlMode.PercentOutput, wheelSpeeds[2]);
      mRearRight.set(ControlMode.PercentOutput, wheelSpeeds[3]);
    }
  }

  public void setIndividualMotorPower(String motorAbbreviation, double power) {
    if (mIsEnabled) {
      switch (motorAbbreviation) {
      case "FL":
        mFrontLeft.set(ControlMode.PercentOutput, power);
        break;
      case "RL":
        mRearLeft.set(ControlMode.PercentOutput, power);
        break;
      case "FR":
        mFrontRight.set(ControlMode.PercentOutput, power);
        break;
      case "RR":
        mRearRight.set(ControlMode.PercentOutput, power);
        break;
      default:
        break;
      }
    }
  }

  public void resetAll() {
    resetEncoders();
    resetSensors();
  }

  public void resetEncoders() {
    mRearLeft.setSelectedSensorPosition(0, 0, 10);
    mRearLeft.getSensorCollection().setQuadraturePosition(0, 10);
    mRearRight.setSelectedSensorPosition(0, 0, 10);
    mRearRight.getSensorCollection().setQuadraturePosition(0, 10);
  }

  public void resetSensors() {
    // Nothing yet
  }

  public void printTelemetry() {
    SmartDashboard.putString("Left Distance: ",
                             Double.toString(getMotorDistance("L")));
    SmartDashboard.putString("Right Distance: ",
                             Double.toString(getMotorDistance("R")));
  }

  // NOTE Getters
  public boolean getEnableStatus() { return mIsEnabled; }

  public double getMotorDistance(String motorAbbreviation) {
    switch (motorAbbreviation) {
    case "L":
      return rotationsToInches(
          mRearLeft.getSelectedSensorPosition(
              0) / // getSensorCollection().getQuadraturePosition() /
          (double)ENCODER_TICKS_PER_ROTATION_LEFT);
    case "R":
      return -rotationsToInches(
          mRearRight.getSelectedSensorPosition(
              0) / // getSensorCollection().getQuadraturePosition() /
          (double)ENCODER_TICKS_PER_ROTATION_RIGHT);
    default:
      return 0;
    }
  }

  // NOTE Private conversion Methods
  private double rotationsToInches(double rotations) {
    return rotations * (WHEEL_DIAMETER * Math.PI);
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
