package org.usfirst.frc.team4077.robot.components;

import org.usfirst.frc.team4077.robot.autonomous.PIDControllerAdvanced;

import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Lift {
  // NOTE Private Objects
  private static Lift mInstance = new Lift();

  public WPI_TalonSRX mLeftLift = new WPI_TalonSRX(2);
  public WPI_TalonSRX mRightLift = new WPI_TalonSRX(6);

  private DigitalInput mLeftTopLimit = new DigitalInput(1);
  private DigitalInput mRightTopLimit = new DigitalInput(0);

  private AnalogInput mLeftLiftPotentiometer = new AnalogInput(0);
  private AnalogInput mRightLiftPotentiometer = new AnalogInput(1);

  private PIDControllerAdvanced mPIDLeftLift;
  private PIDControllerAdvanced mPIDRightLift;

  // NOTE Public Constants
  public static final double DEADBAND = 0.02;
  public static final double SPOOL_SIZE = 1.0;
  public static final double POTENTIOMETER_VALS_PER_INCH =
      (4096 / 10.0) / (SPOOL_SIZE * Math.PI);

  public static final double LEFT_MOTOR_SPEED_SCALE = 1.0;
  public static final double RIGHT_MOTOR_SPEED_SCALE = 1.1375;

  // NOTE Private Variables
  private boolean mIsEnabled = false;

  // NOTE Constructor
  private Lift() {
    mLeftLift.setInverted(false);
    mRightLift.setInverted(true);

    mPIDLeftLift =
        new PIDControllerAdvanced(PIDControllerAdvanced.DIRECT, 5, -1.0, 1.0,
                                  PIDControllerAdvanced.AUTOMATIC);
    mPIDRightLift =
        new PIDControllerAdvanced(PIDControllerAdvanced.DIRECT, 5, -1.0, 1.0,
                                  PIDControllerAdvanced.AUTOMATIC);

    mPIDLeftLift.setTunings(0.6, 0.00004, 0.0);
    mPIDRightLift.setTunings(0.6, 0.00004, 0.0);
    mPIDRightLift.setSetpoint(0.5);
  }

  public static Lift getInstance() { return mInstance; }

  // NOTE Setters
  public void enableComponent(boolean enabled) {
    mIsEnabled = enabled;
    if (mIsEnabled) {
      mPIDLeftLift.initialize();
      mPIDRightLift.initialize();
    }
  }

  public void liftWithLimitSwitchLimits(double speed, boolean safetyLimitOn) {
    if (mIsEnabled) {
      double leftPower = applyDeadband(speed, DEADBAND);
      double rightPower = applyDeadband(speed, DEADBAND);

      leftPower = limitLiftToSwitches(leftPower, "L");
      rightPower = limitLiftToSwitches(rightPower, "R");

      SmartDashboard.putString("Left Lift Power: ", Double.toString(leftPower));
      SmartDashboard.putString("Right Lift Power: ",
                               Double.toString(rightPower));

      mLeftLift.set(leftPower * LEFT_MOTOR_SPEED_SCALE);
      mRightLift.set(rightPower * RIGHT_MOTOR_SPEED_SCALE);
    }
  }

  public void liftToHeight(double inches) {
    mPIDLeftLift.setSetpoint(inches);
    mPIDRightLift.setSetpoint(inches);
  }

  public void runPID(boolean safetyLimitOn) {
    if (mIsEnabled) {
      double leftPower =
          mPIDLeftLift.compute(getLinearLiftDistanceFromPotentiometer("L"));
      double rightPower =
          mPIDRightLift.compute(getLinearLiftDistanceFromPotentiometer("R"));

      leftPower = applyDeadband(leftPower, DEADBAND);
      rightPower = applyDeadband(rightPower, DEADBAND);

      if (safetyLimitOn) {
        leftPower = limitLiftToSwitches(leftPower, "L");
        rightPower = limitLiftToSwitches(rightPower, "R");
      }

      SmartDashboard.putString("Lift Goal: ",
                               Double.toString(mPIDLeftLift.getSetpoint()));
      SmartDashboard.putString("Left PID Output: ", Double.toString(leftPower));
      SmartDashboard.putString("Right PID Output: ",
                               Double.toString(rightPower));

      mLeftLift.set(leftPower);
      // mRightLift.set(rightPower);
    }
  }

  public void setIndividualMotorPower(String motorAbbreviation, double power) {
    if (mIsEnabled) {
      switch (motorAbbreviation) {
      case "L":
        mLeftLift.set(power);
        break;
      case "R":
        mRightLift.set(power);
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
    // Nothing
  }

  public void resetSensors() {
    // Nothing yet
    // TODO Add code for other sensors
  }

  public void printTelemetry() {
    SmartDashboard.putString("Left Limit: ",
                             Boolean.toString(mLeftTopLimit.get()));
    SmartDashboard.putString("Right Limit: ",
                             Boolean.toString(mRightTopLimit.get()));
  }

  // NOTE Getters
  public boolean getEnableStatus() { return mIsEnabled; }

  // NOTE Private conversion Methods
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

  private double getLinearLiftDistanceFromPotentiometer(String motor) {
    switch (motor) {
    case "L":
      return (-mLeftLiftPotentiometer.getValue() + 4096) /
          POTENTIOMETER_VALS_PER_INCH;
    case "R":
      return mRightLiftPotentiometer.getValue() / POTENTIOMETER_VALS_PER_INCH;
    default:
      return 0;
    }
  }

  private double limitLiftToSwitches(double power, String motorAbbreviation) {
    switch (motorAbbreviation) {
    case "L":
      if (mLeftTopLimit.get()) {
        if (power <= 0.0) {
          return power;
        } else {
          return 0.0;
        }
      } else {
        return power;
      }
    case "R":
      if (mRightTopLimit.get()) {
        if (power <= 0.0) {
          return power;
        } else {
          return 0.0;
        }
      } else {
        return power;
      }
    default:
      return 0;
    }
  }
}
