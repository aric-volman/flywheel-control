// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/* CTRE imports */
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/* Optional
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
*/

public class Shooter extends SubsystemBase {

  private double flywheelTolerance = 0.05; // Tolerance of PID controller

  private boolean override = false; // Helps us switch from manual to auto

  private Timer overrideTimer = new Timer(); // We want a toggle button of some sorts

  private double overrideTime = 1.0;

  /* Initialize the talons */
  private final WPI_TalonSRX leftFlywheel = new WPI_TalonSRX(Constants.ShooterPorts.LeftFlywheelPort);
  private final WPI_TalonSRX rightFlywheel = new WPI_TalonSRX(Constants.ShooterPorts.RightFlywheelPort);

  /* Optional talons if you really want to shoot...
  private final WPI_TalonSRX pivot = new WPI_TalonSRX(Constants.BallHandlerPorts.pivotPort);
  private final WPI_VictorSPX roller = new WPI_VictorSPX(Constants.BallHandlerPorts.rollerPort);
  */
  
  /* Initialize PID controllers */
  private final PIDController leftFlywheelPID = new PIDController(Constants.leftFlywheelPIDConsts.pidP, Constants.leftFlywheelPIDConsts.pidI, Constants.leftFlywheelPIDConsts.pidD);
  private final PIDController rightFlywheelPID = new PIDController(Constants.rightFlywheelPIDConsts.pidP, Constants.rightFlywheelPIDConsts.pidI, Constants.rightFlywheelPIDConsts.pidD);

  /* Initialize feedforward */
  private SimpleMotorFeedforward leftFlywheelFF = new SimpleMotorFeedforward(Constants.leftFlywheelFF.kS, Constants.leftFlywheelFF.kV, Constants.leftFlywheelFF.kA);
  private SimpleMotorFeedforward rightFlywheelFF = new SimpleMotorFeedforward(Constants.rightFlywheelFF.kS, Constants.rightFlywheelFF.kV, Constants.rightFlywheelFF.kA);

  /* Optional Network table input that helps with testing:
  private ShuffleboardTab pidTab = Shuffleboard.getTab("Flywheel PID");
  private NetworkTableEntry leftFlywheelInputPIDP = pidTab.add("Left Flywheel PID P", Constants.leftFlywheelPIDConsts.pidP).getEntry();
  private NetworkTableEntry leftFlywheelInputPIDI = pidTab.add("Left Flywheel PID I", Constants.leftFlywheelPIDConsts.pidI).getEntry();
  private NetworkTableEntry leftFlywheelInputPIDD = pidTab.add("Left Flywheel PID D", Constants.leftFlywheelPIDConsts.pidD).getEntry();

  private NetworkTableEntry rightFlywheelInputPIDP = pidTab.add("Right Flywheel PID P", Constants.rightFlywheelPIDConsts.pidP).getEntry();
  private NetworkTableEntry rightFlywheelInputPIDI = pidTab.add("Right Flywheel PID I", Constants.rightFlywheelPIDConsts.pidI).getEntry();
  private NetworkTableEntry rightFlywheelInputPIDD = pidTab.add("Right Flywheel PID D", Constants.rightFlywheelPIDConsts.pidD).getEntry();
  */

  /** Creates a new Shooter. */
  public Shooter() {
    /* Important for using the pivot and roller
    pivot.configFactoryDefault();
    pivot.setInverted(false);
    pivot.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    pivot.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    pivot.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    pivot.configPeakCurrentLimit(20, 5);
    pivot.setNeutralMode(NeutralMode.Brake);
    roller.configFactoryDefault();
    roller.setInverted(false);
    */

    /* Flywheel talon settings */
    leftFlywheel.configFactoryDefault();
    leftFlywheel.setInverted(true);
    leftFlywheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    rightFlywheel.configFactoryDefault();
    rightFlywheel.setInverted(false);
    rightFlywheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    /* PID settings */
    leftFlywheelPID.setTolerance(flywheelTolerance);
    rightFlywheelPID.setTolerance(flywheelTolerance);

    overrideTimer.start(); // Start timer
    overrideTimer.reset(); // Reset timer
  }

  // ------------------- FLYWHEEL METHODS ------------------- //
  public void resetFlywheelEncoders() {
    leftFlywheel.setSelectedSensorPosition(0, 0, 10);
    rightFlywheel.setSelectedSensorPosition(0, 0, 10);
  }

  public double getLeftRPM() {
    return ((leftFlywheel.getSelectedSensorVelocity() * 10)/4096.0)*60.0;
  }

  public double getRightRPM() {
    return ((rightFlywheel.getSelectedSensorVelocity() * 10)/4096.0)*60.0;
  }

  public double getAverageRPM() {
    return ((getLeftRPM() + getRightRPM())/2.0);
  } 

  public double getLeftFlywheelPower() {
    return leftFlywheel.get();
  }

  public double getRightFlywheelPower() {
    return rightFlywheel.get();
  }
  
  public void setFlywheelPower(double speed) {
    leftFlywheel.set(speed);
    rightFlywheel.set(speed);
  }

  public boolean flywheelWithinErrorMargin() {
    return (leftFlywheelPID.atSetpoint() && rightFlywheelPID.atSetpoint());
  }

  public double getFlywheelCurrent() {
    return (leftFlywheel.getStatorCurrent() + rightFlywheel.getStatorCurrent())/2.0;
  }

  public void setFlywheelConstantVelocity(double RPM) {
    leftFlywheel.setVoltage(leftFlywheelFF.calculate(RPM) + leftFlywheelPID.calculate(getLeftRPM(), RPM));
    rightFlywheel.set(rightFlywheelFF.calculate(RPM) + rightFlywheelPID.calculate(getRightRPM(), RPM));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    /* SmartDashboard */
    SmartDashboard.putNumber("Average RPM", getAverageRPM());
    SmartDashboard.putNumber("Average Current", getFlywheelCurrent());

    SmartDashboard.putNumber("Left Flywheel RPM", getLeftRPM());
    SmartDashboard.putNumber("Left Flywheel Power", getLeftFlywheelPower());

    SmartDashboard.putNumber("Right Flywheel RPM", getRightRPM());
    SmartDashboard.putNumber("Right Flywheel Power", getRightFlywheelPower());

    /* Override logic */
    if (RobotContainer.getJoy1().getRawButton(2) && overrideTimer.get() >= overrideTime) {
      override = !override;
      overrideTimer.reset();
    }

    /* Code that toggles from manual to auto */
    if (override) { // Auto code
      if (RobotContainer.getJoy1().getRawButton(1)) {
        setFlywheelConstantVelocity(1000.0); // Sets it to 1000 RPM
      } else {
        setFlywheelConstantVelocity(0.0);
        setFlywheelPower(0.0);
      }
    } else if (!override) { // Default manual override
      setFlywheelPower(-1.0*RobotContainer.getJoy1().getY());
    }

    /* Optional NetworkTables code
    leftFlywheelPID.setPID(leftFlywheelInputPIDP.getDouble(Constants.leftFlywheelPIDConsts.pidP), leftFlywheelInputPIDI.getDouble(Constants.leftFlywheelPIDConsts.pidI), leftFlywheelInputPIDD.getDouble(Constants.leftFlywheelPIDConsts.pidD));
    rightFlywheelPID.setPID(rightFlywheelInputPIDP.getDouble(Constants.rightFlywheelPIDConsts.pidP), rightFlywheelInputPIDI.getDouble(Constants.rightFlywheelPIDConsts.pidI), rightFlywheelInputPIDD.getDouble(Constants.rightFlywheelPIDConsts.pidD));
    */
  }
}
