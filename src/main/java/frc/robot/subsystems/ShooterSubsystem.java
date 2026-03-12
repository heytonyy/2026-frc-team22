// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Configs;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final SparkMax m_shooterMotor;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    m_shooterMotor = new SparkMax(ShooterConstants.kShooterCanId, MotorType.kBrushless);

    m_shooterMotor.configure(Configs.Shooter.motorConfig,
        ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/Speed", m_shooterMotor.get());
  }

  /**
   * Sets the shooter motor output.
   *
   * @param speed Motor output from 0.0 (stopped) to 1.0 (full speed).
   */
  public void setSpeed(double speed) {
    m_shooterMotor.set(speed);
  }
}
