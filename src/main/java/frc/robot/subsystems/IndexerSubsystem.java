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
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
  private final SparkMax m_indexerMotor;

  /** Creates a new IndexerSubsystem. */
  public IndexerSubsystem() {
    m_indexerMotor = new SparkMax(IndexerConstants.kIndexerCanId, MotorType.kBrushless);

    m_indexerMotor.configure(Configs.Indexer.motorConfig,
        ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Indexer/Running", m_indexerMotor.get() != 0);
  }

  /**
   * Runs the indexer motor to release the ball into the shooter.
   * Call this while the button is held.
   */
  public void release() {
    m_indexerMotor.set(IndexerConstants.kIndexerSpeed);
  }

  /** Stops the indexer motor. */
  public void stop() {
    m_indexerMotor.stopMotor();
  }
}
