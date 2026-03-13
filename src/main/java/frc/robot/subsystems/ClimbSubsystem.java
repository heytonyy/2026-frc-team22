// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Configs;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
  private final SparkMax m_climbLeader;

  private final RelativeEncoder m_encoder;

  public ClimbSubsystem() {
    m_climbLeader = new SparkMax(ClimbConstants.kClimbLeaderCanId, MotorType.kBrushless);

    // Apply configurations — reset to a known state, then persist to survive power cycles.
    m_climbLeader.configure(Configs.Climb.leaderConfig,
        ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    m_encoder = m_climbLeader.getEncoder();
    m_encoder.setPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climb/Position", m_encoder.getPosition());
  }

  /**
   * Runs both motors to lift the robot at the configured climb speed.
   * The follower motor mirrors the leader automatically.
   */
  public void lift() {
    m_climbLeader.set(ClimbConstants.kClimbSpeed);
  }

  /**
   * Runs both motors to lower the robot at the configured climb speed.
   */
  public void lower() {
    m_climbLeader.set(-ClimbConstants.kClimbSpeed);
  }

  /**
   * Stops both climb motors.
   * The kBrake idle mode will hold the robot's position after stopping.
   */
  public void stop() {
    m_climbLeader.stopMotor();
  }

  /**
   * Returns the encoder position of the lead climb motor in rotations.
   * Positive values mean the robot has been lifted.
   *
   * @return The position in rotations.
   */
  public double getPosition() {
    return m_encoder.getPosition();
  }
}
