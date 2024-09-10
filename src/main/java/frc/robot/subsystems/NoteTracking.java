// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NoteAlignConstants;
import frc.utils.LimelightHelpers;

public class NoteTracking extends SubsystemBase {
  /** Creates a new NoteTracking. */
  private String m_limelightName;
  private double tx;
  private boolean tv;
  private PIDController m_pid;
  public boolean active;
  public double correction;

  public NoteTracking() {
    m_limelightName = "limelight";

    m_pid = new PIDController(NoteAlignConstants.kP, NoteAlignConstants.kI, NoteAlignConstants.kD);
    m_pid.setTolerance(NoteAlignConstants.kTolerance);
    m_pid.setSetpoint(0);

    active = false;
  }

  @Override
  public void periodic() {
    tv = LimelightHelpers.getTV(m_limelightName);

    if (active && tv) {
      tx = LimelightHelpers.getTX(m_limelightName);

      correction = m_pid.calculate(tx);
    }
    if (!active) {
      correction = 0;
    }
   }

   public void set(boolean state) {
    active = state;
   }

   public boolean get() {
    return active;
   }

   public Command setCmd(boolean state) {
    return runOnce(() -> set(state));
   }

   public void toggle() {
    active = !active;
   }

   public Command toggleCmd() {
    return runOnce(() -> toggle());
   }
}
