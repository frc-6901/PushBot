// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;


public class Drivetrain extends SubsystemBase {
  private WPI_TalonSRX m_leftSRX = new WPI_TalonSRX(DrivetrainConstants.kLeftSRXDrivePort);
  private WPI_VictorSPX m_leftSPX = new WPI_VictorSPX(DrivetrainConstants.kLeftSPXDrivePort);

  private WPI_TalonSRX m_rightSRX = new WPI_TalonSRX(DrivetrainConstants.kRightSRXDrivePort);
  private WPI_VictorSPX m_rightSPX = new WPI_VictorSPX(DrivetrainConstants.kRightSPXDrivePort);

  private TalonSRXSimCollection m_leftTalonSim = new TalonSRXSimCollection(m_leftSRX);
  private TalonSRXSimCollection m_rightTalonSim = new TalonSRXSimCollection(m_rightSRX);

  private DifferentialDrive m_drive = new DifferentialDrive(m_leftSRX, m_rightSRX);



  /** Creates a new Drivetrain. */
  public Drivetrain() {
    m_leftSRX.configFactoryDefault();
    m_leftSPX.configFactoryDefault();
    m_rightSRX.configFactoryDefault();
    m_rightSPX.configFactoryDefault();

    m_leftSPX.follow(m_leftSRX);
    m_rightSPX.follow(m_rightSRX);

    m_rightSRX.setInverted(InvertType.InvertMotorOutput);
    m_rightSPX.setInverted(InvertType.FollowMaster);
  }

  public void drive(double forward, double rotate) {
    m_drive.arcadeDrive(forward, rotate);
  }

  public void setOutput(double leftVolt, double rightVolt) {
    m_leftSRX.setVoltage(leftVolt);
    m_rightSRX.setVoltage(rightVolt);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
