/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  private Encoder encoderX = new Encoder(RobotMap.encoderX[0], RobotMap.encoderX[1]);
  private Encoder encoderY = new Encoder(RobotMap.encoderY[0], RobotMap.encoderY[1]);

  private Victor frontLeft = new Victor(RobotMap.frontLeftMotor);
  private Victor frontRight = new Victor(RobotMap.frontRightMotor);
  private Victor backLeft = new Victor(RobotMap.backLeftMotor);
  private Victor backRight = new Victor(RobotMap.backRightMotor);

  private MecanumDrive mecanumDrive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);

  private ADXRS450_Gyro gyro = new ADXRS450_Gyro();
 
  // private PIDController pidController = new PIDController();

  public DriveTrain() {
    // 6pi / 128 / 12
    double distancePerPulse = 6.0 * Math.PI / 128 / 12;

    // Set distance per pulse
    encoderX.setDistancePerPulse(distancePerPulse);
    encoderY.setDistancePerPulse(distancePerPulse);

    // Set max period
    encoderX.setMaxPeriod(.1);
    encoderY.setMaxPeriod(.1);

    // We don't need reverse direction
    encoderX.setReverseDirection(false);
    encoderY.setReverseDirection(false);

    // Reset the encoders to their initial state
    encoderX.reset();
    encoderY.reset();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void pidDrive(double targetSpeedX, double targetSpeedY) {
    double pidOutputX = 0.0;
    double pidOutputY = 0.0;


    // mecanumDrive.driveCartesian(pidOutputY, pidOutputX, 0);
  }

  private double getVelocityX() {
    return encoderX.getRate();
  }

  private double getVelocityY() {
    return encoderY.getRate();
  }
}
