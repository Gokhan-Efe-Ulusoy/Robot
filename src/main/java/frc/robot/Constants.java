// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class ElevatorConstants {
    public static final int LEFT_MOTOR_ID = 21;
    public static final int RIGHT_MOTOR_ID = 22;

    public static final double GEAR_RATIO = 5.0;

    public static final double MAX_HEIGHT_METERS = 1.65;
    public static final double MIN_HEIGHT_METERS = 0.0;

    public static final double CRUISE_VELOCITY = 3.0; // m/s (placeholder) Converted to rotations/sec in subsystem
    public static final double ACCELERATION = 6.0;    // m/s^2

    public static final double ELV_kP = 8.0;
    public static final double ELV_kD = 0.2;

    public static final double HOME_HEIGHT = 0.0;
    public static final double L1_HEIGHT = 0.35;
    public static final double L2_HEIGHT = 0.75;
    public static final double L3_HEIGHT = 1.15;
    public static final double L4_HEIGHT = 1.55;
    public static final double NET_HEIGHT = 1.65;
    public static final double PROCESSOR_HEIGHT = 0.25;
  }
  
  public static class GripperArmConstants {

    public static final int ARM_MOTOR_ID = 31;
    public static final int GRIPPER_MOTOR_ID = 32;

    public static final int BEAM_BREAK_DIO = 0;
  
    public static final double ARM_CRUISE_VELOCITY = 3.0;
    public static final double ARM_ACCELERATION = 6.0;
  
    public static final double ARM_kP = 6.0;
    public static final double ARM_kD = 0.15;
  
    public static final double MIN_ANGLE_DEG = -30.0;
    public static final double MAX_ANGLE_DEG = 110.0;
  
    public static final double STOW_ANGLE = -20.0;
    public static final double INTAKE_ANGLE = 10.0;
    public static final double SCORE_L1_ANGLE = 25.0;
    public static final double SCORE_L2_ANGLE = 45.0;
    public static final double SCORE_L3_ANGLE = 65.0;
    public static final double SCORE_L4_ANGLE = 85.0;
    public static final double SCORE_NET_ANGLE = 95.0;
  
    public static final double INTAKE_SPEED = 0.6;
    public static final double HOLD_SPEED = 0.15;
    public static final double OUTTAKE_SPEED = -0.6;
  }  
  
  public static class CoralCradleConstants {

    public static final int SERVO_CHANNEL = 1;

    public static final double OPEN_POSITION = 0.2;
    public static final double CLOSED_POSITION = 0.8;
  }
  
  public static class StraightenatorConstants {
    
    public static final int LEFT_MOTOR_ID = 41;
    public static final int RIGHT_MOTOR_ID = 42;
    
    public static final double IN_SPEED = 0.6;
    public static final double OUT_SPEED = -0.5;
    
    public static final double JAM_CURRENT = 35.0;
    public static final double UNJAM_SPEED = -0.4;
    public static final double UNJAM_TIME = 0.25; // seconds
    
  }
  
  public static class IntakeConstants {

    public static final int MOTOR_ID = 51;

    public static final double INTAKE_SPEED = 0.7;
    public static final double EJECT_SPEED = -0.7;
  }
  
  public static class ClimberConstants {

    public static final int MOTOR_ID = 61;

    public static final int TOP_LIMIT_DIO = 2;
    public static final int BOTTOM_LIMIT_DIO = 3; 

    public static final double UP_SPEED = 0.8;
    public static final double DOWN_SPEED = -0.6;

    public static final double SUPPLY_CURRENT_LIMIT = 40.0;
    public static final double SUPPLY_CURRENT_THRESHOLD = 60.0;
    public static final double SUPPLY_TIME_THRESHOLD = 0.2;

    public static final double STALL_CURRENT = 65.0;
    
    public static final double BOTTOM_SOFT_LIMIT_ROTATIONS = 0.0;
    public static final double TOP_SOFT_LIMIT_ROTATIONS = 120.0;
  }
  
  public static class DriveConstants {

    public static final int PIGEON_ID = 50;

    public static final double TRACK_WIDTH = 0.55;
    public static final double WHEEL_BASE = 0.55;

    public static final double MAX_SPEED_MPS = 4.5;

    public static final double NOMINAL_VOLTAGE = 12.0;

    public static final double FL_OFFSET = 0.123;
    public static final double FR_OFFSET = 0.456;
    public static final double BL_OFFSET = 0.789;
    public static final double BR_OFFSET = 0.012;
    public static final double DRIVE_SUPPLY_CURRENT_LIMIT = 50.0;
    public static final double DRIVE_SUPPLY_CURRENT_THRESHOLD = 70.0;
    public static final double DRIVE_SUPPLY_TIME_THRESHOLD = 0.2;

    public static final double STEER_SUPPLY_CURRENT_LIMIT = 20.0;
    public static final double STEER_SUPPLY_CURRENT_THRESHOLD = 30.0;
    public static final double STEER_SUPPLY_TIME_THRESHOLD = 0.2;
   
  }
}
