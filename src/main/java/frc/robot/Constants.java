package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;

public final class Constants {
    public static final double stickDeadband = 0.1;
    public static final String limelightName = "limelight-main";
    public static final boolean useMegaTag2 = true; //Set to false to use MegaTag1

    public static final class Swerve {
        public static final int pigeonID = 0; 

        public static final COTSTalonFXSwerveConstants chosenModule =  
        COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);

        /* Drivetrain Constants */
        //Will have to wait until we get the robot built
        public static final double trackWidth = Units.inchesToMeters(24.5); 
        public static final double wheelBase = Units.inchesToMeters(24.5); 
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        //Lock the wheels foward and run sysid as normal you would for a tank drive
        public static final double driveKS = 0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 0;
            public static final int canCoderID = 0;
            public static final double angleOffset = Rotation2d.fromDegrees(146.98).getRadians();
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 1;
            public static final double angleOffset = Rotation2d.fromDegrees(125.59).getRadians();
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { 
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 3;
            public static final double angleOffset = Rotation2d.fromDegrees(21.88).getRadians();
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 2;
            public static final double angleOffset = Rotation2d.fromDegrees(89.82).getRadians();
        }

        public static final SwerveDrivetrainConstants driveTrainConstants = new SwerveDrivetrainConstants().withCANbusName("rio").withPigeon2Id(Constants.Swerve.pigeonID);

        private static final Slot0Configs steerGains = new Slot0Configs()
            .withKP(angleKP).withKI(angleKI).withKD(angleKD) 
            .withKS(driveKA).withKV(driveKV).withKA(driveKA);

        private static final Slot0Configs driveGains = new Slot0Configs()
            .withKP(driveKP).withKI(driveKI).withKD(driveKD) 
            .withKS(driveKA).withKV(driveKV).withKA(driveKA);

        private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(driveGearRatio)
            .withSteerMotorGearRatio(angleGearRatio)
            .withWheelRadius(chosenModule.wheelDiameter/2)
            .withSlipCurrent(300)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
            .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
            .withSpeedAt12VoltsMps(3.3)
            .withSteerInertia(0.00001)
            .withDriveInertia(0.001)
            .withSteerFrictionVoltage(0.25)
            .withDriveFrictionVoltage(0.25)
            .withFeedbackSource(SteerFeedbackType.RemoteCANcoder)
            .withCouplingGearRatio((150.0 / 7.0) / 1.0)
            .withSteerMotorInverted(false);

        public static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
            Mod0.angleMotorID, Mod0.driveMotorID, Mod0.canCoderID, Mod0.angleOffset, Units.inchesToMeters(12.25), Units.inchesToMeters(12.25), false);
        public static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
            Mod1.angleMotorID, Mod1.driveMotorID, Mod1.canCoderID, Mod1.angleOffset, Units.inchesToMeters(12.25), Units.inchesToMeters(-12.25), false);
        public static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
            Mod2.angleMotorID, Mod2.driveMotorID, Mod2.canCoderID, Mod2.angleOffset, Units.inchesToMeters(-12.25), Units.inchesToMeters(12.25), false);
        public static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
            Mod3.angleMotorID, Mod3.driveMotorID, Mod3.canCoderID, Mod3.angleOffset, Units.inchesToMeters(-12.25), Units.inchesToMeters(-12.25), false);
    }

    //Will have to wait until we build the robot
    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 2;
        public static final double kPYController = 2;
        public static final double kPThetaController = 2;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
    
}
