package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Util.SwerveVoltageRequest;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

import static edu.wpi.first.units.Units.Volts;

public class Swerve extends SwerveDrivetrain implements Subsystem {
    public SwerveModule[] mSwerveMods;
    public Field2d m_Field = new Field2d();//Creates a field object to visualize the robot pose in smartdashboard. 
    public Pigeon2 gyro;

    private final SwerveVoltageRequest m_driveVoltageRequest = new SwerveVoltageRequest();

    private final SysIdRoutine m_SysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(null, Volts.of(4), null, (state)->SignalLogger.writeString("state", state.toString())),
        new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> setControl(m_driveVoltageRequest.withVoltage(volts.in(Volts))), null, this)
    );
    
    public Swerve(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants module1Constants, SwerveModuleConstants module2Constants, SwerveModuleConstants module3Constants, SwerveModuleConstants module4Constants) {
        super(driveTrainConstants, module1Constants, module2Constants, module3Constants, module4Constants);

        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.FrontLeft),
            new SwerveModule(1, Constants.Swerve.FrontRight),
            new SwerveModule(2, Constants.Swerve.BackLeft),
            new SwerveModule(3, Constants.Swerve.BackRight)
        };

        SignalLogger.setPath("/media/sda1/ctre-logs/");
        SignalLogger.start();
    }

    public Command runDriveQuasiTest(SysIdRoutine.Direction direction)
    {
        return m_SysIdRoutine.quasistatic(direction);
    }

    public Command runDriveDynamTest(SysIdRoutine.Direction direction) {
        return m_SysIdRoutine.dynamic(direction);
    }
}