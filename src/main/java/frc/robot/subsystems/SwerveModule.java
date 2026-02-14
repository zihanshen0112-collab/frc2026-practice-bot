// Copyright (c) FIRST and other WPILib contributors.
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.Constants;

public class SwerveModule {
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;

    private final VoltageOut driveVoltage = new VoltageOut(0);
    private final PositionVoltage steerPosition = new PositionVoltage(0);

    //电机初始化
    public SwerveModule(int driveCANId, int steerCANId, double steerOffsetRads) {
        driveMotor = new TalonFX(driveCANId);
        steerMotor = new TalonFX(steerCANId);

        configDriveMotor();
        configSteerMotor();

        // 初始化转向绝对位置（使用内置编码器+偏移量）
        steerMotor.setPosition(steerOffsetRads / Constants.Module.STEER_RAD_PER_ROT);
    }

    //驱动电机初始化
    private void configDriveMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        //不给电时可以推动
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        //NeutralModeValue.Coast 不给电可以移动
        config.CurrentLimits.SupplyCurrentLimit = Constants.Module.DRIVE_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveMotor.getConfigurator().apply(config);
    }

    //转向电机初始化
    private void configSteerMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        //不给电时可以推动
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimit = Constants.Module.STEER_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // 转向PID
        config.Slot0.kP = Constants.Module.STEER_kP;
        config.Slot0.kI = Constants.Module.STEER_kI;
        config.Slot0.kD = Constants.Module.STEER_kD;

        steerMotor.getConfigurator().apply(config);
    }

    // 设置模块目标状态（速度+角度）
    public void setDesiredState(SwerveModuleState desiredState, double robotTurnRate) {
        // 优化：避免模块旋转超过90度
        desiredState.optimize(getAngle());

        // 驱动：速度(m/s) → 电压(V)
        double driveVoltageValue = desiredState.speedMetersPerSecond * Constants.Drivetrain.DRIVE_VOLTAGE_PER_MPS;
        driveMotor.setControl(driveVoltage.withOutput(driveVoltageValue));

        // 转向：角度(rad) → 电机位置(圈)
        double targetRotations = desiredState.angle.getRadians() / Constants.Module.STEER_RAD_PER_ROT;
        double feedforwardRotations = robotTurnRate * 0.05 * 0.02;
        steerMotor.setControl(
        steerPosition.withPosition(targetRotations + feedforwardRotations)
        );
    }
    
    //重制电机
    public void reset() {
        steerMotor.setPosition(0);
    }

    // 获取当前模块状态（速度+角度） 
    public SwerveModuleState getState() {
        double speedMPS = driveMotor.getVelocity().getValueAsDouble() * Constants.Module.DRIVE_MPS_PER_RPS;
        Rotation2d angle = getAngle();
        return new SwerveModuleState(speedMPS, angle);
    }

    // 获取当前位置（驱动总距离+角度）用于里程计 
    public SwerveModulePosition getPosition() {
        double distanceMeters = driveMotor.getPosition().getValueAsDouble() * Constants.Module.DRIVE_MPS_PER_RPS;
        Rotation2d angle = getAngle();
        return new SwerveModulePosition(distanceMeters, angle);
    }

    // 获取当前转向角度（弧度）
    private Rotation2d getAngle() {
        double rad = steerMotor.getPosition().getValueAsDouble() * Constants.Module.STEER_RAD_PER_ROT;
        return Rotation2d.fromRadians(rad);
    }
}
