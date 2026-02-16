// Copyright (c) FIRST and other WPILib contributors.
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

/**
 * 单个Swerve模块类
 * 管理一个驱动电机和一个转向电机
 * 负责模块的速度控制和角度控制
 */
public class SwerveModule {
    // ========== 电机声明 ==========
    
    /** 驱动电机（Kraken X60）- 控制轮子转动 */
    private final TalonFX driveMotor;
    
    /** 转向电机（Kraken X44）- 控制轮子方向 */
    private final TalonFX steerMotor;

    // ========== 控制模式声明 ==========
    
    /** 电压输出控制模式（用于驱动电机） */
    private final VoltageOut driveVoltage = new VoltageOut(0);
    
    /** 位置控制模式（用于转向电机） */
    private final PositionVoltage steerPosition = new PositionVoltage(0);

    // ========== 模块参数 ==========
    
    /** 转向电机的绝对零点偏移（弧度） */
    private final double steerOffsetRads;
    
    /** 上一次的目标角度（用于优化） */
    private Rotation2d lastAngle = Rotation2d.fromDegrees(0);
    
    /** 当前的目标状态（用于调试） */
    private SwerveModuleState desiredState = new SwerveModuleState();
    
    /** 是否启用调试输出 */
    private boolean debugEnabled = false;

    // ========== 构造函数 ==========

    /**
     * 创建Swerve模块实例
     * @param driveCANId 驱动电机CAN ID
     * @param steerCANId 转向电机CAN ID
     * @param steerOffsetRads 转向绝对零点偏移（弧度）
     */
    public SwerveModule(int driveCANId, int steerCANId, double steerOffsetRads) {
        this.steerOffsetRads = steerOffsetRads;
        
        driveMotor = new TalonFX(driveCANId);
        steerMotor = new TalonFX(steerCANId);

        configDriveMotor();
        configSteerMotor();

        // 关键修复：从绝对编码器读取初始位置并应用偏移
        initializeSteerMotor();
    }

    // ========== 电机配置方法 ==========

    /**
     * 配置驱动电机参数
     * 设置电流限制、空闲模式等
     */
    private void configDriveMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // 空闲模式：Brake（刹车）
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // 电流限制
        config.CurrentLimits.SupplyCurrentLimit = Constants.Module.DRIVE_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        // 应用配置
        driveMotor.getConfigurator().apply(config);
    }

    /**
     * 配置转向电机参数
     * 设置PID、电流限制、反馈传感器等
     */
    private void configSteerMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // 空闲模式：Brake（刹车）
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // 电流限制
        config.CurrentLimits.SupplyCurrentLimit = Constants.Module.STEER_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // 转向PID参数
        config.Slot0.kP = Constants.Module.STEER_kP;
        config.Slot0.kI = Constants.Module.STEER_kI;
        config.Slot0.kD = Constants.Module.STEER_kD;
        
        // 配置反馈传感器（使用内置传感器）
        config.Feedback.FeedbackRemoteSensorID = steerMotor.getDeviceID();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        // 应用配置
        steerMotor.getConfigurator().apply(config);
    }

    /**
     * 初始化转向电机
     * 读取绝对编码器位置并应用偏移量
     */
    private void initializeSteerMotor() {
        // 等待传感器更新
        try {
            Thread.sleep(100);  // 简单延时，确保传感器读数稳定
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        
        // 读取绝对位置（圈数）
        double absPosition = steerMotor.getPosition().getValueAsDouble();
        
        // 应用偏移量（将偏移弧度转换为圈数）
        double relativePosition = absPosition - (steerOffsetRads / (2 * Math.PI));
        
        // 规范化到0-1范围（确保角度在0-360度之间）
        relativePosition = relativePosition % 1.0;
        if (relativePosition < 0) relativePosition += 1.0;
        
        // 设置电机位置
        steerMotor.setPosition(relativePosition);
        
        // 初始化上次角度
        lastAngle = Rotation2d.fromRotations(relativePosition);
        
        if (debugEnabled) {
            SmartDashboard.putNumber("Module " + steerMotor.getDeviceID() + " Init Position", 
                                     relativePosition * 360);
        }
    }

    // ========== 公共控制方法 ==========

    /**
     * 设置模块的目标状态
     * @param desiredState 目标状态（速度+角度）
     * @param robotTurnRate 机器人当前角速度（用于feedforward，当前未使用）
     */
    public void setDesiredState(SwerveModuleState desiredState, double robotTurnRate) {
        // 保存目标状态用于调试
        this.desiredState = desiredState;
        
        // 优化模块状态：选择最短路径旋转（不超过90度）
        desiredState.optimize(getAngle());
        SwerveModuleState optimizedState = desiredState;
        
        // 处理速度接近0的情况：保持上次角度，避免不必要的旋转
        if (Math.abs(optimizedState.speedMetersPerSecond) < 0.01) {
            optimizedState.angle = lastAngle;
        }
        
        // ===== 驱动电机控制 =====
        // 将目标速度（米/秒）转换为电压值
        double driveVoltageValue = optimizedState.speedMetersPerSecond * 
                                   Constants.Drivetrain.DRIVE_VOLTAGE_PER_MPS;
        
        // 限制电压在安全范围内（-12V 到 +12V）
        driveVoltageValue = Math.max(-12, Math.min(12, driveVoltageValue));
        
        // 应用驱动电压
        driveMotor.setControl(driveVoltage.withOutput(driveVoltageValue));

        // ===== 转向电机控制 =====
        // 将目标角度转换为电机位置（圈数）
        double targetRotations = optimizedState.angle.getRotations();
        
        // 使用PID控制转向电机到目标位置
        steerMotor.setControl(
            steerPosition.withPosition(targetRotations)
                        .withSlot(0)  // 使用slot0的PID参数
        );
        
        // 保存本次角度用于下一次优化
        lastAngle = optimizedState.angle;
        
        // 调试输出
        if (debugEnabled) {
            SmartDashboard.putNumber("Module " + steerMotor.getDeviceID() + " Target Speed", 
                                     optimizedState.speedMetersPerSecond);
            SmartDashboard.putNumber("Module " + steerMotor.getDeviceID() + " Target Angle", 
                                     optimizedState.angle.getDegrees());
            SmartDashboard.putNumber("Module " + steerMotor.getDeviceID() + " Current Angle", 
                                     getAngle().getDegrees());
            SmartDashboard.putNumber("Module " + steerMotor.getDeviceID() + " Drive Voltage", 
                                     driveVoltageValue);
        }
    }

    /**
     * 重置模块（调试用）
     * 将转向电机位置重置为0
     */
    public void reset() {
        steerMotor.setPosition(0);
        lastAngle = Rotation2d.fromDegrees(0);
        
        if (debugEnabled) {
            SmartDashboard.putString("Module " + steerMotor.getDeviceID() + " Status", "Reset");
        }
    }

    // ========== 状态获取方法 ==========

    /**
     * 获取模块当前状态
     * @return 当前状态（速度+角度）
     */
    public SwerveModuleState getState() {
        // 计算实际速度（RPS * 转换系数 = 米/秒）
        double speedMPS = driveMotor.getVelocity().getValueAsDouble() * Constants.Module.DRIVE_MPS_PER_RPS;
        
        // 获取当前角度
        Rotation2d angle = getAngle();
        
        return new SwerveModuleState(speedMPS, angle);
    }

    /**
     * 获取模块当前位置（用于里程计）
     * @return 当前位置（距离+角度）
     */
    public SwerveModulePosition getPosition() {
        // 计算总行驶距离（圈数 * 转换系数 = 米）
        double distanceMeters = driveMotor.getPosition().getValueAsDouble() * Constants.Module.DRIVE_MPS_PER_RPS;
        
        // 获取当前角度
        Rotation2d angle = getAngle();
        
        return new SwerveModulePosition(distanceMeters, angle);
    }

    /**
     * 获取模块当前的目标状态（用于调试）
     * @return 目标状态
     */
    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    /**
     * 获取当前转向角度
     * @return 当前角度（Rotation2d）
     */
    private Rotation2d getAngle() {
        // 从电机位置读取圈数，直接转换为Rotation2d
        double rotations = steerMotor.getPosition().getValueAsDouble();
        return Rotation2d.fromRotations(rotations);
    }

    // ========== 调试方法 ==========

    /**
     * 启用/禁用调试输出
     * @param enable true：启用调试输出
     */
    public void enableDebug(boolean enable) {
        this.debugEnabled = enable;
    }

    /**
     * 获取驱动电机（用于高级调试）
     * @return 驱动电机对象
     */
    public TalonFX getDriveMotor() {
        return driveMotor;
    }

    /**
     * 获取转向电机（用于高级调试）
     * @return 转向电机对象
     */
    public TalonFX getSteerMotor() {
        return steerMotor;
    }

    /**
     * 获取模块的偏移量（用于调试）
     * @return 偏移量（度）
     */
    public double getOffsetDegrees() {
        return Math.toDegrees(steerOffsetRads);
    }
}