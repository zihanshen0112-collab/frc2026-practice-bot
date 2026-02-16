// Copyright (c) FIRST and other WPILib contributors.
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

/**
 * Swerve底盘子系统
 * 管理四个SwerveModule模块，处理驱动控制和里程计计算
 * 负责机器人的运动控制、位姿估计和陀螺仪管理
 */
public class SwerveDrive extends SubsystemBase {
    
    // ========== 成员变量声明 ==========
    
    /** Swerve模块数组：0-前左，1-前右，2-后左，3-后右 */
    private final SwerveModule[] m_modules = new SwerveModule[4];
    
    /** Pigeon2陀螺仪，用于获取机器人朝向和角速度 */
    private final Pigeon2 m_gyro = new Pigeon2(Constants.CAN.PIGEON_ID);
    
    /** 陀螺仪当前角度（Rotation2d格式） */
    private Rotation2d m_gyroAngle = new Rotation2d();
    
    /** 陀螺仪角度偏移量（用于重置零位） */
    private double m_gyroOffset = 0.0;
    
    /** 上一次循环的角速度，用于调试和feedforward */
    //private double m_prevTurnRate = 0.0;

    /** 里程计：融合编码器和陀螺仪数据计算机器人位姿 */
    private final SwerveDrivePoseEstimator m_poseEstimator;

    // ========== 构造函数 ==========

    /**
     * 创建Swerve底盘实例
     * 初始化四个模块、陀螺仪和里程计
     */
    public SwerveDrive() {
        // 初始化四个模块（顺序：前左、前右、后左、后右）
        // 前左模块
        m_modules[0] = new SwerveModule(
            Constants.CAN.FRONT_LEFT_DRIVE,
            Constants.CAN.FRONT_LEFT_STEER,
            Constants.Module.FRONT_LEFT_OFFSET_RADS
        );
        
        // 前右模块
        m_modules[1] = new SwerveModule(
            Constants.CAN.FRONT_RIGHT_DRIVE,
            Constants.CAN.FRONT_RIGHT_STEER,
            Constants.Module.FRONT_RIGHT_OFFSET_RADS
        );
        
        // 后左模块
        m_modules[2] = new SwerveModule(
            Constants.CAN.BACK_LEFT_DRIVE,
            Constants.CAN.BACK_LEFT_STEER,
            Constants.Module.BACK_LEFT_OFFSET_RADS
        );
        
        // 后右模块
        m_modules[3] = new SwerveModule(
            Constants.CAN.BACK_RIGHT_DRIVE,
            Constants.CAN.BACK_RIGHT_STEER,
            Constants.Module.BACK_RIGHT_OFFSET_RADS
        );

        // 初始化陀螺仪，设置初始偏航角为0
        m_gyro.setYaw(0);

        // 初始化里程计
        m_poseEstimator = new SwerveDrivePoseEstimator(
            Constants.Drivetrain.KINEMATICS,    // 运动学模型
            m_gyroAngle,                         // 初始角度
            getModulePositions(),                 // 初始模块位置
            new Pose2d()                          // 初始位姿（0,0,0）
        );

        // 在Dashboard上显示调试信息
        SmartDashboard.putNumber("Gyro/Initialized", 1);
    }

    // ========== 周期性方法 ==========

    /**
     * 周期性执行（每20ms调用一次）
     * 更新陀螺仪角度和里程计
     * 向Dashboard推送调试数据
     */
    @Override
    public void periodic() {
        // 从Pigeon2读取真实角度（单位：度）
        double yawDegrees = m_gyro.getYaw().getValueAsDouble();
        
        // 应用偏移量并转换为Rotation2d
        double adjustedYaw = yawDegrees - m_gyroOffset;
        m_gyroAngle = Rotation2d.fromDegrees(adjustedYaw);
        
        // 更新里程计（融合陀螺仪和编码器数据）
        m_poseEstimator.update(m_gyroAngle, getModulePositions());
        
        // 更新上一次角速度
        //m_prevTurnRate = getTurnRate();
        
        // 向Dashboard推送调试数据
        updateSmartDashboard();
    }

    // ========== 私有辅助方法 ==========

    /**
     * 获取所有模块的当前位置（用于里程计）
     * @return SwerveModulePosition数组，包含每个模块的距离和角度
     */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = m_modules[i].getPosition();
        }
        return positions;
    }

    /**
     * 获取所有模块的当前状态（用于运动学计算）
     * @return SwerveModuleState数组，包含每个模块的速度和角度
     */
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = m_modules[i].getState();
        }
        return states;
    }

    /**
     * 处理输入值的死区
     * @param value 原始输入值
     * @param deadband 死区阈值
     * @return 处理后的值（小于死区返回0）
     */
    private double applyDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0.0;
        }
        return value;
    }

    /**
     * 更新SmartDashboard调试信息
     */
    private void updateSmartDashboard() {
        // 陀螺仪信息
        SmartDashboard.putNumber("Gyro/Angle", m_gyroAngle.getDegrees());
        SmartDashboard.putNumber("Gyro/Turn Rate", getTurnRate());
        
        // 位姿信息
        Pose2d pose = getPose();
        SmartDashboard.putNumber("Pose/X", pose.getX());
        SmartDashboard.putNumber("Pose/Y", pose.getY());
        SmartDashboard.putNumber("Pose/Rotation", pose.getRotation().getDegrees());
        
        // 底盘速度信息
        ChassisSpeeds speeds = getChassisSpeeds();
        SmartDashboard.putNumber("Chassis/vx", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Chassis/vy", speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Chassis/omega", speeds.omegaRadiansPerSecond);
        
        // 各模块状态
        for (int i = 0; i < 4; i++) {
            SwerveModuleState state = m_modules[i].getState();
            SmartDashboard.putNumber("Module" + i + "/Speed", state.speedMetersPerSecond);
            SmartDashboard.putNumber("Module" + i + "/Angle", state.angle.getDegrees());
        }
    }

    // ========== 公共控制方法 ==========

    /**
     * 驱动底盘的主方法
     * @param xSpeed 前进速度 (+ = 前进，- = 后退)，单位：米/秒
     * @param ySpeed 侧移速度 (+ = 左移，- = 右移)，单位：米/秒
     * @param rotSpeed 旋转速度 (+ = 逆时针，- = 顺时针)，单位：弧度/秒
     * @param fieldRelative 是否场向控制（true：相对于场地，false：相对于机器人）
     */
    public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative) {
        // 应用死区处理
        xSpeed = applyDeadband(xSpeed, Constants.Drivetrain.SPEED_DEADBAND_RATIO * Constants.Drivetrain.MAX_FORWARD_SPEED_MPS);
        ySpeed = applyDeadband(ySpeed, Constants.Drivetrain.SPEED_DEADBAND_RATIO * Constants.Drivetrain.MAX_FORWARD_SPEED_MPS);
        rotSpeed = applyDeadband(rotSpeed, Constants.Drivetrain.SPEED_DEADBAND_RATIO * Constants.Drivetrain.MAX_ROTATE_SPEED_RAD_PER_SEC);
        
        // 限制最大速度
        xSpeed = Math.max(-Constants.Drivetrain.MAX_FORWARD_SPEED_MPS, 
                 Math.min(Constants.Drivetrain.MAX_FORWARD_SPEED_MPS, xSpeed));
        ySpeed = Math.max(-Constants.Drivetrain.MAX_STRAFE_SPEED_MPS, 
                 Math.min(Constants.Drivetrain.MAX_STRAFE_SPEED_MPS, ySpeed));
        rotSpeed = Math.max(-Constants.Drivetrain.MAX_ROTATE_SPEED_RAD_PER_SEC, 
                  Math.min(Constants.Drivetrain.MAX_ROTATE_SPEED_RAD_PER_SEC, rotSpeed));
        
        // 计算机器人坐标系下的速度
        ChassisSpeeds chassisSpeeds;
        
        if (fieldRelative) {
            // 场向控制：将场地坐标系的速度转换为机器人坐标系
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, rotSpeed, m_gyroAngle
            );
        } else {
            // 机器人向控制：直接使用输入速度
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
        }

        // 计算每个模块的目标状态（速度+角度）
        SwerveModuleState[] targetStates = Constants.Drivetrain.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        
        // 规范化模块速度（防止超过物理极限）
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.Drivetrain.MAX_FORWARD_SPEED_MPS);
        
        // 获取当前角速度用于转向feedforward
        double currentTurnRate = getTurnRate();
        
        // 将目标状态发送给每个模块
        setModuleStates(targetStates, currentTurnRate);
    }

    /**
     * 设置所有模块的目标状态
     * @param targetStates 目标状态数组
     * @param currentTurnRate 当前角速度（用于feedforward）
     */
    public void setModuleStates(SwerveModuleState[] targetStates, double currentTurnRate) {
        for (int i = 0; i < 4; i++) {
            m_modules[i].setDesiredState(targetStates[i], currentTurnRate);
        }
    }

    /**
     * 停止所有运动
     * 将速度设为0，模块保持当前角度
     */
    public void stop() {
        drive(0, 0, 0, Constants.State.FIELD_RELATIVE);
    }

    // ========== 陀螺仪相关方法 ==========

    /**
     * 设置陀螺仪角度
     * @param angle 目标角度
     */
    public void setGyroAngle(Rotation2d angle) {
        m_gyroAngle = angle;
    }

    /**
     * 重置陀螺仪零位（当前方向设为前方）
     * 用于在比赛开始时对齐场地坐标系
     */
    public void zeroGyro() {
        double currentYaw = m_gyro.getYaw().getValueAsDouble();
        m_gyroOffset = currentYaw;  // 设置偏移量使当前角度变为0
        m_gyroAngle = Rotation2d.fromDegrees(0);
        
        SmartDashboard.putNumber("Gyro/Zeroed", currentYaw);
    }

    /**
     * 获取当前旋转角速度
     * @return 角速度（弧度/秒），正值为逆时针
     */
    public double getTurnRate() {
        // Pigeon2的getAngularVelocityZWorld()返回度/秒，转换为弧度/秒
        return Math.toRadians(m_gyro.getAngularVelocityZWorld().getValueAsDouble());
    }

    // ========== 位姿相关方法 ==========

    /**
     * 获取当前机器人位姿
     * @return 机器人在场地上的位姿（x, y, rotation）
     */
    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    /**
     * 重置里程计到指定位姿
     * @param pose 目标位姿
     */
    public void resetPose(Pose2d pose) {
        m_poseEstimator.resetPosition(m_gyroAngle, getModulePositions(), pose);
    }

    // ========== 状态获取方法 ==========

    /**
     * 获取当前底盘速度
     * @return 底盘速度（vx, vy, omega）
     */
    public ChassisSpeeds getChassisSpeeds() {
        return Constants.Drivetrain.KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    /**
     * 获取当前陀螺仪角度
     * @return 机器人朝向（Rotation2d）
     */
    public Rotation2d getGyroAngle() {
        return m_gyroAngle;
    }

    /**
     * 获取所有模块的引用（用于调试）
     * @return 模块数组
     */
    public SwerveModule[] getModules() {
        return m_modules;
    }

    // ========== 调试方法 ==========

    /**
     * 重置所有模块（调试用）
     * 将转向电机位置重置为0
     */
    public void resetModules() {
        for (SwerveModule module : m_modules) {
            module.reset();
        }
        SmartDashboard.putString("Status", "Modules Reset");
    }

    /**
     * 启用/禁用模块的调试输出
     * @param enable true：启用调试输出
     */
    public void enableModuleDebug(boolean enable) {
        for (int i = 0; i < m_modules.length; i++) {
            m_modules[i].enableDebug(enable);
        }
    }

    /**
     * 获取模块的目标状态（用于路径规划）
     * @param chassisSpeeds 期望的底盘速度
     * @return 对应的模块状态数组
     */
    public SwerveModuleState[] getTargetStatesFromChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] targetStates = Constants.Drivetrain.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.Drivetrain.MAX_FORWARD_SPEED_MPS);
        return targetStates;
    }

    /**
     * 检查底盘是否稳定（用于自动程序）
     * @return true：所有模块角度误差小于阈值
     */
    public boolean isStable() {
        final double ANGLE_TOLERANCE = 0.05; // 弧度
        
        for (SwerveModule module : m_modules) {
            SwerveModuleState currentState = module.getState();
            SwerveModuleState desiredState = module.getDesiredState();
            
            double angleError = Math.abs(currentState.angle.getRadians() - 
                                        desiredState.angle.getRadians());
            if (angleError > ANGLE_TOLERANCE) {
                return false;
            }
        }
        return true;
    }
}