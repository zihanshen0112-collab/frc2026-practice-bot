package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;


//Swerve底盘
//管理SwerveModule，处理驱动控制和里程计
public class SwerveDrive extends SubsystemBase {
    
    //SwerveModule
    private final SwerveModule[] m_modules = new SwerveModule[4];
    
    // 陀螺仪（未知型号，暂不实现）
    private Rotation2d m_gyroAngle = new Rotation2d();
    //private double m_gyroOffset = 0.0;

    // 里程计
    private final SwerveDrivePoseEstimator m_poseEstimator;

    //创建底盘
    public SwerveDrive() {
        // 初始化四个模块
        //前左
        m_modules[0] = new SwerveModule(
            Constants.CAN.FRONT_LEFT_DRIVE,
            Constants.CAN.FRONT_LEFT_STEER,
            Constants.Module.FRONT_LEFT_OFFSET_RADS
        );
        //前右
        m_modules[1] = new SwerveModule(
            Constants.CAN.FRONT_RIGHT_DRIVE,
            Constants.CAN.FRONT_RIGHT_STEER,
            Constants.Module.FRONT_RIGHT_OFFSET_RADS
        );
        //后左
        m_modules[2] = new SwerveModule(
            Constants.CAN.BACK_LEFT_DRIVE,
            Constants.CAN.BACK_LEFT_STEER,
            Constants.Module.BACK_LEFT_OFFSET_RADS
        );
        //后右
        m_modules[3] = new SwerveModule(
            Constants.CAN.BACK_RIGHT_DRIVE,
            Constants.CAN.BACK_RIGHT_STEER,
            Constants.Module.BACK_RIGHT_OFFSET_RADS
        );

        // 初始化里程计
        m_poseEstimator = new SwerveDrivePoseEstimator(
            Constants.Drivetrain.KINEMATICS,
            m_gyroAngle,
            getModulePositions(),
            new Pose2d()
        );
    }

    @Override
    public void periodic() {
        // 每20ms更新一次里程计
        m_poseEstimator.update(m_gyroAngle, getModulePositions());
    }

    //获取所有模块的当前位置
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = m_modules[i].getPosition();
        }
        return positions;
    }

    //获取所有module的当前状态
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = m_modules[i].getState();
        }
        return states;
    }
    
    /**
     * 驱动底盘method
     * @param xSpeed 前进速度 (+ = 前进)
     * @param ySpeed 侧移速度 (+ = 左移)
     * @param rotSpeed 旋转速度 (+ = 逆时针)
     * @param fieldRelative 是否场向控制（是否有陀螺仪）
     */
    public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative) {
        ChassisSpeeds chassisSpeeds;
        
        if (fieldRelative) {
            // 有陀螺仪
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, rotSpeed, m_gyroAngle
            );
        } else {
            // 没有陀螺仪
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
        }

        // 计算每个模块的目标状态
        SwerveModuleState[] targetStates = Constants.Drivetrain.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        
        // 应用目标状态
        setModuleStates(targetStates);
    }

    //设置module目标状态
    public void setModuleStates(SwerveModuleState[] targetStates) {
        // 速度限制（防止超过物理极限）
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.Drivetrain.MAX_FORWARD_SPEED_MPS);
        
        // 给每个模块发送目标
        for (int i = 0; i < 4; i++) {
            m_modules[i].setDesiredState(targetStates[i]);
        }
    }

    //停止
    public void stop() {
        drive(0, 0, 0, Constants.State.FIELD_RELATIVE);
    }

    //设置陀螺仪角度
    public void setGyroAngle(Rotation2d angle) {
        m_gyroAngle = angle;
    }

    //重置陀螺仪-->0（当前方向设为前方）
    public void zeroGyro() {
        m_gyroAngle = new Rotation2d();
    }

    //获取当前机器人位姿（向量）
    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    //重置里程计到指定位姿
    public void resetPose(Pose2d pose) {
        m_poseEstimator.resetPosition(m_gyroAngle, getModulePositions(), pose);
    }

    //获取当前底盘速度
    public ChassisSpeeds getChassisSpeeds() {
        return Constants.Drivetrain.KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    //重置所有模块（调试用）
    public void resetModules() {
        for (SwerveModule module : m_modules) {
            module.reset();
        }
    }
}