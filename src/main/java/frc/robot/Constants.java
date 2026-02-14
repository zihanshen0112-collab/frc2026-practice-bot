// Copyright (c) FIRST and other WPILib contributors.
package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * 机器人全局常量配置
 * 所有需要根据实际机器人调整的参数都放在这里
 */
public final class Constants {
    
    // ========== CAN 总线 ID（根据实际布线修改！）==========
    public static final class CAN {
        // 驱动电机 (Kraken X60) - 控制轮子转动
        public static final int FRONT_LEFT_DRIVE  = 5;
        public static final int FRONT_RIGHT_DRIVE = 6;
        public static final int BACK_LEFT_DRIVE   = 7;
        public static final int BACK_RIGHT_DRIVE  = 8;

        // 转向电机 (Kraken X44) - 控制轮子方向
        public static final int FRONT_LEFT_STEER   = 1;
        public static final int FRONT_RIGHT_STEER  = 2;
        public static final int BACK_LEFT_STEER    = 3;
        public static final int BACK_RIGHT_STEER   = 4;

        // 陀螺仪（如果有，暂时预留）
        public static final int PIGEON_ID = 0;
    }

    // ========== 驱动模块物理参数 ==========
    public static final class Module {
        //转向电机减速比 (电机转数 : 轮子转数) - Kraken X44 
        public static final double STEER_GEAR_RATIO = 12.8; //待修改
        
        //驱动电机减速比 (电机转数 : 轮子转数) - Kraken X60 
        public static final double DRIVE_GEAR_RATIO = 8.14; //待修改

        //轮子直径（米）- 4英寸 = 0.1016米 
        public static final double WHEEL_DIAMETER_METERS = 0.1016; //待修改
        
        //轮子周长（米）
        public static final double WHEEL_CIRCUMFERENCE_METERS = Math.PI * WHEEL_DIAMETER_METERS;

        //驱动电机速度转换系数：RPS（圈/秒） → 米/秒 
        public static final double DRIVE_MPS_PER_RPS = WHEEL_CIRCUMFERENCE_METERS / DRIVE_GEAR_RATIO;

        // 转向电机位置转换系数：圈 → 弧度 
        public static final double STEER_RAD_PER_ROT = 2 * Math.PI / STEER_GEAR_RATIO;

        // ===== 转向电机 PID 值（需实测调整）=====
        public static final double STEER_kP = 0.1;
        public static final double STEER_kI = 0.0;
        public static final double STEER_kD = 0.0;

        // ===== 电机电流限制 =====
        public static final int DRIVE_CURRENT_LIMIT = 40; // Kraken X60
        public static final int STEER_CURRENT_LIMIT = 30; // Kraken X44

        // ===== 模块绝对零点偏移（弧度）=====
        //轮子朝向正前方时读数
        public static final double FRONT_LEFT_OFFSET_RADS  = 0.0;
        public static final double FRONT_RIGHT_OFFSET_RADS = 0.0;
        public static final double BACK_LEFT_OFFSET_RADS   = 0.0;
        public static final double BACK_RIGHT_OFFSET_RADS  = 0.0;
    }

    // ========== 机器人尺寸（轮子位置，单位：米）==========
    public static final class Drivetrain {
        //车宽一半（左右轮距的一半） 
        public static final double TRACK_WIDTH_HALF = 0.3; //待修改
        
        //车长一半（前后轮距的一半）
        public static final double WHEEL_BASE_HALF = 0.3;  //待修改

        //四个模块相对于机器人中心的坐标
        public static final Translation2d FRONT_LEFT_LOCATION = 
            new Translation2d(WHEEL_BASE_HALF, TRACK_WIDTH_HALF);
        public static final Translation2d FRONT_RIGHT_LOCATION = 
            new Translation2d(WHEEL_BASE_HALF, -TRACK_WIDTH_HALF);
        public static final Translation2d BACK_LEFT_LOCATION = 
            new Translation2d(-WHEEL_BASE_HALF, TRACK_WIDTH_HALF);
        public static final Translation2d BACK_RIGHT_LOCATION = 
            new Translation2d(-WHEEL_BASE_HALF, -TRACK_WIDTH_HALF);

        // 运动学对象 
        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            FRONT_LEFT_LOCATION,
            FRONT_RIGHT_LOCATION,
            BACK_LEFT_LOCATION,
            BACK_RIGHT_LOCATION
        );

        //最大前进速度（米/秒）- 理论值，实际受电压限制 
        public static final double MAX_FORWARD_SPEED_MPS = 4.5;
        
        //最大侧移速度（米/秒） 
        public static final double MAX_STRAFE_SPEED_MPS = 4.5;
        
        //最大旋转速度（弧度/秒）- 每秒一圈 ≈ 6.28 rad/s 
        public static final double MAX_ROTATE_SPEED_RAD_PER_SEC = 2.0 * Math.PI;

        //驱动电机电压与速度关系：12V 对应最大速度 
        public static final double DRIVE_VOLTAGE_PER_MPS = 12.0 / MAX_FORWARD_SPEED_MPS;

        //速度死区比例（摇杆10%以内忽略）
        public static final double SPEED_DEADBAND_RATIO = 0.1;
    }

    // ========== 手柄操作相关 ==========
    public static final class OI {
        //主驾驶手柄USB端口 
        public static final int DRIVER_CONTROLLER_PORT = 0;
        
        //摇杆死区 
        public static final double JOYSTICK_DEADBAND = 0.1;
        
        //默认速度倍率（安全起见限制为40%） 
        public static final double DEFAULT_SPEED_MULTIPLIER = 0.4;
    }

    // ========== 比赛场地信息（留空，暂不使用）==========
    public static final class Field {
        //场地尺寸等，暂时留空
    }

    public static final class State {
        //是否启用陀螺仪
        public static final boolean FIELD_RELATIVE = false;
    }
}