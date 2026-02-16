// Copyright (c) FIRST and other WPILib contributors.
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.subsystems.SwerveDrive;

public class RobotContainer {
    
    // 速度限制（安全起见，默认40%）
    private double m_speedMultiplier = Constants.OI.DEFAULT_SPEED_MULTIPLIER;
    
    // 最大速度（从Constants获取）
    private final double m_maxSpeed = Constants.Drivetrain.MAX_FORWARD_SPEED_MPS;
    private final double m_maxAngularRate = Constants.Drivetrain.MAX_ROTATE_SPEED_RAD_PER_SEC;

    // 手柄
    private final CommandXboxController m_driverController = 
        new CommandXboxController(Constants.OI.DRIVER_CONTROLLER_PORT);

    // 底盘子系统
    public final SwerveDrive m_drivetrain = new SwerveDrive();

    // 自动命令选择器
    private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

    public RobotContainer() {
        configureBindings();
        
        // 添加默认自动命令（空命令）
        m_autoChooser.setDefaultOption("Do Nothing", Commands.none());
        SmartDashboard.putData("Auto Mode", m_autoChooser);

        //初始化陀螺仪
        m_drivetrain.zeroGyro();
    }

    private void configureBindings() {
        
        // 遥感驾驶
        m_drivetrain.setDefaultCommand(
        Commands.run(() -> {
            // 获取摇杆输入
            double forward = -m_driverController.getLeftY();
            double strafe = -m_driverController.getLeftX();
            double rotation = -m_driverController.getRightX();
            
            // 应用死区（虽然drive方法中也有，但这里先处理一下）
            forward = Math.abs(forward) > Constants.OI.JOYSTICK_DEADBAND ? forward : 0;
            strafe = Math.abs(strafe) > Constants.OI.JOYSTICK_DEADBAND ? strafe : 0;
            rotation = Math.abs(rotation) > Constants.OI.JOYSTICK_DEADBAND ? rotation : 0;
            
            // 驱动底盘
            m_drivetrain.drive(
                forward * Constants.Drivetrain.MAX_FORWARD_SPEED_MPS * m_speedMultiplier,
                strafe * Constants.Drivetrain.MAX_STRAFE_SPEED_MPS * m_speedMultiplier,
                rotation * Constants.Drivetrain.MAX_ROTATE_SPEED_RAD_PER_SEC,
                true  
            );
        }, m_drivetrain)
    );

        //机器人禁用时停止所有电机
        RobotModeTriggers.disabled().whileTrue(
            Commands.runOnce(() -> m_drivetrain.stop(), m_drivetrain).ignoringDisable(true)
        );

        // ===== 按钮功能 =====
        
        // A键：重置陀螺仪（当前方向设为前方）
        m_driverController.a().onTrue(
            Commands.runOnce(() -> m_drivetrain.zeroGyro(), m_drivetrain)
        );

        // B键：速度倍率切换（正常 vs 慢速）
        m_driverController.b().toggleOnTrue(
            Commands.runOnce(() -> {
                if (m_speedMultiplier == Constants.OI.DEFAULT_SPEED_MULTIPLIER) {
                    m_speedMultiplier = 1.0;  // 全速
                } else {
                    m_speedMultiplier = Constants.OI.DEFAULT_SPEED_MULTIPLIER;  // 慢速
                }
            })
        );

            // X键：原地旋转演示 - 🔴 修改
        m_driverController.x().whileTrue(
            Commands.run(() -> 
                m_drivetrain.drive(0, 0, 0.5 * m_maxAngularRate, true),
                m_drivetrain
            )
        );

        // Y键：原地旋转演示 - 🔴 修改
        m_driverController.y().whileTrue(
            Commands.run(() -> 
                m_drivetrain.drive(0, 0, -0.5 * m_maxAngularRate, true),
                m_drivetrain
            )
        );

        // 左肩键：重置所有模块（调试用）
        m_driverController.leftBumper().onTrue(
            Commands.runOnce(() -> m_drivetrain.resetModules(), m_drivetrain)
        );

         // 右肩键：前进演示 - 🔴 修改
        m_driverController.rightBumper().whileTrue(
        Commands.run(() -> 
            m_drivetrain.drive(0.3 * m_maxSpeed, 0, 0, true),
            m_drivetrain
        )
        );


        // 左扳机：慢速模式
        m_driverController.leftTrigger().onTrue(
            Commands.runOnce(() -> m_speedMultiplier = 0.2)
        );
        m_driverController.leftTrigger().onFalse(
            Commands.runOnce(() -> m_speedMultiplier = Constants.OI.DEFAULT_SPEED_MULTIPLIER)
        );
    }

    //获取选中的自动命令
    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }
}