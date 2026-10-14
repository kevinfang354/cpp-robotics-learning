#include <iostream>
#include <cmath>
#include <string>

// 常量定义：角度转弧度
constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;

// ==========================================
// 1. 关节类 (Joint): 封装电机与关节限位保护
// ==========================================
class Joint
{
private:
    std::string name_;     // 关节名称
    double angle_deg_;     // 当前关节角度 (度)
    double min_limit_deg_; // 最小软限位 (度)
    double max_limit_deg_; // 最大软限位 (度)

public:
    // 构造函数
    Joint(std::string name, double min_limit, double max_limit)
        : name_(name), angle_deg_(0.0), min_limit_deg_(min_limit), max_limit_deg_(max_limit) {}

    // 设置关节角度（带工业级安全限位保护）
    bool setAngle(double target_angle)
    {
        if (target_angle < min_limit_deg_ || target_angle > max_limit_deg_)
        {
            std::cout << "  [⚠️ 安全拦截] 关节 " << name_ << " 目标角度 " << target_angle
                      << "° 超出限位范围 [" << min_limit_deg_ << "°, " << max_limit_deg_ << "°]！拒绝转动！" << std::endl;
            return false;
        }
        angle_deg_ = target_angle;
        return true;
    }

    // 获取弧度值（供正运动学计算）
    double getAngleRad() const
    {
        return angle_deg_ * DEG_TO_RAD;
    }

    double getAngleDeg() const
    {
        return angle_deg_;
    }

    std::string getName() const
    {
        return name_;
    }
};

// ==========================================
// 2. 机械臂类 (TwoDofArm): 组合关节与正运动学
// ==========================================
class TwoDofArm
{
private:
    double link1_length_; // 连杆1长度 (mm)
    double link2_length_; // 连杆2长度 (mm)
    Joint joint1_;        // 关节1 (肩部)
    Joint joint2_;        // 关节2 (肘部)

public:
    // 构造函数
    TwoDofArm(double l1, double l2)
        : link1_length_(l1), link2_length_(l2),
          joint1_("Shoulder (肩关节)", -180.0, 180.0),
          joint2_("Elbow (肘关节)", -150.0, 150.0) {}

    // 机械臂运动控制接口
    bool moveTo(double theta1_deg, double theta2_deg)
    {
        std::cout << "\n>>> 下发运动指令: Joint1 = " << theta1_deg
                  << "°, Joint2 = " << theta2_deg << "° ..." << std::endl;

        bool ok1 = joint1_.setAngle(theta1_deg);
        bool ok2 = joint2_.setAngle(theta2_deg);

        if (ok1 && ok2)
        {
            std::cout << ">>> 运动执行成功！已到达目标位姿。" << std::endl;
            return true;
        }
        else
        {
            std::cout << ">>> 指令未完全执行（触发安全保护）！保持原位。" << std::endl;
            return false;
        }
    }

    // 正运动学解算 (Forward Kinematics)
    // 根据连杆长度与关节角，推导末端执行器在基座坐标系下的 (X, Y) 坐标
    void calculateFK(double &end_x, double &end_y) const
    {
        double q1 = joint1_.getAngleRad();
        double q2 = joint2_.getAngleRad();

        // 几何推导公式
        end_x = link1_length_ * std::cos(q1) + link2_length_ * std::cos(q1 + q2);
        end_y = link1_length_ * std::sin(q1) + link2_length_ * std::sin(q1 + q2);
    }

    // 打印机械臂当前遥测状态
    void printStatus() const
    {
        double x = 0.0, y = 0.0;
        calculateFK(x, y);

        std::cout << "----------------------------------------" << std::endl;
        std::cout << "【机械臂遥测状态】" << std::endl;
        std::cout << "  - " << joint1_.getName() << ": " << joint1_.getAngleDeg() << "°" << std::endl;
        std::cout << "  - " << joint2_.getName() << ": " << joint2_.getAngleDeg() << "°" << std::endl;
        std::cout << "  - 末端执行器位置: X = " << x << " mm, Y = " << y << " mm" << std::endl;
        std::cout << "----------------------------------------" << std::endl;
    }
};

// ==========================================
// 3. 主函数
// ==========================================
int main()
{
    std::cout << "========================================" << std::endl;
    std::cout << "    🤖 2-DOF 机器人运动控制系统启动    " << std::endl;
    std::cout << "========================================" << std::endl;

    // 创建一台连杆1长200mm、连杆2长150mm的机械臂
    TwoDofArm robot(200.0, 150.0);
    robot.printStatus();

    // 动作 1：关节1转动 30°，关节2转动 45°
    robot.moveTo(30.0, 45.0);
    robot.printStatus();

    // 动作 2：水平完全展开（0°, 0°），末端应该正好在 X = 350mm, Y = 0mm
    robot.moveTo(0.0, 0.0);
    robot.printStatus();

    // 动作 3：极限测试！故意输入超出限位的指令（肘关节限位为 ±150°）
    robot.moveTo(45.0, 175.0);
    robot.printStatus();

    return 0;
}