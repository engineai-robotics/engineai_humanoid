#include "Controllers/close_chain_mapping.h"

//////********************print******************************//////

void Decouple::printVector3d(const Eigen::Vector3d &vec)
{
    std::cout << "[" << vec[0] << ", " << vec[1] << ", " << vec[2] << "]";
}

void Decouple::printKinematicsResult(const InsKinematicsResult &result)
{
    std::cout << "r_A: ";
    for (const auto &vec : result.r_A)
    {
        printVector3d(vec);
        std::cout << " ";
    }
    std::cout << std::endl;

    std::cout << "r_B: ";
    for (const auto &vec : result.r_B)
    {
        printVector3d(vec);
        std::cout << " ";
    }
    std::cout << std::endl;

    std::cout << "r_C: ";
    for (const auto &vec : result.r_C)
    {
        printVector3d(vec);
        std::cout << " ";
    }
    std::cout << std::endl;

    std::cout << "r_bar: ";
    for (const auto &vec : result.r_bar)
    {
        printVector3d(vec);
        std::cout << " ";
    }
    std::cout << std::endl;

    std::cout << "r_rod: ";
    for (const auto &vec : result.r_rod)
    {
        printVector3d(vec);
        std::cout << " ";
    }
    std::cout << std::endl;

    std::cout << "THETA: ";
    std::cout << result.THETA;
    std::cout << std::endl;
}
//////********************print******************************//////

//////********************inverse kinematics*****************//////
InsKinematicsResult
Decouple::inverse_kinematics(
    double q_roll,
    double q_pitch, bool leftLegFlag)
{
    InsKinematicsResult result;
    bool legal_ori = true;

    result.THETA = Eigen::Vector2d::Zero();

    double l_bar = 45; // # up

    // l_rod and l_spacing have 2 elements each
    double l_rod[2] = {215.975, 121}; // # long rod
    // double l_rod2 =  0.121 ;//# short rod
    double l_spacing1 = 44.5; // # long rod
    double l_spacing2 = 40;   // # short rod

    double short_link_angle_0 = 14.2 * M_PI / 180;
    double long_link_angle_0 = 15.5 * M_PI / 180;

    double r_B1_0_x = -20.7 - l_bar * cos(long_link_angle_0);
    double r_B1_0_z = 215.01 - l_bar * sin(long_link_angle_0);
    double r_B2_0_x = -6.3 - l_bar * cos(short_link_angle_0);
    double r_B2_0_z = 119.84 - l_bar * sin(short_link_angle_0);

    if (leftLegFlag)
    { // left leg
        l_spacing1 = -l_spacing1;
        l_spacing2 = -l_spacing2;
    }

    // Define points
    Eigen::Vector3d r_A1_0{-20.7, l_spacing1, 215.01};
    Eigen::Vector3d r_B1_0{r_B1_0_x, l_spacing1, r_B1_0_z};
    Eigen::Vector3d r_C1_0{-43.4, l_spacing1, -12};

    Eigen::Vector3d r_A2_0{-6.3, -l_spacing2, 119.84};
    Eigen::Vector3d r_B2_0{r_B2_0_x, -l_spacing2, r_B2_0_z};
    Eigen::Vector3d r_C2_0{-43.4, -l_spacing2, -12};

    std::vector<Eigen::Vector3d> r_A_0;
    r_A_0.push_back(r_A1_0);
    r_A_0.push_back(r_A2_0);

    std::vector<Eigen::Vector3d> r_B_0;
    r_B_0.push_back(r_B1_0);
    r_B_0.push_back(r_B2_0);

    std::vector<Eigen::Vector3d> r_C_0;
    r_C_0.push_back(r_C1_0);
    r_C_0.push_back(r_C2_0);

    // Rotation matrices
    Eigen::Matrix3d R_y = Eigen::Matrix3d::Zero();
    R_y << cos(q_pitch), 0, sin(q_pitch),
        0, 1, 0,
        -sin(q_pitch), 0, cos(q_pitch);

    Eigen::Matrix3d R_x = Eigen::Matrix3d::Zero();
    R_x << 1, 0, 0,
        0, cos(q_roll), -sin(q_roll),
        0, sin(q_roll), cos(q_roll);

    Eigen::Matrix3d x_rot = R_y * R_x;

    // Vectors to store results
    // std::vector<Eigen::Vector3d> results;

    for (int i = 0; i < 2; i++)
    {
        Eigen::Vector3d r_A_i = r_A_0[i];
        Eigen::Vector3d r_C_i = x_rot * r_C_0[i];
        Eigen::Vector3d rBA_bar = r_B_0[i] - r_A_0[i];

        Eigen::Vector3d r_AB_0 = r_B_0[i] - r_A_0[i];
        Eigen::Vector3d r_CA = r_A_i - r_C_i;

        double M = pow(l_rod[i], 2) - pow(r_CA.norm(), 2) - pow(l_bar, 2);
        double N = r_CA[0] * r_AB_0[0] + r_CA[2] * r_AB_0[2];
        double K = r_CA[0] * r_AB_0[2] - r_CA[2] * r_AB_0[0];

        double a = 4 * (pow(K, 2) + pow(N, 2));
        double b = -4 * M * K;
        double c = pow(M, 2) - 4 * pow(N, 2);
        double theta_i = 0.0;
        if (pow(b, 2) - 4 * a * c < 0.0) // to visualize the working space later
        {
            theta_i = 0.0;
            legal_ori = false;
            // printf("input ankle joint ori is outof ranges!! leg: %d, roll: %f, pitch: %f\n", i, q_roll, q_pitch);
        }
        else
            theta_i = std::asin((-b + sqrt(pow(b, 2) - 4 * a * c)) / (2 * a));

        Eigen::Matrix3d R_y_theta = Eigen::Matrix3d::Zero();
        R_y_theta << std::cos(theta_i), 0, std::sin(theta_i),
            0, 1, 0,
            -std::sin(theta_i), 0, std::cos(theta_i);

        Eigen::Vector3d r_B_i = r_A_i + R_y_theta * rBA_bar;
        Eigen::Vector3d r_bar_i = r_B_i - r_A_i;
        Eigen::Vector3d r_rod_i = r_C_i - r_B_i;

        // Populate results
        result.r_A.push_back(r_A_i);
        result.r_B.push_back(r_B_i);
        result.r_C.push_back(r_C_i);
        result.r_bar.push_back(r_bar_i);
        result.r_rod.push_back(r_rod_i);
        result.THETA[i] = theta_i;
    }

    return result;
}
//////********************inverse kinematics*****************//////

//////********************jacobian***************************//////
std::vector<Eigen::MatrixXd>
Decouple::jacobian(const std::vector<Eigen::Vector3d> &r_C,
                   const std::vector<Eigen::Vector3d> &r_bar,
                   const std::vector<Eigen::Vector3d> &r_rod,
                   double q_pitch)
{
    // Assuming r_C, r_bar, r_rod are vectors of Eigen::Vector3d with at least 2 elements each

    Eigen::Vector3d s_11{0, 1, 0}; // A1 point unit direction vector
    Eigen::Vector3d s_21{0, 1, 0}; // A2 point unit direction vector

    Eigen::MatrixXd J_x = Eigen::MatrixXd::Zero(2, 6);
    J_x.block<1, 3>(0, 0) = r_rod[0].transpose();
    J_x.block<1, 3>(1, 0) = r_rod[1].transpose();
    J_x.block<1, 3>(0, 3) = (r_C[0].cross(r_rod[0])).transpose();
    J_x.block<1, 3>(1, 3) = (r_C[1].cross(r_rod[1])).transpose();

    Eigen::MatrixXd J_theta = Eigen::MatrixXd::Zero(2, 2);
    J_theta(0, 0) = s_11.dot(r_bar[0].cross(r_rod[0]));
    J_theta(1, 1) = s_21.dot(r_bar[1].cross(r_rod[1]));

    /*after*/
    Eigen::MatrixXd J_q = Eigen::MatrixXd::Zero(6, 2); // 第一列对应qd_pitch，第二列对应qd_roll
    J_q(3, 1) = std::cos(q_pitch);
    J_q(4, 0) = 1;
    J_q(5, 1) = -std::sin(q_pitch);

    // std::cout << "J_x: " << J_x << std::endl;
    // std::cout << "J_q: " << J_q << std::endl;
    // std::cout << "J_x*J_q:" << J_x*J_q << std::endl;
    Eigen::MatrixXd J_Temp = (J_x * J_q);
    // Eigen::MatrixXd J_ankle = (J_x*J_q).inverse() * J_theta;
    Eigen::MatrixXd J_motor2Joint = J_Temp.inverse() * J_theta;
    Eigen::MatrixXd J_Joint2motor = J_theta.inverse() * J_Temp;
    // Eigen::MatrixXd J_ankle = (J_x*J_q).completeOrthogonalDecomposition().pseudoInverse() * J_theta;
    std::vector<Eigen::MatrixXd> J_ankle;
    J_ankle.push_back(J_motor2Joint);
    J_ankle.push_back(J_Joint2motor);
    return J_ankle;
    /*after*/
}
//////********************jacobian***************************//////

// from x to theta， from S to P
std::pair<Eigen::Vector2d, std::vector<Eigen::MatrixXd>>
Decouple::getDecouple(double roll, double pitch, bool leftLegFlag)
{
    InsKinematicsResult kinematics = inverse_kinematics(roll, pitch, leftLegFlag);
    std::vector<Eigen::MatrixXd> Jac = jacobian(kinematics.r_C, kinematics.r_bar, kinematics.r_rod, pitch);
    return {kinematics.THETA, Jac};
}

//////********************forward kinematics*****************//////
ForwardMappingResult
Decouple::forwardKinematics(const Eigen::Vector2d &thetaRef, bool leftLegFlag)
{

    ForwardMappingResult mapping_result;

    int count = 0;
    Eigen::Vector2d f_error{10, 10};
    Eigen::Vector2d x_c_k{0, 0}; // {pitch, roll}

    std::vector<Eigen::MatrixXd> Jac;

    /*after*/
    while (f_error.norm() > 1e-3 && count < 100)
    {
        InsKinematicsResult kinematics = inverse_kinematics(x_c_k[1], x_c_k[0], leftLegFlag);
        // printKinematicsResult(kinematics);

        Jac = jacobian(kinematics.r_C, kinematics.r_bar, kinematics.r_rod, x_c_k[0]);
        // std::cout << "===== count:" << count << "\n Jac: " << Jac << "\n THEAT:" << kinematics.THETA << std::endl;
        Eigen::MatrixXd J_motor2Joint = Jac[0];
        // Eigen::MatrixXd J_Joint2motor = Jac[1];
        if (J_motor2Joint.hasNaN())
        {
            std::cerr << "Decouple::forwardKinematics() Jac is nan!!" << std::endl;
            std::cerr << "  roll x_c_k[1],pitch  x_c_k[0] n!!" << x_c_k[1] << "   ---   " << x_c_k[0] << std::endl;
            x_c_k << 0, 0;
#ifdef TARGET_MUJOCO
            exit(1);
#else
            mapping_result.count = -1;
            mapping_result.ankle_joint_ori = x_c_k;
            mapping_result.Jac = Jac;
            return mapping_result; // -1 是失败的标记
#endif
        }

        Eigen::Vector2d thetaCal = Eigen::Vector2d(kinematics.THETA[0], kinematics.THETA[1]);

        f_error = thetaRef - thetaCal;

        x_c_k = x_c_k + J_motor2Joint * f_error;
        // std::cout <<  " thetaCal: " << thetaCal << "\n f_error: " << f_error << "\n pitch_roll:" << x_c_k << std::endl;

        count++;
    }
    /*after*/

    mapping_result.count = count;
    mapping_result.ankle_joint_ori = x_c_k;
    mapping_result.Jac = Jac;

    return mapping_result; // -1 是失败的标记
}
//////********************forward kinematics*****************//////

// from x to theta， from Serial to Parallel
void Decouple::getDecoupleTorque(Eigen::Matrix<double, -1, 1> &tau, Eigen::Matrix<double, -1, 1> &q)
{
    double rightPitch, rightRoll, leftPitch, leftRoll;
    rightPitch = q[10];
    rightRoll = q[11];
    leftPitch = q[4];
    leftRoll = q[5];

    /*after*/
    std::pair<Eigen::Vector2d, std::vector<Eigen::MatrixXd>> left, right; // 联立顺序：先right后left

    right = getDecouple(rightRoll, rightPitch, false);
    // Eigen::MatrixXd J_motor2Joint = Jac[0];
    // Eigen::MatrixXd J_Joint2motor = Jac[1];
    q.segment<2>(10) = right.first;
    tau.segment<2>(10) = right.second[1].transpose() * (tau.segment<2>(10));

    left = getDecouple(leftRoll, leftPitch, true);
    q.segment<2>(4) = left.first;
    tau.segment<2>(4) = left.second[1].transpose() * (tau.segment<2>(4));
    /*after*/
}

// from x to theta， from Serial to Parallel
// force control ,should input current pitch roll
void Decouple::getDecoupleQVT(Eigen::VectorXd &q, Eigen::VectorXd &vel, Eigen::VectorXd &tau)
{
    // q[4] = -q[4];
    // q[10] = -q[10];

    // vel[4] = -vel[4];
    // vel[10] = -vel[10];

    // tau[4] = -tau[4];
    // tau[10] = -tau[10];

    double rightPitch, rightRoll, leftPitch, leftRoll;
    rightPitch = q[10]; // rotation axis [0 1 0]
    rightRoll = q[11];
    leftPitch = q[4]; // rotation axis[0 1 0]
    leftRoll = q[5];

    std::pair<Eigen::Vector2d, std::vector<Eigen::MatrixXd>> left, right; // 联立顺序：先right后left

    right = getDecouple(rightRoll, rightPitch, false);
    q.segment<2>(10) = right.first;
    vel.segment<2>(10) = right.second[1] * (vel.segment<2>(10));
    tau.segment<2>(10) = right.second[0].transpose() * (tau.segment<2>(10));
    // q[4] = -q[4]; // rotation axis[0 -1 0]
    // vel[4] = -vel[4];
    // tau[4] = -tau[4];

    left = getDecouple(leftRoll, leftPitch, true);
    q.segment<2>(4) = left.first;
    vel.segment<2>(4) = left.second[1] * (vel.segment<2>(4));
    tau.segment<2>(4) = left.second[0].transpose() * (tau.segment<2>(4));
    // q[11] = -q[11]; // rotation axis[0 -1 0]
    // vel[11] = -vel[11];
    // tau[11] = -tau[11];
}

void Decouple::getForwardQVT(Eigen::VectorXd &q, Eigen::VectorXd &vel, Eigen::VectorXd &tau)
{

    // q[4] = -q[4]; // rotation axis [0 -1 0]
    // q[11] = -q[11];

    // vel[4] = -vel[4]; // rotation axis [0 -1 0]
    // vel[11] = -vel[11];

    // tau[4] = -tau[4]; // rotation axis [0 -1 0]
    // tau[11] = -tau[11];

    Eigen::Vector2d right_leg_ankle_motor = Eigen::Vector2d::Zero(2, 1);
    Eigen::Vector2d left_leg_ankle_motor = Eigen::Vector2d::Zero(2, 1);

    right_leg_ankle_motor = q.segment<2>(10);
    left_leg_ankle_motor = q.segment<2>(4);

    ForwardMappingResult right_leg_result = forwardKinematics(right_leg_ankle_motor, false);
    q.segment<2>(10) = right_leg_result.ankle_joint_ori;
    vel.segment<2>(10) = right_leg_result.Jac[0] * (vel.segment<2>(10));             // vel transfer from motor to ankle joint
    tau.segment<2>(10) = right_leg_result.Jac[1].transpose() * (tau.segment<2>(10)); // tau transfer from motor to ankle joint
    // q[4] = -q[4];                                                                  // rotation axis[0 -1 0]
    // vel[4] = -vel[4];
    // tau[4] = -tau[4];

    ForwardMappingResult left_leg_result = forwardKinematics(left_leg_ankle_motor, true);
    q.segment<2>(4) = left_leg_result.ankle_joint_ori;
    vel.segment<2>(4) = left_leg_result.Jac[0] * (vel.segment<2>(4));             // vel transfer from motor to ankle joint
    tau.segment<2>(4) = left_leg_result.Jac[1].transpose() * (tau.segment<2>(4)); // tau transfer from motor to ankle joint
    //     q[10] = -q[10];                                                                 // rotation axis[0 -1 0]
    //     vel[10] = -vel[10];
    //     tau[10] = -tau[10];
}
