#include "Wbc.h"


namespace controllers{


    void Wbc::kin_wbc(Robot_info &robot,Gait_info &gait,Swing_info &swing,double dt)
    {
        //支撑腿的数量
        int contact_num = gait.contact_num;
        
        // ---------------------------------------------
        // task1 ：支撑腿不动
        // ---------------------------------------------
        //中间变量        
        N_pre.setZero();
        J_pre[0].setZero();
        if( contact_num != 0 )//有支撑腿
        {
            int stand_num = 0;
            //传入量
            for(int i=0;i<4;i++)
            {
                if(gait.Gait_state[i] == 1)
                {
                    J[0].block<3,18>(stand_num*3,0) = robot.J_foot[i];
                    Jdtqdt[0].segment<3>(stand_num*3) = robot.Jdt_qdt_foot[i];
                    stand_num++;
                }
            }
            if( stand_num != contact_num ) { ROS_ERROR("stand: num != contact_num");}
            //中间变量
            Eigen::MatrixXd J1_pre_inv_(18, 3 * contact_num);
            utils::WeightedInverse(J[0].topRows(3 * contact_num),robot.M_q_inv,J1_pre_inv_);
            N_pre = Eigen::MatrixXd::Identity(18, 18) - J1_pre_inv_ * J[0].topRows(3 * contact_num);
            //目标量
            q_ddt_cmd[0] = J1_pre_inv_ * ( -Jdtqdt[0].head(3 * contact_num) );   
        }else{ //没有支撑腿
            N_pre = Eigen::MatrixXd::Identity(18, 18);
            q_ddt_cmd[0].setZero();
        }

        // ---------------------------------------------
        // task2 ：姿态控制 
        //
        // ---------------------------------------------
        //传入量
        J[1].block<3,3>(0,3) = robot.body_Rot_world;
        Jdtqdt[1] = robot.Jdt_qdt_base_angular;
        //角度 roll pitch 保持0 yaw按照速度误差
        x_delta_cmd[1] = -robot.euler;
        x_delta_cmd[1][2] = robot.world_omega_des[2] * dt;
        x_dt_cmd[1] = robot.world_omega_des;
        x_ddt_cmd[1] = ori_kp * x_delta_cmd[1] + ori_kd * ( robot.world_omega_des - robot.body_Rot_world * robot.body_Omega );
        //中间变量
        J_pre[1] = J[1] * N_pre;
        Eigen::MatrixXd J2_pre_inv_(18, 3);
        utils::WeightedInverse(J_pre[1],robot.M_q_inv,J2_pre_inv_);
        N_pre = N_pre * ( Eigen::MatrixXd::Identity(18, 18) - J2_pre_inv_ * J_pre[1] );
        //目标量 
        q_delta_cmd[1] = J2_pre_inv_ * ( x_delta_cmd[1] - J[1]*q_delta_cmd[0] );
        q_dt_cmd[1] = J2_pre_inv_ * ( x_dt_cmd[1] - J[1]*q_dt_cmd[0] );
        q_ddt_cmd[1] = q_ddt_cmd[0] + J2_pre_inv_ * ( x_ddt_cmd[1] - J[1]*q_ddt_cmd[0] - robot.Jdt_qdt_base_angular);
        
        // ---------------------------------------------
        // task3 ：平动控制
        // ---------------------------------------------
        //传入量
        J[2].block<3,3>(0,0) = robot.body_Rot_world;
        Jdtqdt[2] = robot.Jdt_qdt_base_linear;
        // xy速度跟踪 z保持稳定跟踪
        x_delta_cmd[2] = robot.world_Vel_des * dt;
        x_delta_cmd[2][2] = robot.z_des - robot.world_Pos_com[2];
        x_dt_cmd[2] = robot.world_Vel_des;
        x_ddt_cmd[2] = trans_kp * x_delta_cmd[2] + trans_kd * ( robot.world_Vel_des - robot.world_Vel_com );
        //中间变量
        J_pre[2] = J[2] * N_pre;
        Eigen::MatrixXd J3_pre_inv_(18, 3);
        utils::WeightedInverse(J_pre[2],robot.M_q_inv,J3_pre_inv_);
        N_pre = N_pre * ( Eigen::MatrixXd::Identity(18, 18) - J3_pre_inv_ * J_pre[2] );
        //目标量 
        q_delta_cmd[2] = q_delta_cmd[1] + J3_pre_inv_ * ( x_delta_cmd[2] - J[2]*q_delta_cmd[1] );
        q_dt_cmd[2] = q_dt_cmd[1] + J3_pre_inv_ * ( x_dt_cmd[2] - J[2]*q_dt_cmd[1] );
        q_ddt_cmd[2] = q_ddt_cmd[1] + J3_pre_inv_ * ( x_ddt_cmd[2] - J[2]*q_ddt_cmd[1] - robot.Jdt_qdt_base_linear);
        
        // ---------------------------------------------
        // task4 ：摆动腿控制
        // ---------------------------------------------
        if( contact_num != 4 )//有摆动腿
        {
            //传入量
            int swing_num = 0;
            Eigen::VectorXd x_dt_now(3*(4-contact_num));
            for(int i=0;i<4;i++)
            {
                if(gait.Gait_state[i] == 0)//摆动腿
                {
                    J[3].block<3,18>(swing_num*3,0) = robot.J_foot[i];
                    Jdtqdt[3].segment<3>(swing_num*3) = robot.Jdt_qdt_foot[i];
                    x_delta_cmd[3].segment<3>(swing_num*3) = swing.world_POS_foot.col(i) - robot.world_POS.col(i);
                    x_dt_cmd[3].segment<3>(swing_num*3) = swing.world_VEL_foot.col(i);
                    x_dt_now.segment<3>(swing_num*3) = robot.world_VEL.col(i);
                    swing_num++;
                }
            }
            if( swing_num != (4-contact_num) ) { ROS_ERROR("swing: num != contact_num");}
            x_ddt_cmd[3].head(3*swing_num) = swing_kp * ( x_delta_cmd[3].head(3*swing_num) ) 
                                + swing_kd * ( x_dt_cmd[3].head(3*swing_num) - x_dt_now );
            //中间变量
            J_pre[3].topRows(3 * swing_num) = J[3].topRows(3 * swing_num) * N_pre;
            Eigen::MatrixXd J4_pre_inv_(18, 3 * swing_num);
            utils::WeightedInverse(J_pre[3].topRows(3 * swing_num),robot.M_q_inv,J4_pre_inv_);
            //目标量 
            q_delta_cmd[3] = q_delta_cmd[2] + J4_pre_inv_ * ( x_delta_cmd[3].head(3*swing_num) 
                                                                    - J[3].topRows(3 * swing_num)*q_delta_cmd[2] );
            q_dt_cmd[3] = q_dt_cmd[2] + J4_pre_inv_ * ( x_dt_cmd[3].head(3*swing_num)
                                                                    - J[3].topRows(3 * swing_num)*q_dt_cmd[2] );
            q_ddt_cmd[3] = q_ddt_cmd[2] + J4_pre_inv_ * ( x_ddt_cmd[3].head(3*swing_num) 
                                                        - J[3].topRows(3 * swing_num)*q_ddt_cmd[2] - Jdtqdt[3].head(3*swing_num) );

        }else{//没有就跳过这个任务；
            q_delta_cmd[3] = q_delta_cmd[2];
            q_dt_cmd[3] = q_dt_cmd[2];
            q_ddt_cmd[3] = q_ddt_cmd[2];
        }
    
    }

    void Wbc::wbic(Robot_info &robot,Gait_info &gait,Swing_info &swing)
    {
        //使用会变化的取内存 .block(startRow, startCol, numRows, numCols) 和 .segment(startIndex, size)


        int contact_num = gait.contact_num;

        //CE
        dyn_CE.block<6,6>(0,0) = robot.M_q.block<6,6>(0,0); //q
        Eigen::MatrixXd J_c_T;
        J_c_T.resize(6,3*contact_num);
        J_c_T.noalias() = J[0].block(0,0,3*contact_num,6).transpose();
        dyn_CE.block(0,6,6,3*contact_num) = -J_c_T;

        qp_CE = dyn_CE.transpose();
        //ce
        Eigen::VectorXd f_mpc;
        f_mpc.resize(3*contact_num);
        int stand_num = 0;
        for(int i=0;i<4;i++)
        {
            if(gait.Gait_state[i] == 1)//支撑腿
            {
                f_mpc.segment<3>(stand_num*3) = robot.mpc_force.segment<3>(3*i);
                stand_num++;
            }
        }
        if( stand_num != contact_num ) { ROS_ERROR("qp: num != contact_num");}
        qp_ce = robot.h_q_dq.head(6) + q_ddt_cmd[3].head(6);
        qp_ce.noalias() -= J_c_T * f_mpc;

        //CI
        CA.setZero(); UB.setZero(); LB.setZero();
        for(int i=0;i<contact_num;i++)
        {
            CA.block<5,3>(5*i,3*i) = ci;
            UB.segment<5>(5*i) = ubi;
            LB.segment<5>(5*i) = lbi;
        }
        dyn_CI.setZero();
        dyn_CI.block(0,6,5*contact_num,3*contact_num) = -CA.block(0,0,5*contact_num,3*contact_num);
        dyn_CI.block(5*contact_num,6,5*contact_num,3*contact_num) = CA.block(0,0,5*contact_num,3*contact_num);

        qp_CI.noalias() = dyn_CI.transpose();
        //ci
        qp_ci.segment(0,5*contact_num) = UB.head(5*contact_num) - CA.block(0,0,5*contact_num,3*contact_num) * f_mpc;
        qp_ci.segment(5*contact_num,5*contact_num) = CA.block(0,0,5*contact_num,3*contact_num) * f_mpc - LB.head(5*contact_num);


        eigenToQuadProgMat(qp_G.block(0,0,6+3*contact_num,6+3*contact_num),quadprog_G);   eigenToQuadProgVec(qp_g0.segment(0,6+3*contact_num),quadprog_g0);
        eigenToQuadProgMat(qp_CE.block(0,0,6+3*contact_num,6),quadprog_CE);   eigenToQuadProgVec(qp_ce.segment(0,6),quadprog_ce0);
        eigenToQuadProgMat(qp_CI.block(0,0,6+3*contact_num,5*contact_num),quadprog_CI);   eigenToQuadProgVec(qp_ci.segment(0,5*contact_num),quadprog_ci0);

        
        quadprog_x.resize(6+3*contact_num);
        double cost = quadprogpp::solve_quadprog(quadprog_G, quadprog_g0, quadprog_CE, quadprog_ce0, quadprog_CI, quadprog_ci0, quadprog_x);

        quadProgToEigenVec(quadprog_x,qp_result);
    }


}