
// //基于TF树控制臂臂
// //mode:0基于吸盘平移，mode:1基于原点坐标系平移
// // static void ArmControByTF(Transform *TF,uint8_t mode,float x_offset,float y_offset,float roll_offset,float pitch_offset){
// //     if (Control_ARM_flag == 0) {
// //         memcpy(&arm_origin_place,&arm_param_t,sizeof(arm_param_t));
// //         arm_origin_place.height = Z_current_height;
// //         arm_origin_place.mid_yaw_angle = -mid_yaw_motor->measure.total_angle;
// //         arm_origin_place.assorted_yaw_angle = assorted_yaw_angle;
// //         arm_origin_place.assorted_roll_angle = assorted_roll_angle;
// //         arm_origin_place.tail_motor_angle =  -tail_motor_encoder->measure.total_angle;
// //         arm_origin_place.big_yaw_angle = big_yaw_origin_angle;
// //         // float a1 = abs(assorted_yaw_angle_);
// //         // float a2 = 180 - abs(mid_yaw_angle_);
// //         // float a3 = 180 - a1 - a2;
// //         // arm_origin_place.big_yaw_angle = (mid_yaw_angle_>=0) ? (big_yaw_angle_ + a3) : (big_yaw_angle_ - a3); 
// //         // float n = sqrtf(arm1*arm1 + arm2*arm2 - 2*arm1*arm2*cosf(a2*DEGREE_2_RAD));
// //         // float a4 = asinf(arm2*sinf(a2)/n) - a3;
// //         memset(&arm_rc_contro_place,0,sizeof(arm_rc_contro_place));
// //         arm_rc_contro_place.x = arm1 + arm2 + arm3;
// //         // arm_rc_contro_place.x = arm3 + n*sqrtf(1-sinf(a4)*sinf(a4));
// //         // arm_rc_contro_place.y = assorted_yaw_angle_>=0 ? -n*sinf(a4) : n*sinf(a4);
// //         // arm_rc_contro_place.z = 0;

// //         cal_ARM_TF(&arm_controller_TF,0,x_offset,y_offset);
// //         Control_ARM_flag = 1;
// //     }
// //     if(cal_ARM_TF(&arm_controller_TF,mode,x_offset,y_offset)){
// //         arm_origin_place.assorted_roll_angle += roll_offset;
// //         arm_origin_place.tail_motor_angle += pitch_offset;
// //         LIMIT_MIN_MAX(arm_origin_place.tail_motor_angle,-90,90);
// //         if(!cal_ARM_joint_angle(TF))
// //         {
// //             arm_origin_place.assorted_roll_angle -= roll_offset;
// //             arm_origin_place.tail_motor_angle -= pitch_offset;
// //         }
// //     }
// // }


// /* 臂臂各关节堵转检测，原理是当其输出较大电流但未发生位移超过1s时，认为其卡住了，将整个臂臂强制离线 */
// // todo:后续看该如何加入复活的操作
// // static void ArmStuckDetection(){
// //     static uint16_t joint_move_cnt[5];
// //     static float joint_origin_angle[5];
// //     static uint8_t joint_error_flag = 0;
// //     DJIMotorInstance* djimotor_ptr;
// //     DRMotorInstance* DRmotor_ptr;
// //     void* motor;
// //     if(joint_crash_flag !=1 ){
// //         for(int i = 0;i<5;i++){
// //             if(((DJIMotorInstance*)joint_motor[i])->motor_type == M2006 || ((DJIMotorInstance*)joint_motor[i])->motor_type == M3508){
// //                 djimotor_ptr = (DJIMotorInstance*)joint_motor[i];
// //                 if(abs(djimotor_ptr->motor_controller.speed_PID.Output) >= djimotor_ptr->motor_controller.speed_PID.MaxOut/4.0f && djimotor_ptr->stop_flag!=0){
// //                     if(joint_move_cnt[i]==0){
// //                         joint_origin_angle[i]=djimotor_ptr->measure.total_angle;
// //                     }
// //                     joint_move_cnt[i]++;
// //                     if(joint_move_cnt[i] > 1000 && limit_bool(djimotor_ptr->measure.total_angle-joint_origin_angle[i],10,-10)){
// //                         joint_error_flag |= (0x0001<<i);
// //                     }else{
// //                         joint_move_cnt[i]=0;
// //                     }
// //                 }else{
// //                     joint_move_cnt[i] = 0;
// //                 }
// //             }else{
// //                 DRmotor_ptr = (DRMotorInstance*)joint_motor[i];
// //                 if(abs(DRmotor_ptr->motor_controller.speed_PID.Output) >= DRmotor_ptr->motor_controller.speed_PID.MaxOut/4.0f && DRmotor_ptr->stop_flag!=0){
// //                     if(joint_move_cnt[i]==0){
// //                         joint_origin_angle[i]=DRmotor_ptr->measure.total_angle;
// //                     }
// //                     joint_move_cnt[i]++;
// //                     if(joint_move_cnt[i] > 1000 && limit_bool(DRmotor_ptr->measure.total_angle-joint_origin_angle[i],2,-2)){
// //                         joint_error_flag |= (0x0001<<i);
// //                     }else{
// //                         joint_move_cnt[i]=0;
// //                     }
// //                 }else{
// //                     joint_move_cnt[i] = 0;
// //                 }
// //             }

// //             if(joint_error_flag){
// //                 LOGWARNING("[arm] joint was crashed, id: %x", joint_error_flag);
// //                 joint_crash_flag = 1;
// //             }
// //         }
// //     }
// //     if(joint_crash_flag == 1){
// //         ArmDisable();
// //     }
// // }


// // 根据控制数据变换末端位姿（重构TF树）
// static uint8_t cal_ARM_TF(Transform* TF,uint8_t mode,float dx,float dy){
//     /* 原逆解算式，直接计算末端位姿的变换矩阵 */
//     // 原末端位姿变换矩阵
//     // float last_TransformationMatrix_data[16] = {
//     //     1 - 2 * powf(TF->localRotation.y, 2) - 2 * powf(TF->localRotation.z, 2), 2 * (TF->localRotation.x * TF->localRotation.y - TF->localRotation.z * TF->localRotation.w), 2 * (TF->localRotation.x * TF->localRotation.z + TF->localRotation.y * TF->localRotation.w), TF->localPosition.x,
//     //     2 * (TF->localRotation.x * TF->localRotation.y + TF->localRotation.z * TF->localRotation.w), 1 - 2 * powf(TF->localRotation.x, 2) - 2 * powf(TF->localRotation.z, 2), 2 * (TF->localRotation.y * TF->localRotation.z - TF->localRotation.x * TF->localRotation.w), TF->localPosition.y,
//     //     2 * (TF->localRotation.x * TF->localRotation.z - TF->localRotation.y * TF->localRotation.w), 2 * (TF->localRotation.y * TF->localRotation.z + TF->localRotation.x * TF->localRotation.w), 1 - 2 * powf(TF->localRotation.x, 2) - 2 * powf(TF->localRotation.y, 2), TF->localPosition.z,
//     //     0, 0, 0, 1};
//     // Matrix last_TransformationMatrix = {4, 4, last_TransformationMatrix_data};

//     // float TransformationMatrix_data[16] = {
//     //     cosf(data->Roatation_Vertical) * cosf(data->Roatation_Horizontal), -sinf(data->Roatation_Vertical), cosf(data->Roatation_Vertical) * sinf(data->Roatation_Horizontal), data->Translation_x,
//     //     sinf(data->Roatation_Vertical) * cosf(data->Roatation_Horizontal), cosf(data->Roatation_Vertical), sinf(data->Roatation_Vertical) * sinf(data->Roatation_Horizontal), data->Translation_y,
//     //     -sinf(data->Roatation_Horizontal), 0, cosf(data->Roatation_Horizontal), 0,
//     //     0, 0, 0, 1};
//     // Matrix TransformationMatrix = {4, 4, TransformationMatrix_data};
//     // M_mul(&Matrix_keep, &last_TransformationMatrix, &TransformationMatrix);

//     /*
//             ↑x
//             |
//         y←——O———
//            z|    
//     */
//     Vector3 sub_vec;
//     //mode:0 基于吸盘朝向进行平移
//     if(mode == 0){
//         /* 改版正解算式，从arm2处(arm2初始坐标+偏移坐标)重构TF树 */
//         // arm2处的旋转矩阵
//         float arm3_roll_angle_new                            = degree2rad_limited(assorted_roll_angle_);
//         float TransformationMatrix_arm2_origin__origin__q_data[9] = {
//             1, 0, 0,
//             0, cosf(arm3_roll_angle_new),   -sinf(arm3_roll_angle_new),
//             0, sinf(arm3_roll_angle_new),   cosf(arm3_roll_angle_new),
//             };
//         Matrix TransformationMatrix_arm2_origin__origin__q_m = {3, 3, TransformationMatrix_arm2_origin__origin__q_data};
//         // arm3相对于arm2的旋转矩阵
//         float arm4_pitch_angle_new                           = degree2rad_limited(tail_motor_angle_);
//         float TransformationMatrix_arm3_arm2__arm2__q_data[9] = {
//             cosf(arm4_pitch_angle_new),0, sinf(arm4_pitch_angle_new),
//             0, 1, 0,
//             -sinf(arm4_pitch_angle_new),0,  cosf(arm4_pitch_angle_new),
//         };
//         Matrix TransformationMatrix_arm3_arm2__arm2__q_m = {3, 3, TransformationMatrix_arm3_arm2__arm2__q_data};

//         // target旋转矩阵
//         float TransformationMatrix_target_origin__origin__q_data[9];
//         Matrix TransformationMatrix_target_origin__origin__q_m = {3,3,TransformationMatrix_target_origin__origin__q_data};
//         M_mul(&TransformationMatrix_target_origin__origin__q_m,&TransformationMatrix_arm2_origin__origin__q_m,&TransformationMatrix_arm3_arm2__arm2__q_m);
//         // sub平移矩阵
//         float TransformationMatrix_target_sub__origin__p_data[3] = {dx,dy,0};
//         Matrix TransformationMatrix_target_sub__origin__p_m = {3,1,TransformationMatrix_target_sub__origin__p_data};
//         M_mul(&Matrix_keep, &TransformationMatrix_target_origin__origin__q_m,&TransformationMatrix_target_sub__origin__p_m);

//         memcpy((float*)&sub_vec,(float*)&Matrix_data_keep,sizeof(Vector3));
//         arm_rc_contro_place.x += sub_vec.x;
//         arm_rc_contro_place.y += sub_vec.y;
//         arm_rc_contro_place.z += sub_vec.z;
//         // arm4平移矩阵
//         float TransformationMartix_target_arm3__origin__p_data[3] = {arm4,0,0};
//         Matrix TransformationMartix_target_arm3__origin__p_m = {3,1,TransformationMartix_target_arm3__origin__p_data};
//         M_mul(&Matrix_keep, &TransformationMatrix_target_origin__origin__q_m,&TransformationMartix_target_arm3__origin__p_m);

//         // 获取最终位姿
//         TF->localPosition.x = Matrix_data_keep[0] + arm_rc_contro_place.x;
//         TF->localPosition.y = Matrix_data_keep[1] + arm_rc_contro_place.y;
//         TF->localPosition.z = Matrix_data_keep[2] + arm_rc_contro_place.z;
//         rotationToQuaternion((float *)TransformationMatrix_target_origin__origin__q_data,&TF->localRotation);
//     }else if(mode==1){
//     // mode:1 基于世界坐标系进行平移
//         sub_vec.x = dx;sub_vec.y = dy;sub_vec.z = 0;
//         arm_rc_contro_place.x += sub_vec.x;
//         arm_rc_contro_place.y += sub_vec.y;
//         arm_rc_contro_place.z += sub_vec.z;
//         TF->localPosition.x += sub_vec.x;
//         TF->localPosition.y += sub_vec.y;
//     }

//     // 判断末端位置是否超出有效解范围
//     float TF_X       = sqrtf(TF->localPosition.x * TF->localPosition.x + TF->localPosition.y * TF->localPosition.y);
//     // 未超出则返回1
//     if (limit_bool(TF_X,0.745f,0.430f) && limit_bool(TF->localPosition.z,0.03+arm4,-0.575-arm4))
//         return 1;
//     // 超出则撤销更改，返回0
//     arm_rc_contro_place.x -= sub_vec.x;
//     arm_rc_contro_place.y -= sub_vec.y;
//     arm_rc_contro_place.z -= sub_vec.z;
//     return 0;
// }
// //根据末端TF树计算关节角度
// static uint8_t cal_ARM_joint_angle(Transform* new_TF)
// {
//     // memcpy(&arm_controller_TF, new_TF, sizeof(Transform)); //方便监视

//     arm_controller_data_s update_data;
//     if(Update_angle(new_TF, &update_data)) {
//         // if (Control_ARM_flag == 0){
//         //     arm_origin_place.big_yaw_angle-=update_data.big_yaw_angle;
//         //     arm_origin_place.height-=new_TF->localPosition.z * 1000;
//         // }else if(Control_ARM_flag==2){
//         //     memset(&arm_origin_place,0,sizeof(arm_origin_place));
//         // }
//         arm_param_t.big_yaw_angle      = arm_origin_place.big_yaw_angle + update_data.big_yaw_angle;
//         arm_param_t.mid_yaw_angle      = update_data.mid_yaw_angle;
//         arm_param_t.assorted_yaw_angle = update_data.assorted_yaw_angle;
//         arm_param_t.assorted_roll_angle = arm_origin_place.assorted_roll_angle;
//         arm_param_t.tail_motor_angle = arm_origin_place.tail_motor_angle;
//         arm_param_t.height = arm_origin_place.height + new_TF->localPosition.z * 1000;
//         return 2;
//     }
//     return 0;
// }