#include "Demo_catch_YOLO.h"

#define IMG_WIDTH 1280
#define IMG_HEIGHT 720
#define Fucing_Crashed 100

enum TaskStep
{
    Task_StartStrategy,
    Task_Walk,
    Task_Look_and_Catch,
    Task_Put,
    Task_Walk_Back,
    Task_FinishStrategy
};

Demo_catch_YOLO::Demo_catch_YOLO()
{
    this->nTaskStep = TaskStep::Task_Look_and_Catch;
}

Demo_catch_YOLO::~Demo_catch_YOLO()
{
}

void Demo_catch_YOLO::MainStrategy(int good_number)
{
    this->good_index = good_number;
    switch (this->nTaskStep)
    {
    case TaskStep::Task_StartStrategy:
    {
        this->StartStrategy(good_number);
        break;
    }
    case TaskStep::Task_Walk:
    {
        this->Walk();
        break;
    }
    case TaskStep::Task_Look_and_Catch:
    {
        this->Look_and_Catch();
        break;
    }
    case TaskStep::Task_Walk_Back:
    {
        this->Walk_Back();
        break;
    }
    case TaskStep::Task_FinishStrategy:
    {
        this->FinalStrategy();
        break;
    }
    default:
        break;
    }
}

void Demo_catch_YOLO::StartStrategy(int good_number)
{
    CStrategyAssistant->vec_strStrategyStage.push_back(" Demo_catch_YOLO - StartStrategy ");
    std::cout << " Demo_catch_YOLO " << std::endl;    
    std::cout << " StartStrategy " << std::endl;

    good_index = good_number;

    this->nTaskStep = TaskStep::Task_Look_and_Catch;
}

void Demo_catch_YOLO::Walk(void)
{
    CStrategyAssistant->vec_strStrategyStage.push_back(" Demo_catch_YOLO - Walk ");
    std::cout << " Walk " << std::endl;

    //向前進
    int count_turn = 0;
    int count_stop = 0;
    CMay->CMayBody->CMayMobilePlatform->TorqueEnable[0] = true;
    CMay->CMayBody->CMayMobilePlatform->TorqueEnable[1] = true;
    CMay->CMayBody->CMayMobilePlatform->Trans_Motor_Torque_Commd();
    while (count_turn < 150)
    {

        CMay->CMayBody->CMayMobilePlatform->Wheel_Commd_Ackermann(70, 0, FORWARD);
        CMay->CMayBody->CMayMobilePlatform->Trans_Motor_Commd();
        //cout << "target          " << (fAvgDeltaDistance - abs(fDistanceShouldRotate)) << endl;
        count_turn++;
        this_thread::sleep_for(chrono::milliseconds(100));
    }
    while (count_stop < 200)
    {
        CMay->CMayBody->CMayMobilePlatform->TorqueEnable[0] = false;
        CMay->CMayBody->CMayMobilePlatform->TorqueEnable[1] = false;
        CMay->CMayBody->CMayMobilePlatform->Trans_Motor_Torque_Commd();
        count_stop++;
        this_thread::sleep_for(chrono::milliseconds(10));
    }

    //右轉90度
    count_turn = 0;
    CMay->CMayBody->CMayMobilePlatform->TorqueEnable[0] = true;
    CMay->CMayBody->CMayMobilePlatform->TorqueEnable[1] = true;
    CMay->CMayBody->CMayMobilePlatform->Trans_Motor_Torque_Commd();
    while (count_turn < 48) //50
    {

        CMay->CMayBody->CMayMobilePlatform->Wheel_Commd_SelfRotation(50, TURN_RIGHT);
        CMay->CMayBody->CMayMobilePlatform->Trans_Motor_Commd();
        //cout << "target          " << (fAvgDeltaDistance - abs(fDistanceShouldRotate)) << endl;
        count_turn++;
        this_thread::sleep_for(chrono::milliseconds(100));
    }

    count_stop = 0;
    while (count_stop < 200)
    {
        CMay->CMayBody->CMayMobilePlatform->TorqueEnable[0] = false;
        CMay->CMayBody->CMayMobilePlatform->TorqueEnable[1] = false;
        CMay->CMayBody->CMayMobilePlatform->Trans_Motor_Torque_Commd();
        count_stop++;
        this_thread::sleep_for(chrono::milliseconds(10));
    }


    this_thread::sleep_for(chrono::milliseconds(500));
    this->nTaskStep = TaskStep::Task_Look_and_Catch;
}



void Demo_catch_YOLO::Look_and_Catch(void)
{
    CStrategyAssistant->vec_strStrategyStage.push_back(" Demo_catch_YOLO - Look_and_Catch ");
    std::cout << " Look_and_Catch " << std::endl;

    CMay->CMayVision->CYolo->names_file = CMay->CMayVision->CYolo->yolo_dir + "/KJ_NEW.names";
    CMay->CMayVision->CYolo->cfg_file = CMay->CMayVision->CYolo->yolo_dir + "/yolov2_KJ_NEW.cfg";
    CMay->CMayVision->CYolo->weights_file = CMay->CMayVision->CYolo->yolo_dir + "/yolov2_KJ_NEW_9800.weights";
    CMay->CMayVision->CYolo->obj_names = CMay->CMayVision->CYolo->objects_names_from_file(CMay->CMayVision->CYolo->names_file);
    CMay->CMayVision->CYolo->OpenDetector();

    CMay->CMayBody->CMayHeadandLiftingPlatform->HeadandTurnShoulderMotor_Command(FIRST_HEAD_MOTOR_ID, 40, 1769);
    CMay->CMayBody->CMayHeadandLiftingPlatform->TransMotorEnable = true;

    CMay->CMayBody->CMayHeadandLiftingPlatform->HeadandTurnShoulderMotor_Command(FIRST_HEAD_MOTOR_ID + 1, 40, 1707);
    CMay->CMayBody->CMayHeadandLiftingPlatform->TransMotorEnable = true;
    this_thread::sleep_for(chrono::milliseconds(3000));



    //yolo
    // 開啟 彩圖 和 深度圖
    CMay->CMayVision->CImageConverter->GetImgForProcessing(0);
    CMay->CMayVision->CImageConverter->GetImgForProcessing(1);

    // 開啟 點雲圖
    CMay->CMayVision->CPclConverter->GetCloudForProcessing();

    cv::Mat mat_img = CMay->CMayVision->CImageConverter->img_for_processing.clone();
    cv::Mat mat_d_img = CMay->CMayVision->CImageConverter->depth_for_processing.clone();

    CMay->CMayVision->CYolo->result_vec = CMay->CMayVision->CYolo->yolo_detector->detect(mat_img);
    CMay->CMayVision->CYolo->draw_boxes(mat_img, CMay->CMayVision->CYolo->result_vec, CMay->CMayVision->CYolo->obj_names);
    cv::imwrite("Yolo1.jpg", mat_img);
    cv::imshow("window name", mat_img);
    cv::waitKey(3);

    int Height_Left = 0;  //左手物品夾取高度
    int Height_Right = 0; //右手物品夾取高度

    vector<int> target{good_index};
    vector<cv::Point> tmp_center;
    vector<int> tmp_obj_id;
    CMay->CMayVision->CYolo->target_obj_center(target, tmp_obj_id, tmp_center);

    for (int i = 0; i < tmp_obj_id.size(); i++)
    {
        if (tmp_obj_id[i] == good_index) // 選擇的物品
        {
            float tmp_ctr_x = tmp_center[i].x;
            float tmp_ctr_y = tmp_center[i].y;
            while (true)
            {
                float tmp_depth = mat_d_img.at<float>(tmp_ctr_y, tmp_ctr_x);
                if (!isnan(tmp_depth) && tmp_depth > 0 && tmp_depth < 1)
                    break;
                tmp_ctr_y++;
            }
            cv::Point pt_center = cv::Point(tmp_ctr_x, tmp_ctr_y);
            int point_cloud_index;

            point_cloud_index = tmp_ctr_y * CMay->CMayVision->CImageConverter->img_for_processing.size().width + tmp_ctr_x;

            cv::Point3f pc_coord(0, 0, 0);
            cv::Point3i pc_color(0, 0, 0);
            pc_coord.x = CMay->CMayVision->CPclConverter->cloud_for_processing->points[point_cloud_index].x;
            pc_coord.y = CMay->CMayVision->CPclConverter->cloud_for_processing->points[point_cloud_index].y;
            pc_coord.z = CMay->CMayVision->CPclConverter->cloud_for_processing->points[point_cloud_index].z;
            cout << "pc_coord " << pc_coord.x << " " << pc_coord.y << " " << pc_coord.z << endl;
            float hx;
            float hy;
            float hz;
            CMay->CMayBody->CMayHeadandLiftingPlatform->HeadCoordinate(pc_coord.y, pc_coord.z, pc_coord.x, hx, hy, hz);
            float *arm_solL_ShoulderToArm = CMay->CMayBody->CMayRightArm->CenterToShoulder(hx, hy, hz);
            float *arm_sol = CMay->CMayBody->CMayRightArm->ShoulderToArm(arm_solL_ShoulderToArm[0], arm_solL_ShoulderToArm[1], arm_solL_ShoulderToArm[2]);
            this_thread::sleep_for(chrono::milliseconds(5000));


            // Start action
            float f_angle = 40;
            int theta = -180;

            CMay->CMayBody->CMayRightArm->GotoPosition(f_angle, CMay->CMayBody->CMayRightArm->TransRotate(0, 0, 0), 350, -180, 0, 10000);
            this_thread::sleep_for(chrono::milliseconds(10000));
            CMay->CMayBody->CMayRightArm->ReleaseObj(60);
            this_thread::sleep_for(chrono::milliseconds(3000));
            f_angle = 20;
            this_thread::sleep_for(chrono::milliseconds(1000));            
            CMay->CMayBody->CMayRightArm->GotoPosition(f_angle, CMay->CMayBody->CMayRightArm->TransRotate(0, 0, 0), 400, arm_sol[1]-100, 0, 5000);
            this_thread::sleep_for(chrono::milliseconds(5000));

            f_angle = 0;
            CMay->CMayBody->CMayRightArm->GotoPosition(f_angle, CMay->CMayBody->CMayRightArm->TransRotate(0, 0, 0),
                                                        arm_sol[0]-70,
                                                        arm_sol[1]-50,
                                                        arm_sol[2],
                                                        5000);
            this_thread::sleep_for(chrono::milliseconds(5000));
            CMay->CMayBody->CMayRightArm->GotoPosition(f_angle, CMay->CMayBody->CMayRightArm->TransRotate(0, 0, 0),
                                                        arm_sol[0]-20,
                                                        arm_sol[1]-50,
                                                        arm_sol[2],
                                                        5000);
            this_thread::sleep_for(chrono::milliseconds(5000));
            cout << " Catch Point 1: " << arm_sol[0] << " " << arm_sol[1] << " " << arm_sol[2] << " " << endl;
            this_thread::sleep_for(chrono::milliseconds(2000));
            CMay->CMayBody->CMayRightArm->GraspObj(2048, 100, 400);
            this_thread::sleep_for(chrono::milliseconds(2500));
            
            CMay->CMayBody->CMayRightArm->GotoPosition(f_angle, CMay->CMayBody->CMayRightArm->TransRotate(0, 0, 0),
                                                        arm_sol[0]-15,
                                                        arm_sol[1]-40,
                                                        arm_sol[2],
                                                        5000);
            this_thread::sleep_for(chrono::milliseconds(5000));    
            CMay->CMayBody->CMayRightArm->GotoPosition(f_angle, CMay->CMayBody->CMayRightArm->TransRotate(0, 0, 0),
                                                        arm_sol[0]-150,
                                                        arm_sol[1]-40,
                                                        arm_sol[2],
                                                        5000);
            this_thread::sleep_for(chrono::milliseconds(5000));
            f_angle = 40;
            cout << " Catch Point 1: " << arm_sol[0] << " " << arm_sol[1] << " " << arm_sol[2] << " " << endl;                                   
            CMay->CMayBody->CMayRightArm->GotoPosition(f_angle, CMay->CMayBody->CMayRightArm->TransRotate(0, 0, 0),
                                                        450,
                                                        arm_sol[1]-40,
                                                        arm_sol[2],
                                                        5000);
            this_thread::sleep_for(chrono::milliseconds(5000));            
            CMay->CMayBody->CMayRightArm->GotoPosition(f_angle, CMay->CMayBody->CMayRightArm->TransRotate(0, 0, 0), 450, 0, 0, 2500);
            this_thread::sleep_for(chrono::milliseconds(5000));
            f_angle = 20;
            CMay->CMayBody->CMayRightArm->GotoPosition(f_angle, CMay->CMayBody->CMayRightArm->TransRotate(0, 0, 0), 400, -180, 0, 2500);
            this_thread::sleep_for(chrono::milliseconds(5000));
            f_angle = -40;
            CMay->CMayBody->CMayRightArm->GotoPosition(f_angle, CMay->CMayBody->CMayRightArm->TransRotate(0, 0, 0), 400, -180, 0, 5000);
            this_thread::sleep_for(chrono::milliseconds(5000));
            CMay->CMayBody->CMayRightArm->GotoPosition(f_angle, CMay->CMayBody->CMayRightArm->TransRotate(0, 20, 0), 400, -180, 0, 5000);
            this_thread::sleep_for(chrono::milliseconds(5000));
            CMay->CMayBody->CMayRightArm->GotoPosition(f_angle, CMay->CMayBody->CMayRightArm->TransRotate(0, 40, 0), 400, -180, -300, 5000);
            this_thread::sleep_for(chrono::milliseconds(5000));
            CMay->CMayBody->CMayRightArm->GotoPosition(f_angle, CMay->CMayBody->CMayRightArm->TransRotate(0, 0, 0),
                                                        350,
                                                        -150,
                                                        0,
                                                        5000);
            this_thread::sleep_for(chrono::milliseconds(5000));
            //Height_Right = arm_sol[1]; // 左手夾取高度
        }
        break; //不要亂看，不要亂學
    }
    CMay->CMayVision->CYolo->result_vec.clear();


    this_thread::sleep_for(chrono::milliseconds(500));
    this->nTaskStep = TaskStep::Task_FinishStrategy;
}

void Demo_catch_YOLO::Walk_Back(void)
{
    CStrategyAssistant->vec_strStrategyStage.push_back(" Demo_catch_YOLO - Walk_Back ");
    std::cout << " Walk_Back " << std::endl;

    // Right 90 degree
    int count_turn = 0;
    int count_stop = 0;
    CMay->CMayBody->CMayMobilePlatform->TorqueEnable[0] = true;
    CMay->CMayBody->CMayMobilePlatform->TorqueEnable[1] = true;
    CMay->CMayBody->CMayMobilePlatform->Trans_Motor_Torque_Commd();
    while (count_turn < 48) //50
    {

        CMay->CMayBody->CMayMobilePlatform->Wheel_Commd_SelfRotation(50, TURN_RIGHT);
        CMay->CMayBody->CMayMobilePlatform->Trans_Motor_Commd();
        //cout << "target          " << (fAvgDeltaDistance - abs(fDistanceShouldRotate)) << endl;
        count_turn++;
        this_thread::sleep_for(chrono::milliseconds(100));
    }

    while (count_stop < 200)
    {
        CMay->CMayBody->CMayMobilePlatform->TorqueEnable[0] = false;
        CMay->CMayBody->CMayMobilePlatform->TorqueEnable[1] = false;
        CMay->CMayBody->CMayMobilePlatform->Trans_Motor_Torque_Commd();
        count_stop++;
        this_thread::sleep_for(chrono::milliseconds(10));
    }


    //向前進
    count_turn = 0;
    count_stop = 0;
    CMay->CMayBody->CMayMobilePlatform->TorqueEnable[0] = true;
    CMay->CMayBody->CMayMobilePlatform->TorqueEnable[1] = true;
    CMay->CMayBody->CMayMobilePlatform->Trans_Motor_Torque_Commd();
    while (count_turn < 150)
    {

        CMay->CMayBody->CMayMobilePlatform->Wheel_Commd_Ackermann(70, 0, FORWARD);
        CMay->CMayBody->CMayMobilePlatform->Trans_Motor_Commd();
        //cout << "target          " << (fAvgDeltaDistance - abs(fDistanceShouldRotate)) << endl;
        count_turn++;
        this_thread::sleep_for(chrono::milliseconds(100));
    }
    while (count_stop < 200)
    {
        CMay->CMayBody->CMayMobilePlatform->TorqueEnable[0] = false;
        CMay->CMayBody->CMayMobilePlatform->TorqueEnable[1] = false;
        CMay->CMayBody->CMayMobilePlatform->Trans_Motor_Torque_Commd();
        count_stop++;
        this_thread::sleep_for(chrono::milliseconds(10));
    }

    this_thread::sleep_for(chrono::milliseconds(500));
    this->nTaskStep = TaskStep::Task_FinishStrategy;
}

void Demo_catch_YOLO::FinalStrategy(void)
{
    CStrategyAssistant->vec_strStrategyStage.push_back(" Demo_catch_YOLO - FinalStrategy ");
    std::cout << " FinalStrategy " << std::endl;

    this_thread::sleep_for(chrono::milliseconds(500));
    CStrategyAssistant->bEndingStrategy = true;
    CStrategyAssistant->bRunAnotherStrategy = true; // �i�H�����L����

    this->nTaskStep = TaskStep::Task_StartStrategy;
}

void Demo_catch_YOLO::ALL_Combined(int good_number)
{
    StartStrategy(good_number);
    //Walk();
    Look_and_Catch();
    //Walk_Back();
	FinalStrategy();
}