#include "estimator.h"

#include "backend/vertex_inverse_depth.h"
#include "backend/vertex_pose.h"
#include "backend/vertex_speedbias.h"
#include "backend/edge_reprojection.h"
#include "backend/edge_imu.h"

#include <ostream>
#include <fstream>

using namespace myslam;

Estimator::Estimator() : f_manager{Rs}
{
    // ROS_INFO("init begins");
    clearState();
}

void Estimator::setParameter()
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = TIC[i];
        ric[i] = RIC[i];
        // cout << "1 Estimator::setParameter tic: " << tic[i].transpose()
        //     << " ric: " << ric[i] << endl;
    }
    cout << "1 Estimator::setParameter FOCAL_LENGTH: " << FOCAL_LENGTH << endl;
    f_manager.setRic(ric);
    project_sqrt_info_ = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    td = TD;
}

void Estimator::clearState()
{
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();

        if (pre_integrations[i] != nullptr)
            delete pre_integrations[i];
        pre_integrations[i] = nullptr;
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d::Zero();
        ric[i] = Matrix3d::Identity();
    }

    for (auto &it : all_image_frame)
    {
        if (it.second.pre_integration != nullptr)
        {
            delete it.second.pre_integration;
            it.second.pre_integration = nullptr;
        }
    }

    solver_flag = INITIAL;
    first_imu = false,
    sum_of_back = 0;
    sum_of_front = 0;
    frame_count = 0;
    solver_flag = INITIAL;
    initial_timestamp = 0;
    all_image_frame.clear();
    td = TD;

    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;

    tmp_pre_integration = nullptr;

    last_marginalization_parameter_blocks.clear();

    f_manager.clearState();

    failure_occur = 0;
    relocalization_info = 0;

    drift_correct_r = Matrix3d::Identity();
    drift_correct_t = Vector3d::Zero();
}

//对于一组cam前面的一组imu的每一帧数据的处理
/*
 *　设定当前frame_count的 pre_integrations对象指针
    进行预积分计算　tmp_pre_integration　和当前　pre_integrations[frame_count]是一样的,这个数据会绑定到每一帧上

    下面再进行积分
    初始化之前没有什么用，当初始化之后，frame_count=WINDOWS_SIZE, 这个就会用积分来估计body系下的每一帧的P R V
 */
void Estimator::processIMU(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }
    //如果当前imu测量还没有为预积分贡献一次累加，那么就添加一次frame_count的pre_integrations*
    if (!pre_integrations[frame_count])
    {
        //指向一个预积分模型的类的实例化对象
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }
    //因为frame_count=0,被固定，前面不需要有对应的imu值
    // std::cout << "current frame_count: "<< frame_count << std::endl;
    if (frame_count != 0)
    {
        //预积分增量对于每一包imu都是从零开始积分，只记录两个imu之间的增量
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity); //根据当前帧进行中值积分，并更新jacobian和协方差
        //if(solver_flag != NON_LINEAR)
        tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        // frame_count里面不存imu数据，后面每一帧frame_count会存对应的imu数据
        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        //这一部分是积分，把增量更新在状态量之上
        //frame_count等于０～９还体现不出来，初始化之后会填充一组初值
        //当frame_count=WINDOWS_SIZE时，这个就会用积分来估计body系下的每一帧的P R V
        int j = frame_count;
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc; //预积分计算的 对应每一个frameconut的pvq
        Vs[j] += dt * un_acc;
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
    // std::cout << "processIMU over;"<<endl;
    //  cout << "current_frame_count imu buffer size: "<< linear_acceleration_buf[frame_count].size() << endl;

    //处理完一帧frame_count预积分增量　以及　叠加P R V的积分量
    //  printf("frame_count: %d P: %f %f %f \n",frame_count, pre_integrations[frame_count]->delta_p.x(), pre_integrations[frame_count]->delta_p.y(), pre_integrations[frame_count]->delta_p.z());
    //  printf("frame_count: %d P: %f %f %f \n",frame_count, Ps[frame_count].x(), Ps[frame_count].y(), Ps[frame_count].z());
}

/*
 * processImage 输入一帧图像的若干特征点
 * addFeatureCheckParallax:两个功能，把当前帧的特征点更新到feature list中，根据特征点的新旧比以及视差，设定关键帧和边缘化策略
 * ImageFrame imageframe(image, header);　all_image_frame.insert(make_pair(header, imageframe));把当前帧更新到all_image_frame中
 */
void Estimator::processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double header)
{
    //ROS_DEBUG("new image coming ------------------------------------------");
    // cout << "Adding feature points: " << image.size() << endl;
    if (f_manager.addFeatureCheckParallax(frame_count, image, td))
        marginalization_flag = MARGIN_OLD; //当前追踪的特征点太少，或者last second last third 视差较大
    else
        marginalization_flag = MARGIN_SECOND_NEW; //如果视差太小就把　last second 抹掉


    cout << "this frame is--------------------" << (marginalization_flag ? "reject" : "accept") << endl;
    //ROS_DEBUG("this frame is--------------------%s", marginalization_flag ? "reject" : "accept");
    //ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    //ROS_DEBUG("Solving %d", frame_count);
    // cout << "number of feature: " << f_manager.getFeatureCount()<<endl;
    Headers[frame_count] = header;

    ImageFrame imageframe(image, header);
    imageframe.pre_integration = tmp_pre_integration;
    all_image_frame.insert(make_pair(header, imageframe));
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]}; //更新下一个预积分增量

    //在线校准cam与imu外参，只在系统运行之初
    //利用两帧视觉帧构建对极几何约束估计Rc1c2,两帧之间的预积分增量计算相对应的Rb1b2,然后通过两者之间的约束构建方程组求解Ｒbc
    if (ESTIMATE_EXTRINSIC == 2)
    {
        cout << "calibrating extrinsic param, rotation movement is needed" << endl;
        if (frame_count != 0)
        {
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
            Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
            {
                // ROS_WARN("initial extrinsic rotation calib success");
                // ROS_WARN_STREAM("initial extrinsic rotation: " << endl
                //    << calib_ric);
                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                ESTIMATE_EXTRINSIC = 1;
                cout << "initial extrinsic rotation calib success!" << endl;
            }
        }
    }

    //系统初始化过程
    /*
    * 
     */
    if (solver_flag == INITIAL)
    {
        if (frame_count == WINDOW_SIZE)
        {
            bool result = false;
            if (ESTIMATE_EXTRINSIC != 2 && (header - initial_timestamp) > 0.1)
            {
                // cout << "1 initialStructure" << endl;
                result = initialStructure();
                initial_timestamp = header;
            }
            if (result)
            {
                solver_flag = NON_LINEAR;
                solveOdometry();
                slideWindow();
                f_manager.removeFailures();
                cout << "Initialization finish!" << endl;
                last_R = Rs[WINDOW_SIZE];
                last_P = Ps[WINDOW_SIZE];
                last_R0 = Rs[0];
                last_P0 = Ps[0];
            }
            else
                slideWindow();
        }
        else
            frame_count++;
    }
    //系统初始化只有　优化求解过程
    else
    {
        TicToc t_solve;
        solveOdometry();
        //ROS_DEBUG("solver costs: %fms", t_solve.toc());

        if (failureDetection())
        {
            // ROS_WARN("failure detection!");
            failure_occur = 1;
            clearState();
            setParameter();
            // ROS_WARN("system reboot!");
            return;
        }

        TicToc t_margin;
        slideWindow();
        f_manager.removeFailures();
        //ROS_DEBUG("marginalization costs: %fms", t_margin.toc());
        // prepare output of VINS
        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i++)
            key_poses.push_back(Ps[i]);

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
    }
}

/*
 * 单目＋IMU初始化过程
 * －　先根据IMU预积分数据，计算imu的运动幅度，运动幅度太小，无法进行后续初始化
 * －　
 */
bool Estimator::initialStructure()
{
    TicToc t_sfm;
    cout << "Initialization: all image frame size: " << all_image_frame.size() << endl;
    //check imu observability
    {
        cout << "Initialization: check imu observability " << endl;
        map<double, ImageFrame>::iterator frame_it;
        Vector3d sum_g;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            sum_g += tmp_g;
        }
        //计算均值，all_image_frame在三个轴向平均速度
        Vector3d aver_g;
        aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
        //平均方差的计算，初略估计运动的幅度
        double var = 0;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
            //cout << "frame g " << tmp_g.transpose() << endl;
        }
        var = sqrt(var / ((int)all_image_frame.size() - 1));
        //ROS_WARN("IMU variation %f!", var);
        if (var < 0.25) //imu激励不够，幅度再大一点
        {
            // ROS_INFO("IMU excitation not enough!");
            cout << "IMU excitation not enough" << endl;
            return false;
        }
    }
    // global SFM
    Quaterniond Q[frame_count + 1];
    Vector3d T[frame_count + 1];
    map<int, Vector3d> sfm_tracked_points;
    vector<SFMFeature> sfm_f; //管理 sfm的特征点，也就是到目前为止的feature list中的点，属性稍微不同
    cout << "Initialization: global SFM " << endl;
    for (auto &it_per_id : f_manager.feature)
    {
        int imu_j = it_per_id.start_frame - 1;
        SFMFeature tmp_feature;
        tmp_feature.state = false;
        tmp_feature.id = it_per_id.feature_id;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            Vector3d pts_j = it_per_frame.point;
            //从start_frame开始，push特征点在每一帧归一化相机坐标
            tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
        }
        sfm_f.push_back(tmp_feature);
    }
    cout << "Initialization: all features:" << f_manager.feature.size() << " sfm_feature: " << sfm_f.size() << endl;
    Matrix3d relative_R;
    Vector3d relative_T;
    int l;
    //在窗口中找到一个与frame_count帧视差较大的帧计算视差，并根据这两帧的特征点对计算R T
    if (!relativePose(relative_R, relative_T, l))
    {
        cout << "Not enough features or parallax; Move device around" << endl;
        return false;
    }
    cout << "Initialization: according  parallax between " << l << " with newest frame,5point compute R&t  " << endl;
    GlobalSFM sfm;
    if (!sfm.construct(frame_count + 1, Q, T, l,
                       relative_R, relative_T,
                       sfm_f, sfm_tracked_points))
    {
        cout << "global SFM failed!" << endl;
        marginalization_flag = MARGIN_OLD;
        return false;
    }

    //solve pnp for all frame
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator it;
    frame_it = all_image_frame.begin();
    for (int i = 0; frame_it != all_image_frame.end(); frame_it++)
    {
        // provide initial guess
        cout << "Initialization: solve pnp for all frame. frame_it: " << frame_it->first << "WINDOW_it: " << Headers[i] << endl;
        cv::Mat r, rvec, t, D, tmp_r;
        if ((frame_it->first) == Headers[i])
        {
            frame_it->second.is_key_frame = true;
            frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose();
            frame_it->second.T = T[i];
            i++;
            continue;
        }
        if ((frame_it->first) > Headers[i])
        {
            i++;
        }
        Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Vector3d P_inital = -R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, t);

        frame_it->second.is_key_frame = false;
        vector<cv::Point3f> pts_3_vector;
        vector<cv::Point2f> pts_2_vector;
        for (auto &id_pts : frame_it->second.points)
        {
            int feature_id = id_pts.first;
            for (auto &i_p : id_pts.second)
            {
                it = sfm_tracked_points.find(feature_id);
                if (it != sfm_tracked_points.end())
                {
                    Vector3d world_pts = it->second;
                    cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                    pts_3_vector.push_back(pts_3);
                    Vector2d img_pts = i_p.second.head<2>();
                    cv::Point2f pts_2(img_pts(0), img_pts(1));
                    pts_2_vector.push_back(pts_2);
                }
            }
        }
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
        if (pts_3_vector.size() < 6)
        {
            cout << "Not enough points for solve pnp pts_3_vector size " << pts_3_vector.size() << endl;
            return false;
        }
        if (!cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))
        {
            cout << " solve pnp fail!" << endl;
            return false;
        }
        cv::Rodrigues(rvec, r);
        MatrixXd R_pnp, tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        R_pnp = tmp_R_pnp.transpose();
        MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);
        T_pnp = R_pnp * (-T_pnp);
        frame_it->second.R = R_pnp * RIC[0].transpose();
        frame_it->second.T = T_pnp;
    }
    if (visualInitialAlign())
        return true;
    else
    {
        cout << "misalign visual structure with IMU" << endl;
        return false;
    }
    return true;
}

bool Estimator::visualInitialAlign()
{
    TicToc t_g;
    VectorXd x;
    //solve scale
    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
    if (!result)
    {
        //ROS_DEBUG("solve g failed!");
        return false;
    }

    // change state
    for (int i = 0; i <= frame_count; i++)
    {
        Matrix3d Ri = all_image_frame[Headers[i]].R;
        Vector3d Pi = all_image_frame[Headers[i]].T;
        Ps[i] = Pi;
        Rs[i] = Ri;
        all_image_frame[Headers[i]].is_key_frame = true;
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < dep.size(); i++)
        dep[i] = -1;
    f_manager.clearDepth(dep);

    //triangulat on cam pose , no tic
    Vector3d TIC_TMP[NUM_OF_CAM];
    for (int i = 0; i < NUM_OF_CAM; i++)
        TIC_TMP[i].setZero();
    ric[0] = RIC[0];
    f_manager.setRic(ric);
    f_manager.triangulate(Ps, &(TIC_TMP[0]), &(RIC[0]));

    double s = (x.tail<1>())(0);
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }
    for (int i = frame_count; i >= 0; i--)
        Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);
    int kv = -1;
    map<double, ImageFrame>::iterator frame_i;
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
    {
        if (frame_i->second.is_key_frame)
        {
            kv++;
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
        }
    }
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        it_per_id.estimated_depth *= s;
    }

    Matrix3d R0 = Utility::g2R(g);
    double yaw = Utility::R2ypr(R0 * Rs[0]).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    g = R0 * g;
    //Matrix3d rot_diff = R0 * Rs[0].transpose();
    Matrix3d rot_diff = R0;
    for (int i = 0; i <= frame_count; i++)
    {
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
    }
    //ROS_DEBUG_STREAM("g0     " << g.transpose());
    //ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(Rs[0]).transpose());

    return true;
}

/*
 * 窗口里面的每一帧都与最新帧计算视差，
 * 如果视差还算比较大，就计算这一帧与当前帧的的相对Ｒ t
 */
bool Estimator::relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l)
{
    // find previous frame which contians enough correspondance and parallex with newest frame
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        vector<pair<Vector3d, Vector3d>> corres;
        corres = f_manager.getCorresponding(i, WINDOW_SIZE);
        if (corres.size() > 20)
        {
            cout << "corres: " << corres.size() << endl;
            double sum_parallax = 0;
            double average_parallax;
            for (int j = 0; j < int(corres.size()); j++)
            {
                //共视点对
                Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax = sum_parallax + parallax;
            }
            average_parallax = 1.0 * sum_parallax / int(corres.size());
            if (average_parallax * 460 > 30 && m_estimator.solveRelativeRT(corres, relative_R, relative_T))
            {
                l = i;
                //ROS_DEBUG("average_parallax %f choose l %d and newest frame to triangulate the whole structure", average_parallax * 460, l);
                return true;
            }
            cout << "average parallax: " << average_parallax*460 << endl;
        }
    }
    return false;
}

void Estimator::solveOdometry()
{
    if (frame_count < WINDOW_SIZE)
        return;
    if (solver_flag == NON_LINEAR)
    {
        TicToc t_tri;
        f_manager.triangulate(Ps, tic, ric);
        //cout << "triangulation costs : " << t_tri.toc() << endl;
        backendOptimization();
    }
}

/*
 * 把待优化变量组合成向量
 * - 对窗口中每一个cam frame: 总维度　7+9  pose的位置＋四元素　SoeedBias的速度、ba bg分量
 * - 对相机的外参：7维　单目imu就有一个pose和四元素，双目就有两组
 * - 对窗口中的所有landmark 特征点的数量
 * - 如果需要优化imu与cam的时间间隔　td
 * - 
 */
void Estimator::vector2double()
{
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        para_SpeedBias[i][0] = Vs[i].x();
        para_SpeedBias[i][1] = Vs[i].y();
        para_SpeedBias[i][2] = Vs[i].z();

        para_SpeedBias[i][3] = Bas[i].x();
        para_SpeedBias[i][4] = Bas[i].y();
        para_SpeedBias[i][5] = Bas[i].z();

        para_SpeedBias[i][6] = Bgs[i].x();
        para_SpeedBias[i][7] = Bgs[i].y();
        para_SpeedBias[i][8] = Bgs[i].z();
    }
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        para_Ex_Pose[i][0] = tic[i].x();
        para_Ex_Pose[i][1] = tic[i].y();
        para_Ex_Pose[i][2] = tic[i].z();
        Quaterniond q{ric[i]};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int j = 0; j < f_manager.getFeatureCount(); j++)
        para_Feature[j][0] = dep(j); //为什么要把特征点
    if (ESTIMATE_TD)
        para_Td[0][0] = td;
}

/*
 * 把优化的量重新更新到状态量里面去
 */
void Estimator::double2vector()
{
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
    Vector3d origin_P0 = Ps[0];

    if (failure_occur)
    {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = 0;
    }
    Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                     para_Pose[0][3],
                                                     para_Pose[0][4],
                                                     para_Pose[0][5])
                                             .toRotationMatrix());
    double y_diff = origin_R0.x() - origin_R00.x();
    //TODO 航向的偏移
    Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
    if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0)
    {
        //ROS_DEBUG("euler singular point!");
        rot_diff = Rs[0] * Quaterniond(para_Pose[0][6],
                                       para_Pose[0][3],
                                       para_Pose[0][4],
                                       para_Pose[0][5])
                               .toRotationMatrix()
                               .transpose();
    }

    for (int i = 0; i <= WINDOW_SIZE; i++)
    {

        Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();

        Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                    para_Pose[i][1] - para_Pose[0][1],
                                    para_Pose[i][2] - para_Pose[0][2]) +
                origin_P0;

        Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
                                    para_SpeedBias[i][1],
                                    para_SpeedBias[i][2]);

        Bas[i] = Vector3d(para_SpeedBias[i][3],
                          para_SpeedBias[i][4],
                          para_SpeedBias[i][5]);

        Bgs[i] = Vector3d(para_SpeedBias[i][6],
                          para_SpeedBias[i][7],
                          para_SpeedBias[i][8]);
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d(para_Ex_Pose[i][0],
                          para_Ex_Pose[i][1],
                          para_Ex_Pose[i][2]);
        ric[i] = Quaterniond(para_Ex_Pose[i][6],
                             para_Ex_Pose[i][3],
                             para_Ex_Pose[i][4],
                             para_Ex_Pose[i][5])
                     .toRotationMatrix();
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        dep(i) = para_Feature[i][0];

    f_manager.setDepth(dep);
    if (ESTIMATE_TD)
        td = para_Td[0][0];

    // relative info between two loop frame
    if (relocalization_info)
    {
        Matrix3d relo_r;
        Vector3d relo_t;
        relo_r = rot_diff * Quaterniond(relo_Pose[6], relo_Pose[3], relo_Pose[4], relo_Pose[5]).normalized().toRotationMatrix();
        relo_t = rot_diff * Vector3d(relo_Pose[0] - para_Pose[0][0],
                                     relo_Pose[1] - para_Pose[0][1],
                                     relo_Pose[2] - para_Pose[0][2]) +
                 origin_P0;
        double drift_correct_yaw;
        drift_correct_yaw = Utility::R2ypr(prev_relo_r).x() - Utility::R2ypr(relo_r).x();
        drift_correct_r = Utility::ypr2R(Vector3d(drift_correct_yaw, 0, 0));
        drift_correct_t = prev_relo_t - drift_correct_r * relo_t;
        relo_relative_t = relo_r.transpose() * (Ps[relo_frame_local_index] - relo_t);
        relo_relative_q = relo_r.transpose() * Rs[relo_frame_local_index];
        relo_relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs[relo_frame_local_index]).x() - Utility::R2ypr(relo_r).x());
        //cout << "vins relo " << endl;
        //cout << "vins relative_t " << relo_relative_t.transpose() << endl;
        //cout << "vins relative_yaw " <<relo_relative_yaw << endl;
        relocalization_info = 0;
    }
}

/*
 * 各种　failure的情况
 * －　last_track_num < 2
 * －　Bas[WINDOW_SIZE].norm() > 2.5　bias太大了
 * －　Bgs[WINDOW_SIZE].norm() > 1.0
 * －　(tmp_P - last_P).norm() > 5　 两帧之间的移动距离有点离谱
 * －　tmp_P.z() - last_P.z()) > 1　 z的运动太大了
 */
bool Estimator::failureDetection()
{
    //当前帧与上一帧连续追踪的特征点太少了，比如说出现了遮挡
    if (f_manager.last_track_num < 2)
    {
        //ROS_INFO(" little feature %d", f_manager.last_track_num);
        //return true;
    }
    if (Bas[WINDOW_SIZE].norm() > 2.5)
    {
        //ROS_INFO(" big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
        return true;
    }
    if (Bgs[WINDOW_SIZE].norm() > 1.0)
    {
        //ROS_INFO(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
        return true;
    }
    /*
    if (tic(0) > 1)
    {
        //ROS_INFO(" big extri param estimation %d", tic(0) > 1);
        return true;
    }
    */
    Vector3d tmp_P = Ps[WINDOW_SIZE];
    if ((tmp_P - last_P).norm() > 5)
    {
        //ROS_INFO(" big translation");
        return true;
    }
    if (abs(tmp_P.z() - last_P.z()) > 1)
    {
        //ROS_INFO(" big z translation");
        return true;
    }
    Matrix3d tmp_R = Rs[WINDOW_SIZE];
    Matrix3d delta_R = tmp_R.transpose() * last_R;
    Quaterniond delta_Q(delta_R);
    double delta_angle;
    delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
    if (delta_angle > 50)
    {
        //ROS_INFO(" big delta_angle ");
        //return true;
    }
    return false;
}

/*
 * marge掉最老帧
 * 
 */
void Estimator::MargOldFrame()
{
    backend::LossFunction *lossfunction;
    lossfunction = new backend::CauchyLoss(1.0);

    // step1. 构建 problem
    backend::Problem problem(backend::Problem::ProblemType::SLAM_PROBLEM);
    vector<shared_ptr<backend::VertexPose>> vertexCams_vec;
    vector<shared_ptr<backend::VertexSpeedBias>> vertexVB_vec;
    int pose_dim = 0;

    // 先把 外参数 节点加入图优化，这个节点在以后一直会被用到，所以我们把他放在第一个
    shared_ptr<backend::VertexPose> vertexExt(new backend::VertexPose());
    {
        Eigen::VectorXd pose(7);
        pose << para_Ex_Pose[0][0], para_Ex_Pose[0][1], para_Ex_Pose[0][2], para_Ex_Pose[0][3], para_Ex_Pose[0][4], para_Ex_Pose[0][5], para_Ex_Pose[0][6];
        vertexExt->SetParameters(pose);
        problem.AddVertex(vertexExt);
        pose_dim += vertexExt->LocalDimension();
    }

    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        shared_ptr<backend::VertexPose> vertexCam(new backend::VertexPose());
        Eigen::VectorXd pose(7);
        pose << para_Pose[i][0], para_Pose[i][1], para_Pose[i][2], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5], para_Pose[i][6];
        vertexCam->SetParameters(pose);
        vertexCams_vec.push_back(vertexCam);
        problem.AddVertex(vertexCam);
        pose_dim += vertexCam->LocalDimension();

        shared_ptr<backend::VertexSpeedBias> vertexVB(new backend::VertexSpeedBias());
        Eigen::VectorXd vb(9);
        vb << para_SpeedBias[i][0], para_SpeedBias[i][1], para_SpeedBias[i][2],
            para_SpeedBias[i][3], para_SpeedBias[i][4], para_SpeedBias[i][5],
            para_SpeedBias[i][6], para_SpeedBias[i][7], para_SpeedBias[i][8];
        vertexVB->SetParameters(vb);
        vertexVB_vec.push_back(vertexVB);
        problem.AddVertex(vertexVB);
        pose_dim += vertexVB->LocalDimension();
    }

    // IMU
    {
        if (pre_integrations[1]->sum_dt < 10.0)
        {
            std::shared_ptr<backend::EdgeImu> imuEdge(new backend::EdgeImu(pre_integrations[1]));
            std::vector<std::shared_ptr<backend::Vertex>> edge_vertex;
            edge_vertex.push_back(vertexCams_vec[0]);
            edge_vertex.push_back(vertexVB_vec[0]);
            edge_vertex.push_back(vertexCams_vec[1]);
            edge_vertex.push_back(vertexVB_vec[1]);
            imuEdge->SetVertex(edge_vertex);
            problem.AddEdge(imuEdge);
        }
    }

    // Visual Factor
    {
        int feature_index = -1;
        // 遍历每一个特征
        for (auto &it_per_id : f_manager.feature)
        {
            it_per_id.used_num = it_per_id.feature_per_frame.size();
            if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                continue;

            ++feature_index;

            int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
            if (imu_i != 0)
                continue;

            Vector3d pts_i = it_per_id.feature_per_frame[0].point;

            shared_ptr<backend::VertexInverseDepth> verterxPoint(new backend::VertexInverseDepth());
            VecX inv_d(1);
            inv_d << para_Feature[feature_index][0];
            verterxPoint->SetParameters(inv_d);
            problem.AddVertex(verterxPoint);

            // 遍历所有的观测
            for (auto &it_per_frame : it_per_id.feature_per_frame)
            {
                imu_j++;
                if (imu_i == imu_j)
                    continue;

                Vector3d pts_j = it_per_frame.point;

                std::shared_ptr<backend::EdgeReprojection> edge(new backend::EdgeReprojection(pts_i, pts_j));
                std::vector<std::shared_ptr<backend::Vertex>> edge_vertex;
                edge_vertex.push_back(verterxPoint);
                edge_vertex.push_back(vertexCams_vec[imu_i]);
                edge_vertex.push_back(vertexCams_vec[imu_j]);
                edge_vertex.push_back(vertexExt);

                edge->SetVertex(edge_vertex);
                edge->SetInformation(project_sqrt_info_.transpose() * project_sqrt_info_);

                edge->SetLossFunction(lossfunction);
                problem.AddEdge(edge);
            }
        }
    }

    // 先验
    {
        // 已经有 Prior 了
        if (Hprior_.rows() > 0)
        {
            problem.SetHessianPrior(Hprior_); // 告诉这个 problem
            problem.SetbPrior(bprior_);
            problem.SetErrPrior(errprior_);
            problem.SetJtPrior(Jprior_inv_);
            problem.ExtendHessiansPriorSize(15); // 但是这个 prior 还是之前的维度，需要扩展下装新的pose
        }
        else
        {
            Hprior_ = MatXX(pose_dim, pose_dim);
            Hprior_.setZero();
            bprior_ = VecX(pose_dim);
            bprior_.setZero();
            problem.SetHessianPrior(Hprior_); // 告诉这个 problem
            problem.SetbPrior(bprior_);
        }
    }

    std::vector<std::shared_ptr<backend::Vertex>> marg_vertex;
    marg_vertex.push_back(vertexCams_vec[0]);
    marg_vertex.push_back(vertexVB_vec[0]);
    problem.Marginalize(marg_vertex, pose_dim);
    Hprior_ = problem.GetHessianPrior();
    bprior_ = problem.GetbPrior();
    errprior_ = problem.GetErrPrior();
    Jprior_inv_ = problem.GetJtPrior();
}
void Estimator::MargNewFrame()
{

    // step1. 构建 problem
    backend::Problem problem(backend::Problem::ProblemType::SLAM_PROBLEM);
    vector<shared_ptr<backend::VertexPose>> vertexCams_vec;
    vector<shared_ptr<backend::VertexSpeedBias>> vertexVB_vec;
    //    vector<backend::Point3d> points;
    int pose_dim = 0;

    // 先把 外参数 节点加入图优化，这个节点在以后一直会被用到，所以我们把他放在第一个
    shared_ptr<backend::VertexPose> vertexExt(new backend::VertexPose());
    {
        Eigen::VectorXd pose(7);
        pose << para_Ex_Pose[0][0], para_Ex_Pose[0][1], para_Ex_Pose[0][2], para_Ex_Pose[0][3], para_Ex_Pose[0][4], para_Ex_Pose[0][5], para_Ex_Pose[0][6];
        vertexExt->SetParameters(pose);
        problem.AddVertex(vertexExt);
        pose_dim += vertexExt->LocalDimension();
    }

    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        shared_ptr<backend::VertexPose> vertexCam(new backend::VertexPose());
        Eigen::VectorXd pose(7);
        pose << para_Pose[i][0], para_Pose[i][1], para_Pose[i][2], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5], para_Pose[i][6];
        vertexCam->SetParameters(pose);
        vertexCams_vec.push_back(vertexCam);
        problem.AddVertex(vertexCam);
        pose_dim += vertexCam->LocalDimension();

        shared_ptr<backend::VertexSpeedBias> vertexVB(new backend::VertexSpeedBias());
        Eigen::VectorXd vb(9);
        vb << para_SpeedBias[i][0], para_SpeedBias[i][1], para_SpeedBias[i][2],
            para_SpeedBias[i][3], para_SpeedBias[i][4], para_SpeedBias[i][5],
            para_SpeedBias[i][6], para_SpeedBias[i][7], para_SpeedBias[i][8];
        vertexVB->SetParameters(vb);
        vertexVB_vec.push_back(vertexVB);
        problem.AddVertex(vertexVB);
        pose_dim += vertexVB->LocalDimension();
    }

    // 先验
    {
        // 已经有 Prior 了
        if (Hprior_.rows() > 0)
        {
            problem.SetHessianPrior(Hprior_); // 告诉这个 problem
            problem.SetbPrior(bprior_);
            problem.SetErrPrior(errprior_);
            problem.SetJtPrior(Jprior_inv_);

            problem.ExtendHessiansPriorSize(15); // 但是这个 prior 还是之前的维度，需要扩展下装新的pose
        }
        else
        {
            Hprior_ = MatXX(pose_dim, pose_dim);
            Hprior_.setZero();
            bprior_ = VecX(pose_dim);
            bprior_.setZero();
        }
    }

    std::vector<std::shared_ptr<backend::Vertex>> marg_vertex;
    // 把窗口倒数第二个帧 marg 掉
    marg_vertex.push_back(vertexCams_vec[WINDOW_SIZE - 1]);
    marg_vertex.push_back(vertexVB_vec[WINDOW_SIZE - 1]);
    problem.Marginalize(marg_vertex, pose_dim);
    Hprior_ = problem.GetHessianPrior();
    bprior_ = problem.GetbPrior();
    errprior_ = problem.GetErrPrior();
    Jprior_inv_ = problem.GetJtPrior();
}
void Estimator::problemSolve()
{
    backend::LossFunction *lossfunction;
    lossfunction = new backend::CauchyLoss(1.0);
    //    lossfunction = new backend::TukeyLoss(1.0);

    // step1. 构建 problem
    backend::Problem problem(backend::Problem::ProblemType::SLAM_PROBLEM);
    vector<shared_ptr<backend::VertexPose>> vertexCams_vec;
    vector<shared_ptr<backend::VertexSpeedBias>> vertexVB_vec;
    int pose_dim = 0;

    // 先把 外参数 节点加入图优化，这个节点在以后一直会被用到，所以我们把他放在第一个
    shared_ptr<backend::VertexPose> vertexExt(new backend::VertexPose());
    {
        Eigen::VectorXd pose(7);
        pose << para_Ex_Pose[0][0], para_Ex_Pose[0][1], para_Ex_Pose[0][2], para_Ex_Pose[0][3], para_Ex_Pose[0][4], para_Ex_Pose[0][5], para_Ex_Pose[0][6];
        vertexExt->SetParameters(pose);

        if (!ESTIMATE_EXTRINSIC)
        {
            //ROS_DEBUG("fix extinsic param");
            // TODO:: set Hessian prior to zero
            vertexExt->SetFixed();
        }
        else
        {
            //ROS_DEBUG("estimate extinsic param");
        }
        problem.AddVertex(vertexExt);
        pose_dim += vertexExt->LocalDimension();
    }

    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        shared_ptr<backend::VertexPose> vertexCam(new backend::VertexPose());
        Eigen::VectorXd pose(7);
        pose << para_Pose[i][0], para_Pose[i][1], para_Pose[i][2], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5], para_Pose[i][6];
        vertexCam->SetParameters(pose);
        vertexCams_vec.push_back(vertexCam);
        problem.AddVertex(vertexCam);
        pose_dim += vertexCam->LocalDimension();

        shared_ptr<backend::VertexSpeedBias> vertexVB(new backend::VertexSpeedBias());
        Eigen::VectorXd vb(9);
        vb << para_SpeedBias[i][0], para_SpeedBias[i][1], para_SpeedBias[i][2],
            para_SpeedBias[i][3], para_SpeedBias[i][4], para_SpeedBias[i][5],
            para_SpeedBias[i][6], para_SpeedBias[i][7], para_SpeedBias[i][8];
        vertexVB->SetParameters(vb);
        vertexVB_vec.push_back(vertexVB);
        problem.AddVertex(vertexVB);
        pose_dim += vertexVB->LocalDimension();
    }

    // IMU
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        int j = i + 1;
        if (pre_integrations[j]->sum_dt > 10.0)
            continue;

        std::shared_ptr<backend::EdgeImu> imuEdge(new backend::EdgeImu(pre_integrations[j]));
        std::vector<std::shared_ptr<backend::Vertex>> edge_vertex;
        edge_vertex.push_back(vertexCams_vec[i]);
        edge_vertex.push_back(vertexVB_vec[i]);
        edge_vertex.push_back(vertexCams_vec[j]);
        edge_vertex.push_back(vertexVB_vec[j]);
        imuEdge->SetVertex(edge_vertex);
        problem.AddEdge(imuEdge);
    }

    // Visual Factor
    vector<shared_ptr<backend::VertexInverseDepth>> vertexPt_vec;
    {
        int feature_index = -1;
        // 遍历每一个特征
        for (auto &it_per_id : f_manager.feature)
        {
            it_per_id.used_num = it_per_id.feature_per_frame.size();
            if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                continue;

            ++feature_index;

            int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
            Vector3d pts_i = it_per_id.feature_per_frame[0].point;

            shared_ptr<backend::VertexInverseDepth> verterxPoint(new backend::VertexInverseDepth());
            VecX inv_d(1);
            inv_d << para_Feature[feature_index][0];
            verterxPoint->SetParameters(inv_d);
            problem.AddVertex(verterxPoint);
            vertexPt_vec.push_back(verterxPoint);

            // 遍历所有的观测
            for (auto &it_per_frame : it_per_id.feature_per_frame)
            {
                imu_j++;
                if (imu_i == imu_j)
                    continue;

                Vector3d pts_j = it_per_frame.point;

                std::shared_ptr<backend::EdgeReprojection> edge(new backend::EdgeReprojection(pts_i, pts_j));
                std::vector<std::shared_ptr<backend::Vertex>> edge_vertex;
                edge_vertex.push_back(verterxPoint);
                edge_vertex.push_back(vertexCams_vec[imu_i]);
                edge_vertex.push_back(vertexCams_vec[imu_j]);
                edge_vertex.push_back(vertexExt);

                edge->SetVertex(edge_vertex);
                edge->SetInformation(project_sqrt_info_.transpose() * project_sqrt_info_);

                edge->SetLossFunction(lossfunction);
                problem.AddEdge(edge);
            }
        }
    }

    // 先验
    {
        // 已经有 Prior 了
        if (Hprior_.rows() > 0)
        {
            // 外参数先验设置为 0. TODO:: 这个应该放到 solver 里去弄
            //            Hprior_.block(0,0,6,Hprior_.cols()).setZero();
            //            Hprior_.block(0,0,Hprior_.rows(),6).setZero();

            problem.SetHessianPrior(Hprior_); // 告诉这个 problem
            problem.SetbPrior(bprior_);
            problem.SetErrPrior(errprior_);
            problem.SetJtPrior(Jprior_inv_);
            problem.ExtendHessiansPriorSize(15); // 但是这个 prior 还是之前的维度，需要扩展下装新的pose
        }
    }

    problem.Solve(10);

    // update bprior_,  Hprior_ do not need update
    if (Hprior_.rows() > 0)
    {
        std::cout << "----------- update bprior -------------\n";
        std::cout << "             before: " << bprior_.norm() << std::endl;
        std::cout << "                     " << errprior_.norm() << std::endl;
        bprior_ = problem.GetbPrior();
        errprior_ = problem.GetErrPrior();
        std::cout << "             after: " << bprior_.norm() << std::endl;
        std::cout << "                    " << errprior_.norm() << std::endl;
    }

    // update parameter
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        VecX p = vertexCams_vec[i]->Parameters();
        for (int j = 0; j < 7; ++j)
        {
            para_Pose[i][j] = p[j];
        }

        VecX vb = vertexVB_vec[i]->Parameters();
        for (int j = 0; j < 9; ++j)
        {
            para_SpeedBias[i][j] = vb[j];
        }
    }

    // 遍历每一个特征
    for (size_t i = 0; i < vertexPt_vec.size(); ++i)
    {
        VecX f = vertexPt_vec[i]->Parameters();
        para_Feature[i][0] = f[0];
    }
}

void Estimator::backendOptimization()
{
    TicToc t_solver;
    // 借助 vins 框架，维护变量
    vector2double();
    // 构建求解器
    problemSolve();
    // 优化后的变量处理下自由度
    double2vector();
    //ROS_INFO("whole time for solver: %f", t_solver.toc());

    // 维护 marg
    TicToc t_whole_marginalization;
    if (marginalization_flag == MARGIN_OLD)
    {
        vector2double();

        MargOldFrame();

        std::unordered_map<long, double *> addr_shift; // prior 中对应的保留下来的参数地址
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
        if (ESTIMATE_TD)
        {
            addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
        }
    }
    else
    {
        if (Hprior_.rows() > 0)
        {

            vector2double();

            MargNewFrame();

            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
            if (ESTIMATE_TD)
            {
                addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
            }
        }
    }
}

void Estimator::slideWindow()
{
    TicToc t_margin;
    if (marginalization_flag == MARGIN_OLD)
    {
        double t_0 = Headers[0];
        back_R0 = Rs[0];
        back_P0 = Ps[0];
        if (frame_count == WINDOW_SIZE)
        {
            for (int i = 0; i < WINDOW_SIZE; i++)
            {
                Rs[i].swap(Rs[i + 1]);

                std::swap(pre_integrations[i], pre_integrations[i + 1]);

                dt_buf[i].swap(dt_buf[i + 1]);
                linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

                Headers[i] = Headers[i + 1];
                Ps[i].swap(Ps[i + 1]);
                Vs[i].swap(Vs[i + 1]);
                Bas[i].swap(Bas[i + 1]);
                Bgs[i].swap(Bgs[i + 1]);
            }
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];
            Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
            Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

            delete pre_integrations[WINDOW_SIZE];
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();

            if (true || solver_flag == INITIAL)
            {
                map<double, ImageFrame>::iterator it_0;
                it_0 = all_image_frame.find(t_0);
                delete it_0->second.pre_integration;
                it_0->second.pre_integration = nullptr;

                for (map<double, ImageFrame>::iterator it = all_image_frame.begin(); it != it_0; ++it)
                {
                    if (it->second.pre_integration)
                        delete it->second.pre_integration;
                    it->second.pre_integration = NULL;
                }

                all_image_frame.erase(all_image_frame.begin(), it_0);
                all_image_frame.erase(t_0);
            }
            slideWindowOld();
        }
    }
    else
    {
        if (frame_count == WINDOW_SIZE)
        {
            for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
            {
                double tmp_dt = dt_buf[frame_count][i];
                Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];

                pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

                dt_buf[frame_count - 1].push_back(tmp_dt);
                linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
            }

            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
            Vs[frame_count - 1] = Vs[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];
            Bas[frame_count - 1] = Bas[frame_count];
            Bgs[frame_count - 1] = Bgs[frame_count];

            delete pre_integrations[WINDOW_SIZE];
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();

            slideWindowNew();
        }
    }
}

// real marginalization is removed in solve_ceres()
void Estimator::slideWindowNew()
{
    sum_of_front++;
    f_manager.removeFront(frame_count);
}
// real marginalization is removed in solve_ceres()
void Estimator::slideWindowOld()
{
    sum_of_back++;

    bool shift_depth = solver_flag == NON_LINEAR ? true : false;
    if (shift_depth)
    {
        Matrix3d R0, R1;
        Vector3d P0, P1;
        R0 = back_R0 * ric[0];
        R1 = Rs[0] * ric[0];
        P0 = back_P0 + back_R0 * tic[0];
        P1 = Ps[0] + Rs[0] * tic[0];
        f_manager.removeBackShiftDepth(R0, P0, R1, P1);
    }
    else
        f_manager.removeBack();
}
