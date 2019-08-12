#include "System.h"

#include <pangolin/pangolin.h>

using namespace std;
using namespace cv;
using namespace pangolin;
System::System(string sConfig_file)
    : bStart_backend(true)
{
    //string sConfig_file = sConfig_file_ + "euroc_config.yaml";

    cout << "1 System() sConfig_file: " << sConfig_file << endl;
    readParameters(sConfig_file);

    trackerData[0].readIntrinsicParameter(sConfig_file);

    estimator.setParameter();
    ofs_pose.open("../data/pose_output.txt", fstream::app | fstream::out);
    if (!ofs_pose.is_open())
    {
        cerr << "ofs_pose is not open" << endl;
    }
    // thread thd_RunBackend(&System::process,this);
    // thd_RunBackend.detach();
    cout << "2 System() end" << endl;
}

System::~System()
{
    bStart_backend = false;

    pangolin::QuitAll();

    m_buf.lock();
    while (!feature_buf.empty())
        feature_buf.pop();
    while (!imu_buf.empty())
        imu_buf.pop();
    m_buf.unlock();

    m_estimator.lock();
    estimator.clearState();
    m_estimator.unlock();

    ofs_pose.close();
}

/*
 * 收到一帧image data
 * - trackerData[0].readImage(img, dStampSec); //left image tracking　读入一副图片进行特征提取与追踪
 * - updateID(i);//对于新提取的特征点，id往后顺延
 * - feature_buf.push(feature_points); //对于连续追踪到的特征点，放入ｆeature buf待处理
 * 
 */
void System::PubImageData(double dStampSec, Mat &img)
{
    if (!init_feature)
    {
        cout << "1 PubImageData skip the first detected feature, which doesn't contain optical flow speed" << endl;
        init_feature = 1;
        return;
    }

    if (first_image_flag)
    {
        cout << "2 PubImageData first_image_flag" << endl;
        first_image_flag = false;
        first_image_time = dStampSec;
        last_image_time = dStampSec;
        return;
    }
    // detect unstable camera stream
    if (dStampSec - last_image_time > 1.0 || dStampSec < last_image_time)
    {
        cerr << "3 PubImageData image discontinue! reset the feature tracker!" << endl;
        first_image_flag = true;
        last_image_time = 0;
        pub_count = 1;
        return;
    }
    last_image_time = dStampSec;
    // frequency control
    if (round(1.0 * pub_count / (dStampSec - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (dStampSec - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = dStampSec;
            pub_count = 0;
        }
    }
    else
    {
        PUB_THIS_FRAME = false;
    }

    TicToc t_r; //记录处理一帧图片的时间
    // cout << "3 PubImageData t : " << dStampSec << endl;
    trackerData[0].readImage(img, dStampSec); //left image tracking　读入一副图片进行特征提取与追踪

    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        completed |= trackerData[0].updateID(i); //对于新提取的特征点，id往后顺延

        if (!completed)
            break;
    }
    // cout << "current frame feature tracking time: " << t_r.toc() << endl;
    if (PUB_THIS_FRAME)
    {
        pub_count++;
        shared_ptr<IMG_MSG> feature_points(new IMG_MSG());
        feature_points->header = dStampSec;
        vector<set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            auto &un_pts = trackerData[i].cur_un_pts;
            auto &cur_pts = trackerData[i].cur_pts;
            auto &ids = trackerData[i].ids;
            auto &pts_velocity = trackerData[i].pts_velocity;
            for (unsigned int j = 0; j < ids.size(); j++)
            {
                if (trackerData[i].track_cnt[j] > 1) //光流连续追踪的点
                {
                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);
                    double x = un_pts[j].x; //归一化相机坐标
                    double y = un_pts[j].y;
                    double z = 1;
                    feature_points->points.push_back(Vector3d(x, y, z));
                    feature_points->id_of_point.push_back(p_id * NUM_OF_CAM + i);
                    feature_points->u_of_point.push_back(cur_pts[j].x);
                    feature_points->v_of_point.push_back(cur_pts[j].y);
                    // cout << " current pts: " << cur_pts[j].x << " , " << cur_pts[j].y << endl;
                    // cout << " current_un pts: " << un_pts[j].x << " , " << un_pts[j].y << endl;
                    feature_points->velocity_x_of_point.push_back(pts_velocity[j].x);
                    feature_points->velocity_y_of_point.push_back(pts_velocity[j].y);
                }
            }
            // skip the first image; since no optical speed on frist image
            if (!init_pub)
            {
                cout << "4 PubImage init_pub skip the first image!" << endl;
                init_pub = true;
            }
            else
            {
                m_buf.lock();
                feature_buf.push(feature_points); //将当前帧tracking feature 放入　feature buf
                // cout << "5 PubImage t : " << fixed << feature_points->header
                //      << " feature_buf size: " << feature_buf.size() << endl;
                m_buf.unlock();
                con.notify_one(); //随机唤醒一个等待的线程
            }
        }
    } //if PUB_THIS_FRAME end
    cv::Mat show_img;
    cv::cvtColor(img, show_img, CV_GRAY2RGB);
    //是否可视化追踪过程
    if (SHOW_TRACK)
    {
        for (unsigned int j = 0; j < trackerData[0].cur_pts.size(); j++)
        {
            double len = min(1.0, 1.0 * trackerData[0].track_cnt[j] / WINDOW_SIZE);
            cv::circle(show_img, trackerData[0].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2); // BGR  追踪的连续帧如果大于windows size就是red, 新提点是偏蓝
        }

        cv::namedWindow("IMAGE", CV_WINDOW_AUTOSIZE);
        cv::imshow("IMAGE", show_img);
        cv::waitKey(1);
    }
    // cout << "5 PubImage" << endl;
}

//如果其他的module已经提取出来图像的特征，并对特征进行了匹配，我们只需要将这些点放入feature buffer
void System::PubFeatureData(double dStampSec, const vector<int> &feature_id, const vector<Vector2d> &feature, const vector<Vector2d> &observation, std::vector<Vector2d> &featureVelocity)
{

    shared_ptr<IMG_MSG> feature_points(new IMG_MSG());
    feature_points->header = dStampSec;
    vector<set<int>> hash_ids(NUM_OF_CAM);
    for (int i = 0; i < NUM_OF_CAM; i++)
    {

        for (unsigned int j = 0; j < feature_id.size(); j++)
        {

            int p_id = feature_id[j];
            hash_ids[i].insert(p_id);
            double x_value = feature[j].x();
            double y_value = feature[j].y();
            double z = 1;
            feature_points->points.push_back(Vector3d(x_value, y_value, z));
            //feature_points->points.push_back(landmak[j]);
            feature_points->id_of_point.push_back(p_id * NUM_OF_CAM + i);
            feature_points->u_of_point.push_back(observation[j].x());
            feature_points->v_of_point.push_back(observation[j].y());
            feature_points->velocity_x_of_point.push_back(featureVelocity[j].x());
            feature_points->velocity_y_of_point.push_back(featureVelocity[j].y());
        }
        // skip the first image; since no optical speed on frist image
        if (!init_pub)
        {
            cout << "4 PubImage init_pub skip the first image!" << endl;
            init_pub = true;
        }
        else
        {
            m_buf.lock();
            feature_buf.push(feature_points);
            // cout << " PubImage t : " << fixed << feature_points->header
            //     << " feature_buf size: " << feature_buf.size() << endl;
            m_buf.unlock();
            con.notify_one(); //随机唤醒一个等待的线程
        }
    }
}

/*
 * getMeasurments　利用时间戳简单把imu 与 cam数据对齐　[--------O--------O--------O--------O--------]，保证每个视觉帧前面、后面都有若干imu数据
 * 如果数据没有对齐进行相应的等待和丢弃
 * ImgConstPtr img_msg = feature_buf.front();　一个视觉帧
 * vector<ImuConstPtr> IMUs;　视觉帧之前的若干imu帧
 * 　
 * measurements.emplace_back(IMUs, img_msg); //最后将一帧cam和它前面的一组imu打包放入measurements　vector中给下一级
 */
vector<pair<vector<ImuConstPtr>, ImgConstPtr>> System::getMeasurements()
{
    vector<pair<vector<ImuConstPtr>, ImgConstPtr>> measurements;

    while (true)
    {
        if (imu_buf.empty() || feature_buf.empty())
        {
            // cerr << "1 imu_buf.empty() || feature_buf.empty()" << endl;
            return measurements;
        }

        // cout << "imu_buf.back() time: " <<imu_buf.back()->header << "feature_buf.front() time: "<<feature_buf.front()->header << endl;
        // cout << "imu_buf size: " << imu_buf.size() << "  feature_buf size: " << feature_buf.size() << endl;
        //正常情况　[--------O---] 如果imu back小于　feature_front,[[----------- O]]说明imu都在cam前面,需要等imu一会儿，imu频率很高的，要是系统跑起来，imubuffer可定会超过 feature buffer,所以只能发生在beginning
        if (!(imu_buf.back()->header > feature_buf.front()->header + estimator.td)) //imu_back < feature_front　所有的imu都在cam前面
        {
            cerr << "wait for imu, only should happen at the beginning, sum_of_wait: "
                 << sum_of_wait << endl;
            sum_of_wait++;
            return measurements;
        }
        //正常情况　[--------O---] 如果imu front 大于　feature_front,[ O ----------- ],这种情况current frame没有对应的imu数据，也是因为系统启动时，所以要把这帧图像丢到
        if (!(imu_buf.front()->header < feature_buf.front()->header + estimator.td)) //imu_front > feature_front
        {
            cerr << "throw img, only should happen at the beginning" << endl;
            feature_buf.pop();
            continue;
        }
        //排除上面的异常情况后，正常情况　[--------O---]
        //取出这一帧cam feature 和前面的IMU数据进行处理
        ImgConstPtr img_msg = feature_buf.front();
        feature_buf.pop();

        // [--------O---] ==> [O-------------]
        vector<ImuConstPtr> IMUs;
        while (imu_buf.front()->header < img_msg->header + estimator.td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        // cout << "1 getMeasurements IMUs size: " << IMUs.size() << endl;
        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty())
        {
            cerr << "no imu between two image" << endl;
        }
        // cout << "1 getMeasurements img t: " << fixed << img_msg->header
        //     << " imu begin: "<< IMUs.front()->header
        //     << " end: " << IMUs.back()->header
        //     << endl;
        measurements.emplace_back(IMUs, img_msg); //一帧cam和它前面的一组imu
        
    }
    return measurements;
}

/*
 * -接收一帧imu数据
 *
 */
void System::PubImuData(double dStampSec, const Eigen::Vector3d &vGyr,
                        const Eigen::Vector3d &vAcc)
{
    shared_ptr<IMU_MSG> imu_msg(new IMU_MSG());
    imu_msg->header = dStampSec;
    imu_msg->linear_acceleration = vAcc;
    imu_msg->angular_velocity = vGyr;

    if (dStampSec < last_imu_t)
    {
        cerr << "imu message in disorder!" << endl;
        return;
    }
    last_imu_t = dStampSec;
    // cout << "1 PubImuData t: " << fixed << imu_msg->header
    //     << " acc: " << imu_msg->linear_acceleration.transpose()
    //     << " gyr: " << imu_msg->angular_velocity.transpose() << endl;
    m_buf.lock();
    imu_buf.push(imu_msg);
    // cout << "1 PubImuData t: " << fixed << imu_msg->header
    //     << " imu_buf size:" << imu_buf.size() << endl;
    m_buf.unlock();
    con.notify_one();
}

// thread: visual-inertial odometry
/*
 * - getMeasurements  得到一组或者多组对齐的　若干imu和一帧图像对应的数据

 * -  estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
 *      利用这一包的imu数据计算一个预积分增量，后面会填充到对应的视觉帧里，　
 *      同时计算一个imu积分，来估计每一个body的P V R, 后面的非线性优化会对这些值调优
 * 
 * -  estimator.processImage(image, img_msg->header); 对于每一帧imu的所有点
 *      　　把当前帧的特征点更新到feature list中，根据特征点的新旧比以及视差，设定关键帧和边缘化策略
 *  　　    把当前帧更新到all_image_frame中
 * -　当然还有很多的操作都在　processImage中进行
 *          在线的imu-cam外参校准
 *          系统初始化
 *          把feature　list中的特征点三角化来估计深度
 *          非线性优化的求解
 */
void System::ProcessBackEnd()
{
    cout << "1 ProcessBackEnd start" << endl;
    while (bStart_backend)
    {
        // cout << "1 process()" << endl;
        vector<pair<vector<ImuConstPtr>, ImgConstPtr>> measurements; //一组imu和一帧图像对应的数据

        unique_lock<mutex> lk(m_buf);
        //wait有两个参数，
        //前一个参数等待其他线程触发con.notify_one()　来解锁 unique_lock<mutex> lk(m_buf)
        //后一个参数lamda函数，如果返回false 就让出互斥量给其他线程，继续休眠，如果返回true，则可以独占这个互斥量，往下执行
        con.wait(lk, [&] {
            return (measurements = getMeasurements()).size() != 0; //因为操作　getMeasurements也会读写imu_buf和feature buf,所以还是需要m_buf的互斥锁的
        });
        if (measurements.size() >= 1)
        {
             cout << "1 getMeasurements size: " << measurements.size()
                 << " imu size: " << measurements[0].first.size() //这一帧图像前面有多少imu
                 << " cam size: " << measurements.size()
                 << " feature_buf size: " << feature_buf.size()
                 << " imu_buf size: " << imu_buf.size() 
                 << "\r\n" << endl;
        }
        lk.unlock(); //当getMeasurements　参数传递给measurements后，赶紧让出互斥量，重新获取m_estimator互斥量控制权
        m_estimator.lock();
        for (auto &measurement : measurements)
        {
            auto img_msg = measurement.second; //image feature　后面用
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;

            //先处理imu
            for (auto &imu_msg : measurement.first) //对每个imu
            {
                double t = imu_msg->header;
                double img_t = imu_msg->header + estimator.td;
                if (t <= img_t)
                {
                    if (current_time < 0)
                        current_time = t;

                    double dt = t - current_time;
                    assert(dt >= 0);
                    current_time = t;
                    dx = imu_msg->linear_acceleration.x();
                    dy = imu_msg->linear_acceleration.y();
                    dz = imu_msg->linear_acceleration.z();
                    rx = imu_msg->angular_velocity.x();
                    ry = imu_msg->angular_velocity.y();
                    rz = imu_msg->angular_velocity.z();
                    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("1 BackEnd imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);
                }
                else
                {
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    current_time = img_t;
                    assert(dt_1 >= 0);
                    assert(dt_2 >= 0);
                    assert(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu_msg->linear_acceleration.x();
                    dy = w1 * dy + w2 * imu_msg->linear_acceleration.y();
                    dz = w1 * dz + w2 * imu_msg->linear_acceleration.z();
                    rx = w1 * rx + w2 * imu_msg->angular_velocity.x();
                    ry = w1 * ry + w2 * imu_msg->angular_velocity.y();
                    rz = w1 * rz + w2 * imu_msg->angular_velocity.z();
                    estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                }
            }

            // cout << "processing vision data with stamp:" << img_msg->header
            //      << " img_msg->points.size: " << img_msg->points.size() << endl;

            // TicToc t_s;
            // img_msg是一帧cam对应的数据
            map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image;
            //<key-feature_id  value:camera_id(0/1) xyz_uv_velocity>
            //归一化相机坐标 x/z y/z 1  像素坐标，以及x/z y/z 的运动速度
            for (unsigned int i = 0; i < img_msg->points.size(); i++)
            {
                int v = img_msg->id_of_point[i] + 0.5;
                int feature_id = v / NUM_OF_CAM; //当时编码id的时候　p_id * NUM_OF_CAM + i　feature_id*cam数目＋cam0/cam1
                int camera_id = v % NUM_OF_CAM;  // 0/1
                double x = img_msg->points[i].x();
                double y = img_msg->points[i].y();
                double z = img_msg->points[i].z();

                double p_u = img_msg->u_of_point[i];
                double p_v = img_msg->v_of_point[i];
                double velocity_x = img_msg->velocity_x_of_point[i];
                double velocity_y = img_msg->velocity_y_of_point[i];
                assert(z == 1);
                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                image[feature_id].emplace_back(camera_id, xyz_uv_velocity); //image是一帧图像的一组特征点
            }
            TicToc t_processImage;
            estimator.processImage(image, img_msg->header); // 转换成vins的　cam数据的组织结构

            if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            {
                Vector3d p_wi;
                Quaterniond q_wi;
                q_wi = Quaterniond(estimator.Rs[WINDOW_SIZE]);
                p_wi = estimator.Ps[WINDOW_SIZE];
                vPath_to_draw.push_back(p_wi);
                double dStamp = estimator.Headers[WINDOW_SIZE];
                cout << "1 BackEnd processImage dt: " << fixed << t_processImage.toc() << " stamp: " << dStamp << " p_wi: " << p_wi.transpose() << endl;
                ofs_pose << fixed << dStamp << " " << p_wi.transpose() << " " << q_wi.coeffs().transpose() << endl;
            }
        }
        m_estimator.unlock();
    }
}

void System::Draw()
{
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 384, 0.1, 1000),
        pangolin::ModelViewLookAt(-5, 0, 15, 7, 0, 0, 1.0, 0.0, 0.0));

    pangolin::View &d_cam = pangolin::CreateDisplay()
                                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
                                .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(0.75f, 0.75f, 0.75f, 0.75f);
        glColor3f(0, 0, 1);
        pangolin::glDrawAxis(3);

        // draw poses
        glColor3f(0, 0, 0);
        glLineWidth(2);
        glBegin(GL_LINES);
        int nPath_size = vPath_to_draw.size();
        for (int i = 0; i < nPath_size - 1; ++i)
        {
            glVertex3f(vPath_to_draw[i].x(), vPath_to_draw[i].y(), vPath_to_draw[i].z());
            glVertex3f(vPath_to_draw[i + 1].x(), vPath_to_draw[i + 1].y(), vPath_to_draw[i + 1].z());
        }
        glEnd();

        // points
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
        {
            glPointSize(5);
            glBegin(GL_POINTS);
            for (int i = 0; i < WINDOW_SIZE + 1; ++i)
            {
                Vector3d p_wi = estimator.Ps[i];
                glColor3f(1, 0, 0);
                glVertex3d(p_wi[0], p_wi[1], p_wi[2]);
            }
            glEnd();
        }

        // show the imu integration pose
        {
            glPointSize(5);
            glBegin(GL_POINTS);
            for (size_t i = 0; i < imu_integration_poses.size(); ++i)
            {
                Vector3d p_wi = imu_integration_poses[i];
                glColor3f(0, 1, 0);
                glVertex3d(p_wi[0], p_wi[1], p_wi[2]);
            }
            glEnd();
        }

        // show the real pose
        {
            glPointSize(5);
            glBegin(GL_POINTS);
            for (size_t i = 0; i < real_poses.size(); ++i)
            {
                Vector3d p_wi = real_poses[i];
                glColor3f(0, 0, 1);
                glVertex3d(p_wi[0], p_wi[1], p_wi[2]);
            }
            glEnd();
        }

        pangolin::FinishFrame();
        usleep(5000); // sleep 5 ms
    }
}

void System::midPointIntegration(double _dt,
                                 Eigen::Vector3d &acc_0, Eigen::Vector3d &gyro_0,
                                 Eigen::Vector3d &acc_1, Eigen::Vector3d &gyro_1,
                                 Eigen::Vector3d &acc_bias, Eigen::Vector3d &gyro_bias,
                                 Eigen::Vector3d &delta_p, Eigen::Quaterniond &delta_q, Eigen::Vector3d &delta_v)
{
    Eigen::Vector3d un_gyro = 0.5 * (gyro_0 + gyro_1) - gyro_bias;
    delta_q = delta_q * Eigen::Quaterniond(1, un_gyro(0) * _dt / 2, un_gyro(1) * _dt / 2, un_gyro(2) * _dt / 2);

    Eigen::Vector3d gw(0, 0, -9.81); // ENU frame
    Eigen::Vector3d un_acc_0 = delta_q.toRotationMatrix() * (acc_0 - acc_bias) + gw;
    Eigen::Vector3d un_acc_1 = delta_q.toRotationMatrix() * (acc_1 - acc_bias) + gw;
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    // △v
    delta_v = delta_v + un_acc * _dt;

    // △p
    delta_p = delta_p + delta_v * _dt + 0.5 * un_acc * _dt * _dt;
}

void System::eulerIntegration(double _dt, 
                      const Eigen::Vector3d &acc_k,    const Eigen::Vector3d &gyro_k,     // 第k帧 IMU data
                      const Eigen::Vector3d &acc_bias, const Eigen::Vector3d &gyro_bias,  // IMU 偏置项，这里假定为常数
                      Eigen::Vector3d &delta_p,  Eigen::Quaterniond &delta_q,  Eigen::Vector3d &delta_v //前一帧result,以及updated当前帧积分result
                      )
{
    Eigen::Vector3d  un_gyro = gyro_k - gyro_bias;  //  w = gyro_body - gyro_bias
    //  △q  delta_q = [1 , 1/2 * thetax , 1/2 * theta_y, 1/2 * theta_z]
    delta_q = delta_q * Eigen::Quaterniond(1, un_gyro(0)*_dt/2, un_gyro(1)*_dt/2, un_gyro(2)*_dt/2);

    Eigen::Vector3d gw(0,0,-9.81);    // ENU frame
    Eigen::Vector3d un_acc = delta_q.toRotationMatrix() * (acc_k) + gw;  // aw = Rwb * ( acc_body - acc_bias ) + gw

    // △v
    delta_v = delta_v + un_acc*_dt;
    
    // △p
    delta_p = delta_p + delta_v*_dt + 0.5*un_acc*_dt*_dt;
} 
