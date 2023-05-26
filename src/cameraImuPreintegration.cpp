#include "utility.h"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>


#include "lio_sam/usr_state.h"

using namespace gtsam;

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

class IMUPreintegration : public ParamServer
{
public:

    std::mutex mtx;

    ros::Subscriber subImu;
    ros::Subscriber subOdometry;
    //ros::Publisher pubImuOdometry;
    ros::Publisher pubLidarState;

    ros::Subscriber subVinsState;

    ros::Subscriber subVinsOdom;

    bool systemInitialized = false;

    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;
    gtsam::Vector noiseModelBetweenBias;


    gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
    gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;

    gtsam::PreintegratedImuMeasurements *imuIntegratorVins;

    gtsam::PreintegratedImuMeasurements *imuIntegratorVins_last;
    gtsam::PreintegratedImuMeasurements *imuIntegratorVins_current;

    std::deque<sensor_msgs::Imu> imuQueOpt;
    std::deque<sensor_msgs::Imu> imuQueImu;
    std::deque<sensor_msgs::Imu> imuQueVins_last;
    std::deque<sensor_msgs::Imu> imuQueVins_current;

    std::deque<sensor_msgs::Imu> imuQueVins;

    std::deque<lio_sam::usr_state> vinsStateQueue_last;
    std::deque<lio_sam::usr_state> vinsStateQueue_current;

    gtsam::Pose3 prevPose_;
    gtsam::Vector3 prevVel_;
    gtsam::NavState prevState_;
    gtsam::imuBias::ConstantBias prevBias_;

    gtsam::NavState prevStateOdom;
    gtsam::imuBias::ConstantBias prevBiasOdom;


    gtsam::imuBias::ConstantBias vinsBias_current;
    gtsam::Pose3 vinsPose;
    gtsam::Vector3 vinsVel;

    bool doneFirstOpt = false;
    double lastImuT_imu = -1;
    double lastImuT_opt = -1;

    double lastImuT_vins_last = -1;
    double lastImuT_vins_current = -1;

    double lastVinsT_last = -1;
    double lastVinsT_current = -1;


    gtsam::ISAM2 optimizer;
    gtsam::NonlinearFactorGraph graphFactors;
    gtsam::Values graphValues;

    const double delta_t = 0;

    int key = 1;

    gtsam::Pose3 imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));
    gtsam::Pose3 lidar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z()));


    // VINS Parameters
    double lastCorrectionTime = -1;
    bool vinsFlag = false;

    double currentCorrectionTime = 0;

    int counterState =0;

    IMUPreintegration()
    {
        subImu      = nh.subscribe<sensor_msgs::Imu>  (imuTopic,                   2000, &IMUPreintegration::imuHandler,      this, ros::TransportHints().tcpNoDelay());
        //subOdometry = nh.subscribe<nav_msgs::Odometry>("/lio_sam_loopFusion/odometry_rect", 5,    &IMUPreintegration::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        subOdometry = nh.subscribe<nav_msgs::Odometry>("/lio_sam_vins/odometry", 5,    &IMUPreintegration::odometryHandler, this, ros::TransportHints().tcpNoDelay());

        subVinsOdom = nh.subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry_incremental", 25,    &IMUPreintegration::vinsOdomHandler, this, ros::TransportHints().tcpNoDelay());

        subVinsState = nh.subscribe<lio_sam::usr_state>("/usr/opt/lidarState", 200,    &IMUPreintegration::stateHandler_vins, this, ros::TransportHints().tcpNoDelay());

        //pubImuOdometry = nh.advertise<nav_msgs::Odometry> (odomTopic+"_incremental", 2000);
        pubLidarState = nh.advertise<lio_sam::usr_state> ("/usr/opt/vinsState", 20);
        //pubLidarState = nh.advertise<nav_msgs::Odometry> ("/vins_estimator/odometry", 20);

        boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
        p->accelerometerCovariance  = gtsam::Matrix33::Identity(3,3) * pow(imuAccNoise, 2); // acc white noise in continuous
        p->gyroscopeCovariance      = gtsam::Matrix33::Identity(3,3) * pow(imuGyrNoise, 2); // gyro white noise in continuous
        p->integrationCovariance    = gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2); // error committed in integrating position from velocities
        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());; // assume zero initial bias

        priorPoseNoise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
        priorVelNoise   = gtsam::noiseModel::Isotropic::Sigma(3, 1e4); // m/s
        priorBiasNoise  = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3); // 1e-2 ~ 1e-3 seems to be good
        correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished()); // rad,rad,rad,m, m, m
        correctionNoise2 = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished()); // rad,rad,rad,m, m, m
        noiseModelBetweenBias = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();
        
        imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for IMU message thread
        imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for optimization        

        imuIntegratorVins = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for IMU message thread

        imuIntegratorVins_last = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for IMU message thread
        imuIntegratorVins_current = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for optimization        
    }

    void resetOptimization()
    {
        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.1;
        optParameters.relinearizeSkip = 1;
        optimizer = gtsam::ISAM2(optParameters);

        gtsam::NonlinearFactorGraph newGraphFactors;
        graphFactors = newGraphFactors;

        gtsam::Values NewGraphValues;
        graphValues = NewGraphValues;
    }

    void resetParams()
    {
        lastImuT_imu = -1;
        doneFirstOpt = false;
        systemInitialized = false;

        lastCorrectionTime = -1;
        vinsFlag = false;

        lastImuT_vins_last = -1;
        lastImuT_vins_current = -1;

        lastVinsT_last = -1;
        lastVinsT_current = -1;
    }

    void stateHandler_vins(const lio_sam::usr_stateConstPtr& msgState)
    {
        std::lock_guard<std::mutex> lock(mtx);

        //Convert to lidar frame
        vinsStateQueue_last.push_back(*msgState);
        vinsStateQueue_current.push_back(*msgState);


        if (counterState>100){
            vinsFlag = true;
        }
        else{
        counterState += 1;

        }
    }

    void vinsOdomHandler(const nav_msgs::Odometry::ConstPtr& odomMsg){

        std::lock_guard<std::mutex> lock(mtx);

        double vinsCorrectionTime = ROS_TIME(odomMsg);

        if (imuQueOpt.empty())
            return;


        if (systemInitialized == false){
            return;
        }

        while (!imuQueVins.empty())
        {
            // pop and integrate imu data that is between two optimizations
            sensor_msgs::Imu *thisImu = &imuQueVins.front();
            double imuTime = ROS_TIME(thisImu);
            if (imuTime < vinsCorrectionTime - delta_t)
            {
                // double dt = (lastImuT_opt < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_opt);
                double dt = 1.0 / 500.0;
                if (lastImuT_opt > 0 && imuTime - lastImuT_opt >=0)
                {
                    dt = imuTime - lastImuT_opt;
                }            
                if (dt<=0){
                    dt = 1.0/500.0;
                }

                imuIntegratorVins ->integrateMeasurement(
                        gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
                
                lastImuT_opt = imuTime;
                imuQueVins.pop_front();
            }
            else
                break;
        }

        gtsam::NavState vinsState = imuIntegratorVins->predict(prevStateOdom, prevBiasOdom);

        gtsam::Vector3 vel;
        vel = gtsam::Vector3(vinsState.velocity().x(), vinsState.velocity().y(), vinsState.velocity().z());

        //ros::Time odomStamp = odomMsg->header.stamp;
        //publishLidarState(odomStamp, vinsState , prevBias_);
    }

    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);

        currentCorrectionTime = ROS_TIME(odomMsg);

        // make sure we have imu data to integrate
        if (imuQueOpt.empty())
            return;

        float p_x = odomMsg->pose.pose.position.x;
        float p_y = odomMsg->pose.pose.position.y;
        float p_z = odomMsg->pose.pose.position.z;
        float r_x = odomMsg->pose.pose.orientation.x;
        float r_y = odomMsg->pose.pose.orientation.y;
        float r_z = odomMsg->pose.pose.orientation.z;
        float r_w = odomMsg->pose.pose.orientation.w;
        bool degenerate = (int)odomMsg->pose.covariance[0] == 1 ? true : false;
        gtsam::Pose3 lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));


        // 0. initialize system
        if (systemInitialized == false)
        {
            resetOptimization();

            // pop old IMU message
            while (!imuQueOpt.empty())
            {
                if (ROS_TIME(&imuQueOpt.front()) < currentCorrectionTime - delta_t)
                {
                    lastImuT_opt = ROS_TIME(&imuQueOpt.front());
                    imuQueOpt.pop_front();
                }
                else
                    break;
            }
            if (vinsFlag == true){

                 getVinsState_current();
                //gtsam::NavState vinsStatePred_last = getVinsState_last();
                
                noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-5, 1e-5, 1e-5, 1e-3, 1e-3, 1e-3).finished());

                gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), vinsPose, odometryNoise);
                graphFactors.add(priorPose);
                // initial velocity
                gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), vinsVel, priorVelNoise);
                graphFactors.add(priorVel);
                // initial bias
                gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), vinsBias_current, priorBiasNoise);
                graphFactors.add(priorBias);
                //
                // add values
                graphValues.insert(X(0), vinsPose);
                graphValues.insert(V(0), vinsVel);
                graphValues.insert(B(0), vinsBias_current);
            }
            else{
                // initial pose
                prevPose_ = lidarPose.compose(lidar2Imu);
                gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
                graphFactors.add(priorPose);
                // initial velocity
                prevVel_ = gtsam::Vector3(0, 0, 0);
                gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
                graphFactors.add(priorVel);
                // initial bias
                prevBias_ = gtsam::imuBias::ConstantBias();
                gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
                graphFactors.add(priorBias);
                // add values
                graphValues.insert(X(0), prevPose_);
                graphValues.insert(V(0), prevVel_);
                graphValues.insert(B(0), prevBias_);
            }
            // optimize once
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();

            imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
            imuIntegratorVins->resetIntegrationAndSetBias(prevBias_);
            imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
            
            key = 1;
            systemInitialized = true;

            lastCorrectionTime = currentCorrectionTime;

            return;
        }


        // reset graph for speed
        if (key == 100)
        {
            // get updated noise before reset
            gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise  = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(V(key-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(B(key-1)));
            // reset graph
            resetOptimization();
            // add pose
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
            graphFactors.add(priorPose);
            // add velocity
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, updatedVelNoise);
            graphFactors.add(priorVel);
            // add bias
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, updatedBiasNoise);
            graphFactors.add(priorBias);
            // add values
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);
            // optimize once
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();

            key = 1;
        }


        // 1. integrate imu data and optimize
        while (!imuQueOpt.empty())
        {
            // pop and integrate imu data that is between two optimizations
            sensor_msgs::Imu *thisImu = &imuQueOpt.front();
            double imuTime = ROS_TIME(thisImu);
            if (imuTime < currentCorrectionTime - delta_t)
            {
                // double dt = (lastImuT_opt < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_opt);
                double dt = 1.0 / 500.0;
                if (lastImuT_opt > 0 && imuTime - lastImuT_opt >=0)
                {
                    dt = imuTime - lastImuT_opt;
                }            
                if (dt<=0){
                    dt = 1.0/500.0;
                }

                imuIntegratorOpt_->integrateMeasurement(
                        gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
                
                lastImuT_opt = imuTime;
                imuQueOpt.pop_front();
            }
            else
                break;
        }

        // add imu factor to graph
        const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
        gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
        graphFactors.add(imu_factor);
        // add imu bias between factor
        graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                         gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));
        // add camera predicted odometry between lidar factors
        //
        if (vinsFlag == true){

             getVinsState_current();
            //gtsam::NavState vinsStatePred_last = getVinsState_last();
            
            noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-5, 1e-5, 1e-5, 1e-3, 1e-3, 1e-3).finished());

            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(key), vinsPose, odometryNoise);
            graphFactors.add(priorPose);
            // initial velocity
            //gtsam::PriorFactor<gtsam::Vector3> priorVel(V(key), vinsVel, priorVelNoise);
            //graphFactors.add(priorVel);
            // initial bias
            //gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(key), vinsBias_current, priorBiasNoise);
            //graphFactors.add(priorBias);
        }
        // add pose factor
        gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);
        //gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, degenerate ? correctionNoise2 : correctionNoise);
        graphFactors.add(gtsam::BetweenFactor<Pose3>(X(key-1), X(key), prevPose_.between(curPose), correctionNoise2));

        // insert predicted values
        gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
        graphValues.insert(X(key), propState_.pose());
        graphValues.insert(V(key), propState_.v());
        graphValues.insert(B(key), prevBias_);
        // optimize
        optimizer.update(graphFactors, graphValues);
        optimizer.update();
        graphFactors.resize(0);
        graphValues.clear();
        // Overwrite the beginning of the preintegration for the next step.
        gtsam::Values result = optimizer.calculateEstimate();
        prevPose_  = result.at<gtsam::Pose3>(X(key));
        prevVel_   = result.at<gtsam::Vector3>(V(key));
        prevState_ = gtsam::NavState(prevPose_, prevVel_);
        prevBias_  = result.at<gtsam::imuBias::ConstantBias>(B(key));
        // Reset the optimization preintegration object.
        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
        // check optimization
        if (failureDetection(prevVel_, prevBias_))
        {
            resetParams();
            return;
        }


        // 2. after optiization, re-propagate imu odometry preintegration
        prevStateOdom = prevState_;
        prevBiasOdom  = prevBias_;
        // first pop imu message older than current correction data
        double lastImuQT = -1;
        while (!imuQueImu.empty() && ROS_TIME(&imuQueImu.front()) < currentCorrectionTime - delta_t)
        {
            lastImuQT = ROS_TIME(&imuQueImu.front());
            imuQueImu.pop_front();
        }
        // repropogate
        if (!imuQueImu.empty())
        {
            // reset bias use the newly optimized bias
            imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
            // integrate imu message from the beginning of this optimization
            for (int i = 0; i < (int)imuQueImu.size(); ++i)
            {
                sensor_msgs::Imu *thisImu = &imuQueImu[i];
                double imuTime = ROS_TIME(thisImu);
                // double dt = (lastImuQT < 0) ? (1.0 / 500.0) :(imuTime - lastImuQT);
                double dt = 1.0/500.0;
                if (lastImuQT > 0 && imuTime - lastImuQT>=0)
                {
                    dt = imuTime - lastImuQT;
                }
                if (dt<=0){
                    dt = 1.0/500.0;
                }

                imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                                                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
                lastImuQT = imuTime;
            }
        }


        lastImuQT = -1;
        while (!imuQueVins.empty() && ROS_TIME(&imuQueVins.front()) < currentCorrectionTime - delta_t)
        {
            lastImuQT = ROS_TIME(&imuQueVins.front());
            imuQueVins.pop_front();
        }
        // repropogate
        if (!imuQueVins.empty())
        {
            // reset bias use the newly optimized bias
            imuIntegratorVins->resetIntegrationAndSetBias(prevBiasOdom);
            // integrate imu message from the beginning of this optimization
            for (int i = 0; i < (int)imuQueVins.size(); ++i)
            {
                sensor_msgs::Imu *thisImu = &imuQueVins[i];
                double imuTime = ROS_TIME(thisImu);
                // double dt = (lastImuQT < 0) ? (1.0 / 500.0) :(imuTime - lastImuQT);
                double dt = 1.0/500.0;
                if (lastImuQT > 0 && imuTime - lastImuQT>=0)
                {
                    dt = imuTime - lastImuQT;
                }
                if (dt<=0){
                    dt = 1.0/500.0;
                }

                imuIntegratorVins->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                                                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
                lastImuQT = imuTime;
            }
        }

        ++key;
        doneFirstOpt = true;

        ros::Time odomStamp = odomMsg->header.stamp;
        publishLidarState(odomStamp, prevState_, prevBias_);
    }

    void getVinsState_current(){

       double vinsTime = -1;
       //gtsam::NavState vinsState_current;
       //get current camera odometry
       while (!vinsStateQueue_current.empty())
       {
           // pop and integrate imu data that is between two optimizations
           lio_sam::usr_state *vinsState = &vinsStateQueue_current.front();

           vinsTime = ROS_TIME(vinsState);
           if (vinsTime <= currentCorrectionTime)
           {
               float pX = vinsState->pose.pose.position.x;
               float pY = vinsState->pose.pose.position.y;
               float pZ = vinsState->pose.pose.position.z;
               float qX = vinsState->pose.pose.orientation.x;
               float qY = vinsState->pose.pose.orientation.y;
               float qZ = vinsState->pose.pose.orientation.z;
               float qW = vinsState->pose.pose.orientation.w;
               
               float vX = vinsState->twist.twist.linear.x;
               float vY = vinsState->twist.twist.linear.y;
               float vZ = vinsState->twist.twist.linear.z;

               float bias_accX = vinsState->biasAcc_.x;
               float bias_accY = vinsState->biasAcc_.y;
               float bias_accZ = vinsState->biasAcc_.z;

               float bias_gyrX = vinsState->biasGyro_.x;
               float bias_gyrY = vinsState->biasGyro_.y;
               float bias_gyrZ = vinsState->biasGyro_.z;


               vinsPose = gtsam::Pose3(gtsam::Rot3::Quaternion(qW, qX, qY, qZ), gtsam::Point3(pX, pY, pZ));
               vinsVel = gtsam::Vector3(vX, vY, vZ);

               gtsam::imuBias::ConstantBias vinsBias_tmp((gtsam::Vector(6) << bias_accX, bias_accY, bias_accZ, bias_gyrX, bias_gyrY, bias_gyrZ).finished()); 

               vinsBias_current = vinsBias_tmp;
               
               vinsStateQueue_current.pop_front();
           }
           else
               break;
       }
    }


    gtsam::NavState getVinsState_last(){

       double vinsTime = -1;
       gtsam::NavState vinsState_last;
       gtsam::imuBias::ConstantBias vinsBias_last;

       //get last camera odometry
       while (!vinsStateQueue_last.empty())
       {
           // pop and integrate imu data that is between two optimizations
           lio_sam::usr_state *vinsState = &vinsStateQueue_last.front();

           vinsTime = ROS_TIME(vinsState);
           if (vinsTime < lastCorrectionTime - delta_t)
           {
               float pX = vinsState->pose.pose.position.x;
               float pY = vinsState->pose.pose.position.y;
               float pZ = vinsState->pose.pose.position.z;
               float qX = vinsState->pose.pose.orientation.x;
               float qY = vinsState->pose.pose.orientation.y;
               float qZ = vinsState->pose.pose.orientation.z;
               float qW = vinsState->pose.pose.orientation.w;
               
               float vX = vinsState->twist.twist.linear.x;
               float vY = vinsState->twist.twist.linear.y;
               float vZ = vinsState->twist.twist.linear.z;

               float bias_accX = vinsState->biasAcc_.x;
               float bias_accY = vinsState->biasAcc_.y;
               float bias_accZ = vinsState->biasAcc_.z;

               float bias_gyrX = vinsState->biasGyro_.x;
               float bias_gyrY = vinsState->biasGyro_.y;
               float bias_gyrZ = vinsState->biasGyro_.z;


               gtsam::Pose3 vinsPose = gtsam::Pose3(gtsam::Rot3::Quaternion(qW, qX, qY, qZ), gtsam::Point3(pX, pY, pZ));
               gtsam::Vector3 vinsVel = gtsam::Vector3(vX, vY, vZ);

               vinsState_last = gtsam::NavState(vinsPose, vinsVel);

               gtsam::imuBias::ConstantBias vinsBias_tmp((gtsam::Vector(6) << bias_accX, bias_accY, bias_accZ, bias_gyrX, bias_gyrY, bias_gyrZ).finished()); 

               vinsBias_last = vinsBias_tmp;

               lastVinsT_last = vinsTime;

               vinsStateQueue_last.pop_front();
           }
           else
               break;
       }

       while (!imuQueVins_last.empty()){
           // pop and integrate imu data that is between two optimizations
           sensor_msgs::Imu *thisImu = &imuQueVins_last.front();
           double imuTime = ROS_TIME(thisImu);
           if (imuTime < lastVinsT_last){
               imuQueVins_last.pop_front();
           }
           else if (imuTime >= lastVinsT_last && imuTime < lastCorrectionTime - delta_t)
           {
               double dt = 1.0 / 500.0;

               if (lastImuT_vins_last > 0 && imuTime - lastImuT_vins_last >=0)
               {
                   dt = imuTime - lastImuT_vins_last;
               }            
                if (dt<=0){
                    dt = 1.0/500.0;
                }

               imuIntegratorVins_last->integrateMeasurement(
                       gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                       gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
               
               lastImuT_vins_last = imuTime;
               imuQueVins_last.pop_front();
           }
           else
               break;
       }

       // predict last camera odometry
       gtsam::NavState vinsStatePred_last = imuIntegratorVins_last->predict(vinsState_last, vinsBias_last);

       return vinsStatePred_last;

    }

    void publishLidarState(ros::Time odomStamp, gtsam::NavState currentState, gtsam::imuBias::ConstantBias biasOdom){
        // publish odometry
        lio_sam::usr_state lidarState;
        //nav_msgs::Odometry lidarState;
        lidarState.header.stamp = odomStamp;

        gtsam::Pose3 lidarPose = gtsam::Pose3(currentState.quaternion(), currentState.position());

        lidarState.pose.pose.position.x = lidarPose.translation().x();
        lidarState.pose.pose.position.y = lidarPose.translation().y();
        lidarState.pose.pose.position.z = lidarPose.translation().z();
        lidarState.pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
        lidarState.pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
        lidarState.pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
        lidarState.pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();
        
        lidarState.twist.twist.linear.x = currentState.velocity().x();
        lidarState.twist.twist.linear.y = currentState.velocity().y();
        lidarState.twist.twist.linear.z = currentState.velocity().z();

        lidarState.biasAcc_.x = biasOdom.accelerometer().x();
        lidarState.biasAcc_.y = biasOdom.accelerometer().y();
        lidarState.biasAcc_.z = biasOdom.accelerometer().z();

        lidarState.biasGyro_.x = biasOdom.gyroscope().x();
        lidarState.biasGyro_.y = biasOdom.gyroscope().y();
        lidarState.biasGyro_.z = biasOdom.gyroscope().z();

        pubLidarState.publish(lidarState);

    }

    bool failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur)
    {
        Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
        if (vel.norm() > 30)
        {
            ROS_WARN("Large velocity, reset IMU-preintegration!");
            return true;
        }

        Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
        Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
        if (ba.norm() > 1.0 || bg.norm() > 1.0)
        {
            ROS_WARN("Large bias, reset IMU-preintegration!");
            return true;
        }

        return false;
    }

    void imuHandler(const sensor_msgs::Imu::ConstPtr& imu_raw)
    {
        std::lock_guard<std::mutex> lock(mtx);

        sensor_msgs::Imu thisImu = imuConverter(*imu_raw);

        imuQueOpt.push_back(thisImu);
        imuQueImu.push_back(thisImu);

        imuQueVins_last.push_back(thisImu);
        imuQueVins_current.push_back(thisImu);

        imuQueVins.push_back(thisImu);

        //if (doneFirstOpt == false)
        //    return;

        //double imuTime = ROS_TIME(&thisImu);
        //// double dt = (lastImuT_imu < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_imu);
        //double dt = 1.0/500.0;
        //if (lastImuT_imu > 0 && imuTime - lastImuT_imu>=0)
        //{
        //    dt = imuTime - lastImuT_imu;
        //}
        //

        //lastImuT_imu = imuTime;

        //// integrate this single imu message
        //imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu.linear_acceleration.x, thisImu.linear_acceleration.y, thisImu.linear_acceleration.z),
        //                                        gtsam::Vector3(thisImu.angular_velocity.x,    thisImu.angular_velocity.y,    thisImu.angular_velocity.z), dt);

        //// predict odometry
        //gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);

        //// publish odometry
        //nav_msgs::Odometry odometry;
        //odometry.header.stamp = thisImu.header.stamp;
        //odometry.header.frame_id = odometryFrame;
        //odometry.child_frame_id = "odom_imu";

        //// transform imu pose to ldiar
        //gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
        //gtsam::Pose3 lidarPose = imuPose.compose(imu2Lidar);

        //odometry.pose.pose.position.x = lidarPose.translation().x();
        //odometry.pose.pose.position.y = lidarPose.translation().y();
        //odometry.pose.pose.position.z = lidarPose.translation().z();
        //odometry.pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
        //odometry.pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
        //odometry.pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
        //odometry.pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();
        //
        //odometry.twist.twist.linear.x = currentState.velocity().x();
        //odometry.twist.twist.linear.y = currentState.velocity().y();
        //odometry.twist.twist.linear.z = currentState.velocity().z();
        //odometry.twist.twist.angular.x = thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
        //odometry.twist.twist.angular.y = thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
        //odometry.twist.twist.angular.z = thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
        //pubImuOdometry.publish(odometry);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "roboat_loam_lidar");
    
    IMUPreintegration ImuP;

    ROS_INFO("\033[1;32m----> LiDAR IMU Preintegration Started.\033[0m");
    
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    
    return 0;
}
