#include <iostream>
#include <vector>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cmath>
#include <cctype>

#include "Locator.h"
#include "types.h"
#include "display.h"
#include "misc.h"

namespace {

bool ReadEnvDouble(const char* key, double* value) {
    const char* raw = std::getenv(key);
    if (raw == nullptr || raw[0] == '\0') {
        return false;
    }
    char* end = nullptr;
    const double parsed = std::strtod(raw, &end);
    if (end == raw) {
        std::cerr << "[Locator] Invalid env " << key << "=" << raw << std::endl;
        return false;
    }
    *value = parsed;
    return true;
}

double EnvDoubleOrDefault(const char* key, double default_value) {
    double value = default_value;
    ReadEnvDouble(key, &value);
    return value;
}

bool EnvBoolOrDefault(const char* key, bool default_value) {
    const char* raw = std::getenv(key);
    if (raw == nullptr || raw[0] == '\0') {
        return default_value;
    }
    std::string value(raw);
    for (char& ch : value) {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    if (value == "1" || value == "true" || value == "yes" || value == "on") {
        return true;
    }
    if (value == "0" || value == "false" || value == "no" || value == "off") {
        return false;
    }
    return default_value;
}

int EnvIntOrDefault(const char* key, int default_value) {
    return std::max(1, static_cast<int>(EnvDoubleOrDefault(key, default_value)));
}

void ApplyOptionalInitialPrior(PoseBox2D& constraints) {
    double prior_x = 0.0;
    double prior_y = 0.0;
    double prior_theta_deg = 0.0;
    const bool has_x = ReadEnvDouble("ROBOCUP_MARKER_PRIOR_X", &prior_x);
    const bool has_y = ReadEnvDouble("ROBOCUP_MARKER_PRIOR_Y", &prior_y);
    const bool has_theta = ReadEnvDouble("ROBOCUP_MARKER_PRIOR_THETA_DEG", &prior_theta_deg);

    if (has_x) {
        const double window = EnvDoubleOrDefault("ROBOCUP_MARKER_PRIOR_X_WINDOW_M", 1.5);
        constraints.xmin = std::max(constraints.xmin, prior_x - window);
        constraints.xmax = std::min(constraints.xmax, prior_x + window);
    }
    if (has_y) {
        const double window = EnvDoubleOrDefault("ROBOCUP_MARKER_PRIOR_Y_WINDOW_M", 1.5);
        constraints.ymin = std::max(constraints.ymin, prior_y - window);
        constraints.ymax = std::min(constraints.ymax, prior_y + window);
    }
    if (has_theta) {
        const double window_deg = EnvDoubleOrDefault("ROBOCUP_MARKER_PRIOR_THETA_WINDOW_DEG", 70.0);
        constraints.thetamin = deg2rad(prior_theta_deg - window_deg);
        constraints.thetamax = deg2rad(prior_theta_deg + window_deg);
    }
}

void ApplyInitialOwnHalfOuterSearch(PoseBox2D& constraints, const FieldDimensions& fd) {
    const double outer_x = EnvDoubleOrDefault("ROBOCUP_MARKER_INITIAL_OUTER_X_M", 0.8);
    const double outer_y = EnvDoubleOrDefault("ROBOCUP_MARKER_INITIAL_OUTER_Y_M", 0.8);
    const double midline_margin =
        EnvDoubleOrDefault("ROBOCUP_MARKER_INITIAL_MIDLINE_MARGIN_M", 0.3);

    constraints.xmin = -fd.length / 2 - outer_x;
    constraints.xmax = midline_margin;
    constraints.ymin = -fd.width / 2 - outer_y;
    constraints.ymax = fd.width / 2 + outer_y;
    constraints.thetamin = -M_PI;
    constraints.thetamax = M_PI;
}

}  // namespace

bool Locator::confirmInitialLocate(LocateResult& res) {
    if (odomCalibrated || !EnvBoolOrDefault("ROBOCUP_MARKER_INITIAL_CONFIRM", true)) {
        return true;
    }

    const double initial_residual_tolerance =
        EnvDoubleOrDefault("ROBOCUP_MARKER_INITIAL_RESIDUAL_TOLERANCE",
                           pf_locator->residualTolerance);
    if (res.residual > initial_residual_tolerance) {
        pendingInitialConfirmCount = 0;
        pendingInitialResidual = std::numeric_limits<double>::infinity();
        std::cout << "[Locator] initial pose rejected: residual=" << res.residual
                  << " > initial_residual_tolerance=" << initial_residual_tolerance << std::endl;
        return false;
    }

    const int required_count = EnvIntOrDefault("ROBOCUP_MARKER_INITIAL_CONFIRM_COUNT", 2);
    if (required_count <= 1) {
        return true;
    }

    const double max_translation =
        EnvDoubleOrDefault("ROBOCUP_MARKER_INITIAL_CONFIRM_TRANSLATION_M", 0.7);
    const double max_rotation =
        deg2rad(EnvDoubleOrDefault("ROBOCUP_MARKER_INITIAL_CONFIRM_ROTATION_DEG", 35.0));

    if (pendingInitialConfirmCount <= 0) {
        pendingInitialPose = res.pose;
        pendingInitialResidual = res.residual;
        pendingInitialConfirmCount = 1;
        std::cout << "[Locator] initial pose pending 1/" << required_count
                  << ": pose=(" << pendingInitialPose.x << "," << pendingInitialPose.y << ","
                  << rad2deg(pendingInitialPose.theta) << "deg) residual="
                  << pendingInitialResidual << std::endl;
        return false;
    }

    const double translation_delta =
        norm(res.pose.x - pendingInitialPose.x, res.pose.y - pendingInitialPose.y);
    const double rotation_delta = std::abs(toPInPI(res.pose.theta - pendingInitialPose.theta));
    if (translation_delta > max_translation || rotation_delta > max_rotation) {
        pendingInitialPose = res.pose;
        pendingInitialResidual = res.residual;
        pendingInitialConfirmCount = 1;
        std::cout << "[Locator] initial pose confirmation reset: delta=" << translation_delta
                  << "m/" << rad2deg(rotation_delta) << "deg, new residual="
                  << pendingInitialResidual << std::endl;
        return false;
    }

    ++pendingInitialConfirmCount;
    if (res.residual < pendingInitialResidual) {
        pendingInitialPose = res.pose;
        pendingInitialResidual = res.residual;
    }

    if (pendingInitialConfirmCount < required_count) {
        std::cout << "[Locator] initial pose pending " << pendingInitialConfirmCount << "/"
                  << required_count << ": delta=" << translation_delta << "m/"
                  << rad2deg(rotation_delta) << "deg residual=" << res.residual << std::endl;
        return false;
    }

    res.pose = pendingInitialPose;
    std::cout << "[Locator] initial pose confirmed " << pendingInitialConfirmCount << "/"
              << required_count << ": pose=(" << res.pose.x << "," << res.pose.y << ","
              << rad2deg(res.pose.theta) << "deg) best_residual=" << pendingInitialResidual
              << std::endl;
    pendingInitialConfirmCount = 0;
    pendingInitialResidual = std::numeric_limits<double>::infinity();
    return true;
}

void Locator::init(YamlParser _config) {

    config = _config;

    std::string field_size = config.ReadStringFromYaml("field_size");
    fd = field_size == "kid" ? FD_KIDSIZE : FD_ADULTSIZE;

    pf_locator = std::make_shared<ParticleFilter>();
    pf_locator -> init(fd, 3, 0.25, 0.5);
    pf_locator->minMarkerCnt = EnvDoubleOrDefault("ROBOCUP_MARKER_MIN_COUNT", 3.0);
    pf_locator->residualTolerance =
        EnvDoubleOrDefault("ROBOCUP_MARKER_RESIDUAL_TOLERANCE", 0.35);
    pf_locator->convergeTolerance =
        EnvDoubleOrDefault("ROBOCUP_MARKER_CONVERGE_TOLERANCE", 0.30);
    pf_locator->residualDistancePower =
        EnvDoubleOrDefault("ROBOCUP_MARKER_RESIDUAL_DISTANCE_POWER", 0.70);
    std::cout << "[Locator] thresholds min_marker_count=" << pf_locator->minMarkerCnt
              << " residual_tolerance=" << pf_locator->residualTolerance
              << " converge_tolerance=" << pf_locator->convergeTolerance
              << " residual_distance_power=" << pf_locator->residualDistancePower << std::endl;

    display_board = std::make_shared<DisplayBoard>();
    display_board->init(fd);
}

void Locator::detectProcessMarkings(const vector<GameObject> &markingObjs)
{
    const double confidenceValve = EnvDoubleOrDefault("ROBOCUP_MARKER_MIN_CONFIDENCE", 40.0);
    const double farConfidenceStart =
        EnvDoubleOrDefault("ROBOCUP_MARKER_FAR_CONFIDENCE_START_M", 3.0);
    const double farConfidenceSlope =
        EnvDoubleOrDefault("ROBOCUP_MARKER_FAR_CONFIDENCE_SLOPE", 5.0);
    const double farConfidenceMax =
        EnvDoubleOrDefault("ROBOCUP_MARKER_FAR_CONFIDENCE_MAX", 65.0);

    markings.clear();

    for (int i = 0; i < markingObjs.size(); i++)
    {
        auto marking = markingObjs[i];

        const double dynamicConfidenceValve = std::min(
            farConfidenceMax,
            confidenceValve +
                std::max(0.0, marking.range - farConfidenceStart) * farConfidenceSlope);
        if (marking.confidence < dynamicConfidenceValve)
            continue;

        if (marking.posToRobot.x < -0.5 || marking.posToRobot.x > 12.0)
            continue;

        markings.push_back(marking);
    }
}


void Locator::processDetections(const std::vector<::DetectionModule::DetectionResult> &detection_results, const Pose &p_eye2base) {



    std::vector<GameObject> gameObjects;
    for (const auto &result : detection_results) {

        // cout<<"result.class_name(): "<<result.class_name()<<" result.xyz()[0]: "<<result.xyz()[0]<<" result.xyz()[1]: "<<result.xyz()[1]<<endl;

        GameObject gObj;

        gObj.label = result.class_name();

        gObj.boundingBox.xmin = result.box()[0];
        gObj.boundingBox.ymin = result.box()[1];
        gObj.boundingBox.xmax = result.box()[2];
        gObj.boundingBox.ymax = result.box()[3];
        gObj.confidence = result.score() * 100;

        // Get object pose in camera coord
        Pose pose = Pose(result.xyz()[0], result.xyz()[1], result.xyz()[2], 0, 0, 0);

        // Get object pose in robot coord
        Pose obj_pose = p_eye2base * pose;
        auto obj_trans = obj_pose.getTranslation();

        gObj.posToRobot.x = obj_trans[2];
        gObj.posToRobot.y = -obj_trans[0];

        gObj.range = norm(gObj.posToRobot.x, gObj.posToRobot.y);
        gObj.yawToRobot = atan2(gObj.posToRobot.y, gObj.posToRobot.x);
        gObj.pitchToRobot = atan2(1.3, gObj.range);

        // Get object pose in field coord
        transCoord(
            gObj.posToRobot.x, gObj.posToRobot.y, 0,
            robotPoseToField.x, robotPoseToField.y, robotPoseToField.theta,
            gObj.posToField.x, gObj.posToField.y, gObj.posToField.z);

        gameObjects.push_back(gObj);
    }

    std::vector<GameObject> balls, goalPosts, persons, robots, obstacles, markings;
    for (int i = 0; i < gameObjects.size(); i++)
    {
        const auto &obj = gameObjects[i];
        if (obj.label == "Ball")
            balls.push_back(obj);
        if (obj.label == "Goalpost")
            goalPosts.push_back(obj);
        if (obj.label == "Person")
            persons.push_back(obj);
        if (obj.label == "Opponent")
            robots.push_back(obj);
        if (obj.label == "L" || obj.label == "T" || obj.label == "X") {
            // 目前仅识别L,T,X
            markings.push_back(obj);
        }
    }

    detectProcessMarkings(markings);
}


std::vector<FieldMarker> Locator::getMarkers()
{
    std::vector<FieldMarker> res;
    for (size_t i = 0; i < markings.size(); i++){
        auto label = markings[i].label;
        auto x = markings[i].posToRobot.x;
        auto y = markings[i].posToRobot.y;
        auto confidence = markings[i].confidence;

        char markerType = ' ';
        if (label == "L")
            markerType = 'L';
        else if (label == "T")
            markerType = 'T';
        else if (label == "X")
            markerType = 'X';
        else if (label == "P")
            markerType = 'P';
            
        res.push_back(FieldMarker{markerType, x, y, confidence});
    }
    return res;
}

bool Locator::selfLocate() {

    auto markers = getMarkers();
    if (markers.size() < 3) {
        return false;
    }

    double xMin = 0.0, xMax = 0.0, yMin = 0, yMax = 0.0, thetaMin = 0.0, thetaMax = 0.0;
    
    std::string mode = config.ReadStringFromYaml("location_mode");

    if (mode == "enter_field")
    {

        xMin = -fd.length / 2;
        xMax = -fd.circleRadius;

        std::string playerStartPos = config.ReadStringFromYaml("playerStartPos");

        if (playerStartPos == "left")
        {
            yMin = fd.width / 2;
            yMax = fd.width / 2 + 1.0;
        }
        else if (playerStartPos == "right")
        {
            yMin = -fd.width / 2 - 1.0;
            yMax = -fd.width / 2;
        }

        if (playerStartPos == "left")
        {
            thetaMin = -M_PI / 2 - M_PI / 6;
            thetaMax = -M_PI / 2 + M_PI / 6;
        }
        else if (playerStartPos == "right")
        {
            thetaMin = M_PI / 2 - M_PI / 6;
            thetaMax = M_PI / 2 + M_PI / 6;
        }
    }
    else if (mode == "face_forward")
    {
        xMin = -fd.length / 2;
        xMax = fd.length / 2;
        yMin = -fd.width / 2;
        yMax = fd.width / 2;
        thetaMin = -M_PI / 4;
        thetaMax = M_PI / 4;
    }
    else if (mode == "center")
    {
        // Full-field initial search. The old code used xMin=-0.5 and
        // xMax=fd.length/2+2 for half-field testing. On a kid field this
        // incorrectly excludes real poses such as x=-3.6 and can clamp the
        // marker localization result near x=-0.5.
        xMin = -fd.length / 2;
        xMax = fd.length / 2;
        yMin = -fd.width / 2;
        yMax = fd.width / 2;
        thetaMin = -M_PI;
        thetaMax = M_PI;
    }
    else if (mode == "normal" && !odomCalibrated)
    {
        // Robot may start just outside the field and walk in from arbitrary
        // positions on our half.  Search our half plus a small outer margin
        // instead of requiring a fixed ROBOCUP_RGBD_INIT_* pose.
        PoseBox2D initialConstraints{};
        ApplyInitialOwnHalfOuterSearch(initialConstraints, fd);
        xMin = initialConstraints.xmin;
        xMax = initialConstraints.xmax;
        yMin = initialConstraints.ymin;
        yMax = initialConstraints.ymax;
        thetaMin = initialConstraints.thetamin;
        thetaMax = initialConstraints.thetamax;
    }
    else if (mode == "normal" && odomCalibrated)
    {
        int msec = msecsSince(lastSuccessfulLocalizeTime);
        double maxDriftSpeed = 0.2; // 假设每秒最大偏差0.2米
        double maxDrift = msec / 1000.0 * maxDriftSpeed;

        xMin = max(-fd.length / 2, robotPoseToField.x - maxDrift);
        xMax = min(fd.length / 2, robotPoseToField.x + maxDrift);
        yMin = max(-fd.width / 2, robotPoseToField.y - maxDrift);
        yMax = min(fd.width / 2, robotPoseToField.y + maxDrift);
        thetaMin = robotPoseToField.theta - M_PI / 4;
        thetaMax = robotPoseToField.theta + M_PI / 4;
    } else {
        std::cout << "[ERROR]: Unsupported mode, " << mode << std::endl;
        return false;
    }

    // Locate
    PoseBox2D constraints{xMin, xMax, yMin, yMax, thetaMin, thetaMax};
    if (!odomCalibrated && mode == "normal" &&
        EnvBoolOrDefault("ROBOCUP_MARKER_USE_INITIAL_PRIOR", false)) {
        ApplyOptionalInitialPrior(constraints);
    }
    const int num_particles =
        std::max(1, static_cast<int>(EnvDoubleOrDefault(
            (!odomCalibrated && mode == "normal") ? "ROBOCUP_MARKER_INITIAL_PARTICLES"
                                                  : "ROBOCUP_MARKER_PARTICLES",
            (!odomCalibrated && mode == "normal") ? 1200.0 : 300.0)));
    std::cout << "[Locator] field_size="
              << (fd.length == FD_KIDSIZE.length && fd.width == FD_KIDSIZE.width ? "kid" : "adult")
              << " mode=" << mode
              << " constraints x=[" << constraints.xmin << "," << constraints.xmax << "]"
              << " y=[" << constraints.ymin << "," << constraints.ymax << "]"
              << " theta_deg=[" << rad2deg(constraints.thetamin) << ","
              << rad2deg(constraints.thetamax) << "]"
              << " marker_count=" << markers.size()
              << " particles=" << num_particles
              << std::endl;
    auto res = pf_locator -> locateRobot(markers, constraints, num_particles);

    // 0: Success
    // 1: Failure to generate new particles (quantity is 0)
    // 2: The residual error after convergence is unreasonable
    // 3: Not converged
    // 4: The number of Markers is insufficient
    // 5: The probabilities of all particles are too low
    std::cout << "locate result: res: " << to_string(res.code) << " time: " << to_string(res.msecs) << std::endl;

    if (res.success && !confirmInitialLocate(res)) {
        return false;
    }

    if (res.success) {
        calibrateOdom(res.pose.x, res.pose.y, res.pose.theta);
        odomCalibrated = true;
        lastSuccessfulLocalizeTime = std::chrono::high_resolution_clock::now();
    } else if (!odomCalibrated) {
        pendingInitialConfirmCount = 0;
        pendingInitialResidual = std::numeric_limits<double>::infinity();
    }
    
    std::cout << "locate success: " << to_string(res.pose.x) << " " << to_string(res.pose.y) << " " + to_string(rad2deg(res.pose.theta)) << " Dur: " << to_string(res.msecs) << std::endl;
    return res.success;
}

void Locator::calibrateOdom(double x, double y, double theta)
{
    // Calculate odomToField according to robotToOdom(by odometry) and robotToField(by locator)
    double x_or, y_or, theta_or; // or = odom to robot
    x_or = -cos(robotPoseToOdom.theta) * robotPoseToOdom.x - sin(robotPoseToOdom.theta) * robotPoseToOdom.y;
    y_or = sin(robotPoseToOdom.theta) * robotPoseToOdom.x - cos(robotPoseToOdom.theta) * robotPoseToOdom.y;
    theta_or = -robotPoseToOdom.theta;

    transCoord(x_or, y_or, theta_or,
                x, y, theta,
                odomToField.x, odomToField.y, odomToField.theta);

    // transform markers for display
    display_board->clearMarkers();
    for (auto &marking : markings) {
        transCoord(marking.posToRobot.x, marking.posToRobot.y, 0,
            robotPoseToField.x, robotPoseToField.y, robotPoseToField.theta,
            marking.posToField.x, marking.posToField.y, marking.posToField.z
        );
        display_board->addMarker(marking.label, marking.posToField.x, marking.posToField.y);
    }

}
        
