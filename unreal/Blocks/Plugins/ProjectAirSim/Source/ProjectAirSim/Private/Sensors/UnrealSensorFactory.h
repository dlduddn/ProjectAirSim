// Copyright (C) Microsoft Corporation.  
// Copyright (C) 2025 IAMAI CONSULTING CORP
//
// MIT License. All rights reserved.
// Sensorfactory that is responsible for creating sensors like UUnrealCamera,
// UUnrealImu, etc.

#pragma once

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "CoreMinimal.h"
#include "DepthLidar.h"
#include "GPULidar.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/SpringArmComponent.h"
#include "UnrealCamera.h"
#include "UnrealDistanceSensor.h"
#include "UnrealLidar.h"
#include "UnrealLogger.h"
#include "UnrealRadar.h"
#include "UnrealScene.h"
#include "UnrealSensor.h"
#include "UnrealTransforms.h"
#include "core_sim/sensors/camera.hpp"
#include "core_sim/sensors/sensor.hpp"

class PROJECTAIRSIM_API UnrealSensorFactory {
 public:
  static std::pair<std::string, UUnrealSensor*> CreateSensor(
      const microsoft::projectairsim::Sensor& SimSensor,
      USceneComponent* Parent,
      AUnrealScene* UnrealScene) {
    UnrealLogger::Log(microsoft::projectairsim::LogLevel::kTrace,
                      TEXT("[UnrealSensorFactory] Creating Sensor '%S'."),
                      SimSensor.GetId().c_str());

    auto Id = SimSensor.GetId();
    UUnrealSensor* Sensor = nullptr;

    if (SimSensor.GetType() == microsoft::projectairsim::SensorType::kCamera) {
      UnrealLogger::Log(
          microsoft::projectairsim::LogLevel::kTrace,
          TEXT("[UnrealSensorFactory] Creating CameraSensor '%S'."),
          SimSensor.GetId().c_str());

      auto& sim_camera = (microsoft::projectairsim::Camera&)SimSensor;
      USceneComponent* CameraParent = Parent;

      auto cam_settings = sim_camera.GetCameraSettings();
      // [EOIR_TAN 패치] 짐벌 처리 분기:
      //  - Chase(추적/뷰포트) 카메라: 붐(-10m) 오프셋이 커서, 원본 USpringArmComponent로
      //    위치+회전을 '수평 프레임'에서 안정화해야 부드럽게 따라옴(붐이 안 휘둘림).
      //  - 그 외(측방 Left/Right): SpringArm은 90° yaw에서 비대칭 실패 → 링크 직접 부착 +
      //    UUnrealCamera::ApplyGimbalStabilization(MoveRobotToUnrealPose에서 sim-동기 호출).
      const auto& gimbal_set = cam_settings.gimbal_setting;
      const bool bHasGimbal =
          gimbal_set.lock_pitch || gimbal_set.lock_roll || gimbal_set.lock_yaw;
      if (bHasGimbal && Id == "Chase") {
        USpringArmComponent* SpringArm =
            NewObject<USpringArmComponent>(Parent, TEXT("SpringArm0"));
        SpringArm->AttachToComponent(
            Parent, FAttachmentTransformRules::SnapToTargetIncludingScale);
        SpringArm->TargetArmLength = 0.f;
        SpringArm->bInheritPitch = !gimbal_set.lock_pitch;
        SpringArm->bInheritRoll = !gimbal_set.lock_roll;
        SpringArm->bInheritYaw = !gimbal_set.lock_yaw;
        // sim-time과 비동기인 UE DeltaTime 기반 lag는 stuttering 유발 → 비활성.
        SpringArm->bEnableCameraLag = false;
        SpringArm->bEnableCameraRotationLag = false;
        SpringArm->RegisterComponent();
        CameraParent = SpringArm;
      }

      auto camera = NewObject<UUnrealCamera>(CameraParent, Id.c_str());
      camera->Initialize(sim_camera, UnrealScene);
      camera->AttachToComponent(
          CameraParent, FAttachmentTransformRules::KeepRelativeTransform);

      // Apply camera origin offset from the parent (either the parent link or
      // the spring arm attached at the parent link if gimbal rotation
      // stabilization is configured)
      camera->SetRelativePoseFromNed(cam_settings.origin_setting);

      Sensor = camera;
    } else if (SimSensor.GetType() ==
               microsoft::projectairsim::SensorType::kImu) {
      UnrealLogger::Log(microsoft::projectairsim::LogLevel::kTrace,
                        TEXT("[UnrealSensorFactory] Received request to create "
                             "IMU Sensor '%S'."),
                        SimSensor.GetId().c_str());
    } else if (SimSensor.GetType() ==
               microsoft::projectairsim::SensorType::kAirspeed) {
      UnrealLogger::Log(microsoft::projectairsim::LogLevel::kTrace,
                        TEXT("[UnrealSensorFactory] Received request to create "
                             "Airspeed Sensor '%S'."),
                        SimSensor.GetId().c_str());
    } else if (SimSensor.GetType() ==
               microsoft::projectairsim::SensorType::kBarometer) {
      UnrealLogger::Log(microsoft::projectairsim::LogLevel::kTrace,
                        TEXT("[UnrealSensorFactory] Received request to create "
                             "Barometer Sensor '%S'."),
                        SimSensor.GetId().c_str());
    } else if (SimSensor.GetType() ==
               microsoft::projectairsim::SensorType::kMagnetometer) {
      UnrealLogger::Log(microsoft::projectairsim::LogLevel::kTrace,
                        TEXT("[UnrealSensorFactory] Received request to create "
                             "Magnetometer Sensor '%S'."),
                        SimSensor.GetId().c_str());
    } else if (SimSensor.GetType() ==
               microsoft::projectairsim::SensorType::kLidar) {
      UnrealLogger::Log(
          microsoft::projectairsim::LogLevel::kTrace,
          TEXT("[UnrealSensorFactory] Creating LidarSensor '%S'."),
          SimSensor.GetId().c_str());
      auto& sim_lidar =
          static_cast<const microsoft::projectairsim::Lidar&>(SimSensor);

      if (sim_lidar.GetLidarSettings().lidar_kind ==
          microsoft::projectairsim::LidarKind::kGPUCylindrical) {
        auto lidar = NewObject<UGPULidar>(Parent, Id.c_str());
        lidar->Initialize(sim_lidar);
        lidar->AttachToComponent(
            Parent, FAttachmentTransformRules::KeepRelativeTransform);
        Sensor = lidar;
      } else if (sim_lidar.GetLidarSettings().lidar_kind ==
                 microsoft::projectairsim::LidarKind::kDepthCylindrical) {
                auto lidar = NewObject<UDepthLidar>(Parent, Id.c_str());
        lidar->Initialize(sim_lidar);
        lidar->AttachToComponent(
            Parent, FAttachmentTransformRules::KeepRelativeTransform);
        Sensor = lidar;
      } else {
        auto lidar = NewObject<UUnrealLidar>(Parent, Id.c_str());
        lidar->Initialize(sim_lidar);
        lidar->AttachToComponent(
            Parent, FAttachmentTransformRules::KeepRelativeTransform);
        Sensor = lidar;
      }
    } else if (SimSensor.GetType() ==
               microsoft::projectairsim::SensorType::kDistanceSensor) {
      UnrealLogger::Log(
          microsoft::projectairsim::LogLevel::kTrace,
          TEXT("[UnrealSensorFactory] Creating DistanceSensor '%S'."),
          SimSensor.GetId().c_str());
      auto& sim_distance_sensor =
          static_cast<const microsoft::projectairsim::DistanceSensor&>(SimSensor);
      auto distance_sensor =
          NewObject<UUnrealDistanceSensor>(Parent, Id.c_str());
      distance_sensor->Initialize(sim_distance_sensor);
      distance_sensor->AttachToComponent(
          Parent, FAttachmentTransformRules::KeepRelativeTransform);
      Sensor = distance_sensor;
    } else if (SimSensor.GetType() ==
               microsoft::projectairsim::SensorType::kDistanceSensor) {
      UnrealLogger::Log(
          microsoft::projectairsim::LogLevel::kTrace,
          TEXT("[UnrealSensorFactory] Creating DistanceSensor '%S'."),
          SimSensor.GetId().c_str());
      auto& sim_distance_sensor =
          static_cast<const microsoft::projectairsim::DistanceSensor&>(SimSensor);
      auto distance_sensor =
          NewObject<UUnrealDistanceSensor>(Parent, Id.c_str());
      distance_sensor->Initialize(sim_distance_sensor);
      distance_sensor->AttachToComponent(
          Parent, FAttachmentTransformRules::KeepRelativeTransform);
      Sensor = distance_sensor;
    } else if (SimSensor.GetType() ==
               microsoft::projectairsim::SensorType::kRadar) {
      UnrealLogger::Log(
          microsoft::projectairsim::LogLevel::kTrace,
          TEXT("[UnrealSensorFactory] Creating RadarSensor '%S'."),
          SimSensor.GetId().c_str());
      auto& sim_radar =
          static_cast<const microsoft::projectairsim::Radar&>(SimSensor);
      auto radar = NewObject<UUnrealRadar>(Parent, Id.c_str());
      radar->Initialize(sim_radar);
      radar->AttachToComponent(
          Parent, FAttachmentTransformRules::KeepRelativeTransform);
      Sensor = radar;
    } else if (SimSensor.GetType() ==
               microsoft::projectairsim::SensorType::kGps) {
      UE_LOG(SimPlugin, Log,
             TEXT("Received request to create GPS Sensor '%S'."),
             SimSensor.GetId().c_str());
    } else if (SimSensor.GetType() ==
               microsoft::projectairsim::SensorType::kBattery) {
      UE_LOG(SimPlugin, Log,
             TEXT("Received request to create Battery Sensor '%S'."),
             SimSensor.GetId().c_str());
    } else {
      UnrealLogger::Log(
          microsoft::projectairsim::LogLevel::kError,
          TEXT("[UnrealSensorFactory] Unsupported sensor type '%S'."),
          SimSensor.GetId().c_str());
      throw;
    }
    return {Id, Sensor};
  }
};
