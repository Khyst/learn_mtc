#!/usr/bin/env python3
"""
Gazebo 월드 내 물체들의 초기 포즈를 리셋하는 스크립트.
gz service 를 호출하여 각 모델을 SDF에 정의된 초기 위치로 되돌립니다.
"""
import subprocess
import sys
import time

WORLD_NAME = "panda_world"

# (모델 이름, x, y, z, roll, pitch, yaw) — panda_world.sdf 원본과 동일
MODELS = [
    # Bumblebee 피규어
    ("Transformers_Age_of_Extinction_Mega_1Step_Bumblebee_Figure",
     0.75, -0.25, 1.025, 0, 0, 0),
    # 커피 머그
    ("ACE_Coffee_Mug_Kristen_16_oz_cup",
     0.75, 0.0, 1.025, 0, 0, 1.5708),
    # 바구니
    ("Avengers_Thor_PLlrpYniaeB",
     0.75, 0.28, 1.025, 0, 0, 0),
]


def set_pose(model_name: str, x, y, z, roll, pitch, yaw):
    """gz service 를 호출하여 모델의 포즈를 설정합니다."""
    req = (
        f'name: "{model_name}", '
        f'position: {{x: {x}, y: {y}, z: {z}}}, '
        f'orientation: {{x: {roll}, y: {pitch}, z: {yaw}, w: 1}}'
    )
    cmd = [
        "ign", "service",
        "-s", f"/world/{WORLD_NAME}/set_pose",
        "--reqtype", "ignition.msgs.Pose",
        "--reptype", "ignition.msgs.Boolean",
        "--timeout", "2000",
        "--req", req,
    ]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            print(f"  ✓ {model_name} → ({x}, {y}, {z})")
        else:
            print(f"  ✗ {model_name}: {result.stderr.strip()}")
    except subprocess.TimeoutExpired:
        print(f"  ✗ {model_name}: timeout")
    except FileNotFoundError:
        # 'ign' 명령어가 없으면 'gz' 명령어 시도
        cmd[0] = "gz"
        cmd[5] = "gz.msgs.Pose"
        cmd[7] = "gz.msgs.Boolean"
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                print(f"  ✓ {model_name} → ({x}, {y}, {z})")
            else:
                print(f"  ✗ {model_name}: {result.stderr.strip()}")
        except Exception as e:
            print(f"  ✗ {model_name}: {e}")


def main():
    print("[reset_gz_world] Gazebo 환경 초기화 중...")
    time.sleep(0.5)  # Gazebo 서비스가 준비될 때까지 잠시 대기
    for model in MODELS:
        set_pose(*model)
    print("[reset_gz_world] 초기화 완료!")


if __name__ == "__main__":
    main()
