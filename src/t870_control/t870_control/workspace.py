"""저장소 루트 찾기.

launch 파일과 노드가 경로 CSV(gps_recordings/)를 찾을 때 쓴다. 이 모듈의 위치에서
위로 올라가며 gps_recordings 폴더가 있는 곳을 저장소 루트로 본다.

소스 트리(src/), colcon 빌드 · 설치 폴더(build_t870/, install_t870/)가 모두 저장소
안에 있으므로, 심볼릭 링크 설치든 복사 설치든 같은 저장소를 가리킨다. 경로 CSV 는
현장에서 QGIS 로 고친 뒤 바로 쓰므로 패키지 share 에 설치하지 않고 저장소의 파일을
직접 읽는다.
"""

import os
from pathlib import Path


def find_workspace():
    """gps_recordings 가 있는 저장소 루트. 못 찾으면 현재 작업 폴더."""
    for parent in Path(__file__).resolve().parents:
        if (parent / 'gps_recordings').is_dir():
            return str(parent)
    return os.getcwd()
