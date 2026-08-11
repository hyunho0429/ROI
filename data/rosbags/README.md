# ROSbag 저장 위치

이 폴더에는 localization·perception·통합 검증용 rosbag 파일을 저장한다.

권장 파일:

- localization_alignment.bag
- perception_run.bag
- full_validation.bag

용량이 큰 bag 파일은 Git에 추가하지 않는다. 파일 이름, MORAI 버전,
map/scenario, 센서 주기, 기록 topic, 정렬 결과를 별도의 기록 문서에 남긴다.

MORAI Launcher의 Rosbag Replay에 직접 넣는 파일은 MORAI PC의
Data/SaveFile/Rosbag 경로로 복사한다.

