# MORAI 데이터

대회 데이터를 버전별로 보관합니다.

```text
morai_ws/data/
├─ routes/              대회 주행 경로
├─ mgeo/<지도버전>/      공식 MGeo 원본
└─ competition_refs/    대회 공지·참고 이미지
```

MGeo 원본 JSON의 이름을 바꾸지 않습니다. 지도 버전, 출처, 좌표계 메모,
파일 checksum을 함께 기록합니다.
