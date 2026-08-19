# PhysiCar API 참고

- 원문: <https://physicar.ai/ko/learn/reference/physicar-api/>
- 확인일: 2026-08-19

PhysiCar의 클라우드 AI 서비스는 텍스트 Chat API와 음성 Realtime API로 구분된다.

## Chat

- 엔드포인트: `POST https://api.physicar.ai/chat`
- 모델 목록: `GET https://api.physicar.ai/chat/models`
- 인증: `Authorization: Bearer <token>`
- 입력: `user_message`, `prompt`, 선택적인 `chat_id`와 `turn`
- 스트리밍: Server-Sent Events
- 주요 이벤트: `text`, `tool_call`, `audio`, `done`, `error`

도구 호출이 반환되면 클라이언트가 직접 도구를 실행하고 다음 턴의
`tool_call_outputs`에 같은 `call_id`와 실행 결과를 넣는다. 프롬프트는 요청마다
보내거나 서버에 저장한 뒤 `prompt_id`로 참조할 수 있다. MyApp에서는 요청 시점마다
`physicarSession.token()`을 읽어야 하며 공유 환경에서 토큰을 전역 캐시하면 안 된다.

## Realtime

- 엔드포인트: `wss://api.physicar.ai/realtime`
- 모델 목록: `GET https://api.physicar.ai/realtime/models`
- 브라우저 인증: WebSocket 서브프로토콜 `token.<토큰>`
- 전송: JSON 텍스트 프레임, 오디오는 base64 PCM16
- 시작 이벤트: `session.start`
- 입력 이벤트: `audio`, `text`, `image`, `tool_result`
- 출력 이벤트: `session.ready`, `audio.delta`, `transcript.delta`, `tool_call`,
  `turn.complete`, `interrupted`, `error`, `session.end`

샘플레이트는 고정값으로 추정하지 않고 `session.ready.audio_config`를 따른다.
`interrupted` 이벤트를 받으면 재생 중인 오디오를 폐기해야 한다. 세션 시간·유휴 제한,
동시 연결 수와 비용 제한은 운영 정책이므로 공식 원문을 최종 기준으로 확인한다.

