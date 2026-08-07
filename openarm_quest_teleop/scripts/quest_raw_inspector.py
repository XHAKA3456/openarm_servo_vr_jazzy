#!/usr/bin/env python3
"""
Quest raw 데이터 가이드형 실측 진단기.

TCP 5454에서 개행구분 JSON을 받아, 화면 안내에 따라 정해진 동작을 수행하면
구간(phase)별로 데이터를 모아 통계를 낸다. 무엇이 오는지(스키마) + 전송률(Hz)
+ 구간별 노이즈/가동범위를 정량화한다.

사전 준비:
  1) teleop C++ 노드는 꺼둔다 (포트 5454 충돌 방지).
  2) adb reverse tcp:5454 tcp:5454  (Quest USB 연결 상태)
  3) python3 quest_raw_inspector.py
  ※ 데이터는 트리거(전송 버튼)를 눌러야 흐른다. 각 구간 내내 눌러 유지할 것.

언제든 Ctrl+C 로 중단하면 그때까지의 요약을 출력한다.
"""
import socket
import json
import math
import time
import threading
import signal
import sys
from collections import defaultdict

HOST, PORT = "0.0.0.0", 5454
PREP_SEC = 3      # 각 구간 시작 전 준비 카운트다운
REC_SEC = 7       # 각 구간 측정 시간

# (이름, 안내문) — 순서대로 진행
PHASES = [
    ("still",        "양손 컨트롤러를 '가만히' 들고 계세요  (정지 노이즈 측정)"),
    ("right_left",   "오른손을 천천히 '왼쪽'으로 움직이세요"),
    ("right_right",  "오른손을 천천히 '오른쪽'으로 움직이세요"),
    ("right_up",     "오른손을 천천히 '위'로 움직이세요"),
    ("right_down",   "오른손을 천천히 '아래'로 움직이세요"),
    ("right_fwd",    "오른손을 천천히 '앞/뒤'로 움직이세요"),
    ("right_rot",    "오른 '손목'을 천천히 좌우로 비트세요  (회전)"),
    ("left_only",    "'왼손'만 움직이세요  (오른손은 가만히)"),
    ("right_only",   "'오른손'만 움직이세요  (왼손은 가만히)"),
    ("gripper",      "'트리거(그리퍼)'를 천천히 눌렀다 떼며 변화시키세요"),
]

# ---- 공유 상태 (리더 스레드 ↔ 메인) ----
lock = threading.Lock()
records = []          # (arrival_time, flat_dict)
first_obj = [None]
running = True
connected = threading.Event()


def flatten(obj, prefix=""):
    out = {}
    if isinstance(obj, dict):
        for k, v in obj.items():
            out.update(flatten(v, f"{prefix}.{k}" if prefix else k))
    elif isinstance(obj, bool):
        out[prefix] = 1.0 if obj else 0.0
    elif isinstance(obj, (int, float)):
        out[prefix] = float(obj)
    return out


def stats(vals):
    n = len(vals)
    if n == 0:
        return None
    mean = sum(vals) / n
    var = sum((v - mean) ** 2 for v in vals) / n
    return (min(vals), max(vals), mean, math.sqrt(var), n)


def reader_thread():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(1)
    srv.settimeout(0.5)
    conn = None
    while running and conn is None:
        try:
            conn, addr = srv.accept()
            print(f"\n[연결됨] {addr}")
            connected.set()
        except socket.timeout:
            continue
        except OSError:
            srv.close()
            return
    if conn is None:
        srv.close()
        return
    conn.settimeout(0.5)
    buf = ""
    while running:
        try:
            data = conn.recv(4096)
        except socket.timeout:
            continue
        except OSError:
            break
        if not data:
            break
        buf += data.decode(errors="ignore")
        while "\n" in buf:
            line, buf = buf.split("\n", 1)
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                continue
            flat = flatten(obj)
            t = time.time()
            with lock:
                if first_obj[0] is None:
                    first_obj[0] = obj
                records.append((t, flat))
    try:
        conn.close()
    except OSError:
        pass
    srv.close()


def slice_records(t0, t1):
    with lock:
        return [(t, f) for (t, f) in records if t0 <= t <= t1]


def report_phase(name, instr, t0, t1):
    rows = slice_records(t0, t1)
    dur = t1 - t0
    print(f"\n── [{name}] {instr}")
    if not rows:
        print("   ⚠ 데이터 없음 (트리거를 안 눌렀거나 전송 안 됨)")
        return
    hz = len(rows) / dur if dur > 0 else 0
    print(f"   메시지 {len(rows)}개 / {dur:.1f}s = {hz:.1f} Hz")
    # 필드별 집계
    bykey = defaultdict(list)
    for _, f in rows:
        for k, v in f.items():
            bykey[k].append(v)
    # 변화량(range) 큰 순으로, position/euler/rotation/trigger 위주
    interesting = []
    for k, vals in bykey.items():
        s = stats(vals)
        if not s:
            continue
        rng = s[1] - s[0]
        interesting.append((rng, k, s))
    interesting.sort(reverse=True)
    shown = 0
    print(f"   {'field':<26}{'min':>10}{'max':>10}{'mean':>10}{'std':>11}")
    for rng, k, s in interesting:
        # 정지 구간은 노이즈(std) 보려고 다 의미있음 / 그 외엔 변화 있는 것만
        if name != "still" and rng < 1e-4:
            continue
        print(f"   {k:<26}{s[0]:>10.4f}{s[1]:>10.4f}{s[2]:>10.4f}{s[3]:>11.5f}")
        shown += 1
        if shown >= 16:
            print("   ... (생략)")
            break


def overall_report():
    with lock:
        allrec = list(records)
    print("\n\n========== 전체 요약 ==========")
    if not allrec:
        print("수신된 메시지가 전혀 없습니다.")
        print("점검: ① 트리거를 눌렀는지 ② adb reverse tcp:5454 ③ 앱이 127.0.0.1:5454로 보내는지")
        return
    times = [t for t, _ in allrec]
    dur = times[-1] - times[0] if len(times) > 1 else 0
    if dur > 0:
        print(f"총 {len(allrec)}개 / {dur:.1f}s / 평균 {len(allrec)/dur:.1f} Hz")
    if len(times) > 2:
        dts = [(times[i] - times[i-1]) * 1000 for i in range(1, len(times))]
        s = stats(dts)
        print(f"도착간격[ms]: min={s[0]:.1f} max={s[1]:.1f} mean={s[2]:.1f} std={s[3]:.1f} (지터)")
    tss = [f["timestamp"] for _, f in allrec if "timestamp" in f]
    if len(tss) > 2:
        d = [tss[i]-tss[i-1] for i in range(1, len(tss))]
        s = stats(d)
        print(f"payload timestamp 증분 평균={s[2]:.4f} (단위추정 s≈{s[2]:.3f}/ms≈{s[2]:.1f})")


def countdown(msg, sec):
    for r in range(sec, 0, -1):
        print(f"\r{msg} {r}...   ", end="", flush=True)
        time.sleep(1)
    print("\r" + " " * (len(msg) + 12) + "\r", end="", flush=True)


def main():
    global running

    def on_sigint(sig, frame):
        global running
        running = False
        print("\n[중단] 요약 출력 중...")
    signal.signal(signal.SIGINT, on_sigint)

    th = threading.Thread(target=reader_thread, daemon=True)
    th.start()

    print(f"[inspector] {HOST}:{PORT} 대기 중...")
    print(">>> 트리거(전송 버튼)를 누르면 시작합니다. 각 구간 내내 눌러 유지하세요. <<<")
    while running and not connected.wait(timeout=0.5):
        pass
    if not running:
        overall_report(); return
    # 첫 메시지 도착까지 대기
    while running and first_obj[0] is None:
        time.sleep(0.1)
    if running and first_obj[0] is not None:
        print("\n========== 첫 메시지 전체 구조 (실제 스키마) ==========")
        print(json.dumps(first_obj[0], indent=2, ensure_ascii=False))
        print("=====================================================")

    phase_spans = []
    for name, instr in PHASES:
        if not running:
            break
        print("\n" + "═" * 60)
        print(f">>> 다음: {instr}")
        countdown("   준비...", PREP_SEC)
        if not running:
            break
        t0 = time.time()
        countdown(f"   ▶ 측정중 ({instr})", REC_SEC)
        t1 = time.time()
        phase_spans.append((name, instr, t0, t1))
        report_phase(name, instr, t0, t1)

    print("\n\n##### 구간별 결과 다시 요약 #####")
    for name, instr, t0, t1 in phase_spans:
        report_phase(name, instr, t0, t1)
    overall_report()
    running = False


if __name__ == "__main__":
    main()
