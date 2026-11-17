#!/usr/bin/env python3
"""서보별 펄스폭 실시간 측정 (ST-Link/SWD 경유 — CAN 어댑터·UART 불필요)

각 서보가 실제로 도는 양 끝의 펄스폭(µs)을 직접 돌려 보며 재고, src/main.c의
servo_us_at_0[] / servo_us_at_180[] 표에 반영한다. 반영한 뒤에는 CAN으로 0.0°를 보내면
"0° 끝"으로, 180.0°를 보내면 "180° 끝"으로 간다.

사용법: 보드를 USB(ST-Link)로 연결하고, 서보 전원을 켠 뒤
    tools/calibrate.py            # 대화형 측정
    tools/calibrate.py --check    # 서보를 움직이지 않고 연결·펌웨어 상태만 점검

측정 순서 (서보마다): 0~9로 서보 선택 → w/s로 한쪽 끝까지 천천히 돌림 → 끝에서 윙 소리가
나거나 뜨거우면 한두 걸음 되돌림 → [ 로 기록 → 반대쪽 끝까지 돌려 ] 로 기록.
"뒤집어 달린" 서보는 [ 와 ] 를 반대로 기록하면 방향이 뒤집힌다.

주의:
  - 이 도구는 SWD로 CCR 레지스터를 직접 쓴다. 첫 조작을 하는 순간 그 서보는 현재 위치에서
    해당 펄스 위치로 바로 이동한다 (펄스가 없던 서보는 힘이 빠져 있었을 수 있다).
  - CAN으로 명령이 들어오면 펌웨어가 CCR을 다시 덮어쓴다. ROS 노드는 끄고 쓴다.
  - 종료하면 건드린 서보는 시작할 때 값으로 복원한다.
"""
from __future__ import annotations

import argparse
import os
import re
import select
import shutil
import socket
import subprocess
import sys
import tempfile
import termios
import time
import tty
from contextlib import contextmanager
from pathlib import Path
from typing import NamedTuple

ROOT = Path(__file__).resolve().parent.parent
MAIN_C = ROOT / "src" / "main.c"
OPENOCD_DIR = Path.home() / ".platformio/packages/tool-openocd"
PIO = Path.home() / ".platformio/penv/bin/pio"

US_CENTER = 1500
MIN_SPAN_US = 200            # 이보다 좁으면 거의 안 움직이는 보정으로 보고 경고한다
EXPECT_PSC, EXPECT_ARR = 83, 19999   # 타이머 1틱 = 1µs, 주기 20ms (Stm32servosepc.md 5장)

# STM32F407 타이머 레지스터 (stm32f407xx.h의 TIM_TypeDef 오프셋과 같음)
TIM_BASE = {2: 0x40000000, 3: 0x40000400, 4: 0x40000800}
CCR_OFF = {1: 0x34, 2: 0x38, 3: 0x3C, 4: 0x40}
CR1_OFF, CCER_OFF, PSC_OFF, ARR_OFF = 0x00, 0x20, 0x28, 0x2C

STEPS = {"w": 10, "s": -10, "W": 1, "S": -1, ".": 1, ",": -1, "e": 50, "d": -50}

HELP = """\
키 (한 글자씩, Enter 불필요. 한글 입력 상태면 영문으로 바꿀 것):
  w / s       : 펄스 +10 / -10 µs      W / S (또는 . / ,) : +1 / -1 µs
  e / d       : 펄스 +50 / -50 µs      c : 1500 µs(중앙)로 이동
  [           : 지금 펄스를 "0° 끝"으로 기록
  ]           : 지금 펄스를 "180° 끝"으로 기록
  0 ~ 9       : motor_id 선택          n / p : 다음 / 이전
  r           : 선택한 서보 펄스 끄기 (CCR=0, 힘 빼기)
  q           : 종료 → 결과 출력, main.c 반영 여부 질문
"""


class ParseError(Exception):
    pass


class Servo(NamedTuple):
    id: int
    name: str
    pin: str
    tim: int
    ch: int

    @property
    def addr(self) -> int:
        return TIM_BASE[self.tim] + CCR_OFF[self.ch]


# ───────────────────────── main.c 파싱 / 패치 ─────────────────────────

def define_int(src: str, name: str) -> int:
    m = re.search(rf"^#define\s+{name}\s+(\d+)U?\b", src, re.M)
    if not m:
        raise ParseError(f"main.c에서 '#define {name}'을 찾지 못함")
    return int(m.group(1))


def parse_servos(src: str) -> list[Servo]:
    """servo[] 표에서 motor_id → (타이머, 채널)을 읽는다. 핀 이름은 줄 끝 주석에서 읽는다."""
    count = define_int(src, "SERVO_COUNT")
    m = re.search(r"static const ServoChannel servo\[SERVO_COUNT\]\s*=\s*\{(.*?)^\};", src, re.S | re.M)
    if not m:
        raise ParseError("main.c에서 servo[] 표를 찾지 못함")
    body = m.group(1)
    chans = re.findall(r"\{\s*&htim(\d)\s*,\s*TIM_CHANNEL_(\d)\s*\}", body)
    if len(chans) != count:
        raise ParseError(f"servo[] 항목이 {len(chans)}개인데 SERVO_COUNT는 {count}")
    names: dict[int, tuple[str, str]] = {}
    for line in body.splitlines():
        if "//" in line:
            for i, name, pin in re.findall(r"(\d+)\s+([A-Za-z0-9_]+)\s+(P[A-K]\d+)", line.split("//", 1)[1]):
                names[int(i)] = (name, pin)
    servos = []
    for i, (t, c) in enumerate(chans):
        tim, ch = int(t), int(c)
        if tim not in TIM_BASE or ch not in CCR_OFF:
            raise ParseError(f"servo[{i}]: TIM{tim} 채널 {ch}은(는) 이 도구가 모르는 조합")
        name, pin = names.get(i, (f"motor{i}", "?"))
        servos.append(Servo(i, name, pin, tim, ch))
    return servos


def parse_cal(src: str, count: int) -> tuple[list[int], list[int]]:
    out = []
    for name in ("servo_us_at_0", "servo_us_at_180"):
        m = re.search(rf"static const uint16_t {name}\[SERVO_COUNT\]\s*=\s*\{{([^}}]*)\}};", src)
        if not m:
            raise ParseError(f"main.c에서 {name}[] 표를 찾지 못함")
        vals = [int(v) for v in re.findall(r"\d+", m.group(1))]
        if len(vals) != count:
            raise ParseError(f"{name}[]에 값이 {len(vals)}개인데 SERVO_COUNT는 {count}")
        out.append(vals)
    return out[0], out[1]


def patch_cal(src: str, at0: list[int], at180: list[int]) -> str:
    for name, vals in (("servo_us_at_0", at0), ("servo_us_at_180", at180)):
        pat = re.compile(rf"(static const uint16_t {name}\[SERVO_COUNT\]\s*=\s*\{{)[^}}]*(\}};)")
        body = " " + ", ".join(str(v) for v in vals) + " "
        src, n = pat.subn(lambda m: m.group(1) + body + m.group(2), src)
        if n != 1:
            raise ParseError(f"main.c에서 {name}[] 표를 정확히 한 곳 찾지 못함 (찾은 수: {n})")
    return src


# ───────────────────────── OpenOCD (telnet) ─────────────────────────

def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class OpenOCD:
    """openocd를 백그라운드로 띄우고 telnet으로 명령을 전달한다 (빈 포트를 골라 쓴다)."""

    PROMPT = b"\r> "
    IAC = re.compile(rb"\xff[\xfb-\xfe].|\xff.")

    def __init__(self):
        exe = OPENOCD_DIR / "bin/openocd"
        scripts = OPENOCD_DIR / "openocd/scripts"
        if not exe.exists():
            found = shutil.which("openocd")
            if not found:
                sys.exit("openocd를 찾지 못함 (PlatformIO tool-openocd 또는 PATH)")
            exe, scripts = Path(found), None
        port = free_port()
        cmd = [str(exe)]
        if scripts:
            cmd += ["-s", str(scripts)]
        cmd += ["-f", "interface/stlink.cfg", "-f", "target/stm32f4x.cfg",
                "-c", "gdb_port disabled", "-c", "tcl_port disabled", "-c", f"telnet_port {port}"]
        self.log = tempfile.TemporaryFile()
        self.proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=self.log)
        self.sock = None
        for _ in range(60):
            if self.proc.poll() is not None:
                self.log.seek(0)
                tail = self.log.read().decode(errors="replace")[-600:]
                sys.exit("openocd 실행 실패 — 보드 USB 연결, 다른 디버거(PIO Debug 등) 종료를 확인\n" + tail)
            try:
                self.sock = socket.create_connection(("127.0.0.1", port), timeout=2)
                break
            except OSError:
                time.sleep(0.25)
        if self.sock is None:
            self.close()
            sys.exit("openocd telnet 연결 실패")
        self.sock.settimeout(3)
        self._read_prompt()

    def _read_prompt(self) -> str:
        buf = b""
        while not buf.endswith(self.PROMPT):
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("openocd 연결 끊김")
            buf += chunk
        buf = self.IAC.sub(b"", buf).replace(b"\x00", b"").replace(b"\r", b"")
        return buf.decode("ascii", "replace")

    def cmd(self, line: str) -> str:
        self.sock.sendall(line.encode() + b"\n")
        text = self._read_prompt()
        err = re.search(r"^.*(?:error|failed|invalid command).*$", text, re.M | re.I)
        if err:
            raise RuntimeError(f"openocd '{line}' 실패: {err.group(0).strip()}")
        return text

    def write32(self, addr: int, value: int) -> None:
        # mww는 접근에 실패해도 응답에 오류를 싣지 않는다(로그로만 나감) → 읽어서 확인한다
        self.cmd(f"mww 0x{addr:08X} 0x{value:08X}")
        got = self.read32(addr)
        if got != value:
            raise RuntimeError(f"0x{addr:08X}에 {value}를 썼는데 읽으면 {got} — 쓰기 실패 또는 다른 쪽이 덮어쓰는 중")

    def read32(self, addr: int) -> int:
        # 응답 첫 줄은 입력 에코, 값 줄은 "0x40000434: 000005dc " 형식
        text = self.cmd(f"mdw 0x{addr:08X}")
        m = re.search(rf"^0x{addr:08x}:\s+([0-9a-f]{{8}})\b", text, re.M | re.I)
        if not m:
            raise RuntimeError(f"레지스터 읽기 실패 0x{addr:08X}: {text!r}")
        return int(m.group(1), 16)

    def close(self) -> None:
        try:
            if self.sock:
                self.sock.sendall(b"exit\n")
                self.sock.close()
        except OSError:
            pass
        self.proc.terminate()
        try:
            self.proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.proc.kill()
        self.log.close()


# ───────────────────────── 펌웨어 상태 점검 ─────────────────────────

def check_firmware(ocd, servos: list[Servo]) -> list[str]:
    """이 도구는 'CCR 값 = 펄스폭(µs)'을 전제한다. 보드에서 도는 펌웨어가 그렇지 않으면 문제 목록을 돌려준다."""
    problems = []
    for tim in sorted({s.tim for s in servos}):
        base = TIM_BASE[tim]
        psc, arr, cr1 = ocd.read32(base + PSC_OFF), ocd.read32(base + ARR_OFF), ocd.read32(base + CR1_OFF)
        if (psc, arr) != (EXPECT_PSC, EXPECT_ARR):
            problems.append(f"TIM{tim}: PSC={psc}, ARR={arr} (기대 {EXPECT_PSC}/{EXPECT_ARR}) — CCR이 µs가 아님")
        if not cr1 & 1:
            problems.append(f"TIM{tim}: 카운터가 멈춰 있음 (CR1.CEN=0)")
    for s in servos:
        ccer = ocd.read32(TIM_BASE[s.tim] + CCER_OFF)
        if not (ccer >> ((s.ch - 1) * 4)) & 1:
            problems.append(f"motor {s.id} ({s.name}): TIM{s.tim}_CH{s.ch} 출력이 꺼져 있음 — 이 보드의 펌웨어가 Servo_Start를 안 부름?")
    return problems


# ───────────────────────── 키 입력 ─────────────────────────

@contextmanager
def raw_keys():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        yield fd
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def read_key(fd: int) -> str:
    """한 글자. 방향키(ESC 시퀀스)는 "\\x1b", 한글 등 비ASCII는 "\\x80"으로 통째로 삼킨다 ('[' 오입력 방지)."""
    b = os.read(fd, 1)
    if not b:
        return ""
    if b == b"\x1b" or b[0] >= 0x80:
        while select.select([fd], [], [], 0.02)[0]:
            os.read(fd, 16)
        return "\x1b" if b == b"\x1b" else "\x80"
    return b.decode("ascii", "ignore")


# ───────────────────────── 측정 상태 ─────────────────────────

class Calibrator:
    def __init__(self, ocd, servos: list[Servo], us_min: int, us_max: int):
        self.ocd, self.servos = ocd, servos
        self.us_min, self.us_max = us_min, us_max
        self.orig = [ocd.read32(s.addr) for s in servos]        # 종료 때 복원할 시작 값
        self.us: list[int | None] = [v if us_min <= v <= us_max else None for v in self.orig]
        self.cal: list[dict[str, int | None]] = [{"at0": None, "at180": None} for _ in servos]
        self.touched: set[int] = set()
        self.idx = 0
        self.msg = ""

    def _write(self, value: int) -> bool:
        """선택한 서보의 CCR에 쓴다. 실패하면 화면 메시지로 알리고 상태는 바꾸지 않는다."""
        self.touched.add(self.idx)          # 일부만 써졌을 수 있으니 실패해도 종료 때 복원을 시도한다
        try:
            self.ocd.write32(self.servos[self.idx].addr, value)
        except RuntimeError as e:
            self.msg = f"쓰기 실패: {e}"
            return False
        return True

    def _set(self, value: int) -> None:
        value = max(self.us_min, min(self.us_max, value))
        if self._write(value):
            self.us[self.idx] = value

    def handle(self, k: str) -> bool:
        """키 하나를 처리한다. 종료하려면 False."""
        n = len(self.servos)
        cur = self.us[self.idx]
        if k in STEPS:
            self._set((cur if cur is not None else US_CENTER) + STEPS[k])
        elif k == "c":
            self._set(US_CENTER)
        elif k == "r":
            if self._write(0):
                self.us[self.idx] = None
                self.msg = "펄스 끔 (힘 빠짐)"
        elif k in ("[", "]"):
            if cur is None:
                self.msg = "펄스가 없는 상태 — w/s/c로 먼저 펄스를 보내세요"
            else:
                self.cal[self.idx]["at0" if k == "[" else "at180"] = cur
        elif k.isdigit() and int(k) < n:
            self.idx = int(k)
        elif k == "n":
            self.idx = (self.idx + 1) % n
        elif k == "p":
            self.idx = (self.idx - 1) % n
        elif k == "\x80":
            self.msg = "한글 입력 상태입니다 — 영문(A)으로 바꿔 주세요"
        elif k in ("q", ""):
            return False
        return True

    def status_line(self) -> str:
        s = self.servos[self.idx]
        us = self.us[self.idx]
        now = f"{us:4d} µs" if us is not None else "펄스 없음"
        a0, a180 = self.cal[self.idx]["at0"], self.cal[self.idx]["at180"]
        return (f"[motor {s.id} {s.name}/{s.pin}] 현재 {now}   "
                f"0°끝={a0 if a0 is not None else '-'}  180°끝={a180 if a180 is not None else '-'}")

    def restore(self) -> None:
        failed = []
        for i in sorted(self.touched):
            try:
                self.ocd.write32(self.servos[i].addr, self.orig[i])
            except (RuntimeError, OSError) as e:
                failed.append(f"motor {i}: {e}")
        if failed:
            print("⚠ 복원 실패 (보드를 리셋하면 모든 서보 펄스가 꺼집니다):\n  " + "\n  ".join(failed))
        elif self.touched:
            print(f"건드린 서보 {len(self.touched)}개를 시작할 때 펄스로 복원했습니다.")


def merge_results(cal, cur0: list[int], cur180: list[int]) -> tuple[list[int], list[int]]:
    """측정한 값만 현재 표에 덮어쓴 새 표를 돌려준다 (측정 안 한 서보는 그대로)."""
    new0, new180 = list(cur0), list(cur180)
    for i, c in enumerate(cal):
        if c["at0"] is not None:
            new0[i] = c["at0"]
        if c["at180"] is not None:
            new180[i] = c["at180"]
    return new0, new180


def report(servos, cal, cur0, cur180, us_min, us_max) -> tuple[list[int], list[int], int]:
    new0, new180 = merge_results(cal, cur0, cur180)
    print("\n=== 측정 결과 (main.c 현재값 → 측정값) ===")
    done = 0
    for s in servos:
        c = cal[s.id]
        if c["at0"] is None and c["at180"] is None:
            continue
        done += 1
        notes = []
        if c["at0"] is None or c["at180"] is None:
            notes.append("⚠ 한쪽만 기록 — 나머지는 현재 값 유지")
        if abs(new180[s.id] - new0[s.id]) < MIN_SPAN_US:
            notes.append(f"⚠ 범위가 {MIN_SPAN_US} µs 미만 (거의 안 움직임)")
        if new0[s.id] > new180[s.id]:
            notes.append("방향 반전")
        if us_min in (new0[s.id], new180[s.id]) or us_max in (new0[s.id], new180[s.id]):
            notes.append(f"⚠ 하드 리밋({us_min}/{us_max})에 닿음 — 실제 끝이 더 멀 수 있음")
        print(f"  motor {s.id} {s.name:<7} 0°끝 {cur0[s.id]:>4} → {new0[s.id]:<4}  "
              f"180°끝 {cur180[s.id]:>4} → {new180[s.id]:<4}  {' / '.join(notes)}")
    return new0, new180, done


def apply_and_flash(path: Path, new0: list[int], new180: list[int]) -> None:
    with open(path, encoding="utf-8", newline="") as f:
        src = f.read()
    patched = patch_cal(src, new0, new180)
    with open(path, "w", encoding="utf-8", newline="") as f:
        f.write(patched)
    shown = path.relative_to(ROOT) if path.is_relative_to(ROOT) else path
    print(f"✔ {shown} 의 servo_us_at_0[] / servo_us_at_180[] 갱신 완료")
    if input("바로 빌드+업로드할까요? (보드가 리셋되어 서보는 첫 CAN 명령 전까지 대기) [y/N] ").strip().lower() == "y":
        pio = str(PIO) if PIO.exists() else "pio"
        subprocess.run([pio, "run", "-t", "upload"], cwd=ROOT, check=False)


def interactive(ocd, servos, us_min, us_max) -> Calibrator:
    cal = Calibrator(ocd, servos, us_min, us_max)
    print(HELP)
    with raw_keys() as fd:
        try:
            while True:
                print("\r\033[K" + cal.status_line(), end="", flush=True)
                try:
                    k = read_key(fd)
                except KeyboardInterrupt:
                    break
                keep = cal.handle(k)
                if cal.msg:
                    print("\r\033[K" + cal.msg)
                    cal.msg = ""
                if not keep:
                    break
        finally:
            print()
            cal.restore()
    return cal


def main() -> None:
    ap = argparse.ArgumentParser(description="서보별 펄스폭 측정 (SWD)")
    ap.add_argument("--check", action="store_true", help="서보를 움직이지 않고 연결·펌웨어 상태만 점검")
    ap.add_argument("--force", action="store_true", help="펌웨어 점검에서 문제가 나와도 계속")
    ap.add_argument("--main-c", type=Path, default=MAIN_C, help="수정할 main.c (기본: src/main.c)")
    args = ap.parse_args()

    try:
        src = args.main_c.read_text(encoding="utf-8")
        servos = parse_servos(src)
        cur0, cur180 = parse_cal(src, len(servos))
        us_min, us_max = define_int(src, "SERVO_MIN_US"), define_int(src, "SERVO_MAX_US")
    except (OSError, ParseError) as e:
        sys.exit(f"main.c 읽기 실패: {e}")

    if not args.check and not (sys.stdin.isatty() and sys.stdout.isatty()):
        sys.exit("대화형 도구입니다 — 터미널에서 직접 실행하세요 (점검만: --check)")

    print("openocd 연결 중...")
    ocd = OpenOCD()
    try:
        problems = check_firmware(ocd, servos)
        if args.check:
            print("\nmotor  이름     핀    타이머/채널   CCR 주소      현재 CCR(µs)  main.c 0°끝/180°끝")
            for s in servos:
                print(f"  {s.id:<4} {s.name:<8} {s.pin:<5} TIM{s.tim}_CH{s.ch}      0x{s.addr:08X}   "
                      f"{ocd.read32(s.addr):>6}       {cur0[s.id]}/{cur180[s.id]}")
            print("\n" + ("펌웨어 상태 OK (PSC/ARR/카운터/채널 출력 모두 기대대로)" if not problems
                          else "문제:\n  " + "\n  ".join(problems)))
            sys.exit(1 if problems else 0)
        if problems:
            print("⚠ 보드의 펌웨어가 이 도구의 전제와 다릅니다:\n  " + "\n  ".join(problems))
            if not args.force:
                sys.exit("서보가 엉뚱한 펄스를 받을 수 있어 중단합니다 (무시하려면 --force)")
        print("연결됨. 서보 전원이 켜져 있고 ROS 노드가 꺼져 있는지 확인하세요.\n")
        cal = interactive(ocd, servos, us_min, us_max)
    finally:
        ocd.close()

    new0, new180, done = report(servos, cal.cal, cur0, cur180, us_min, us_max)
    if done == 0:
        print("기록된 값이 없어 아무것도 반영하지 않습니다.")
        return
    print("\nmain.c에 들어갈 값:")
    print("  servo_us_at_0   = { " + ", ".join(map(str, new0)) + " }")
    print("  servo_us_at_180 = { " + ", ".join(map(str, new180)) + " }")
    if input("\nmain.c에 반영할까요? [y/N] ").strip().lower() == "y":
        apply_and_flash(args.main_c, new0, new180)


if __name__ == "__main__":
    main()
