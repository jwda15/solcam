"""LCD UI 프리뷰 (tkinter) — 설치/ROS 불필요. Windows에서 바로 실행.

pygame이 안 깔리는 환경(예: Python 3.14처럼 최신이라 pygame wheel 미배포)에서도
UI를 확인하려고 만든 버전. tkinter는 Windows Python에 기본 포함이라
'pip install' 자체가 필요 없다.

실제 메뉴 상태기계(menu.py)를 그대로 import해서 동작/타이밍은 로봇과 동일하고,
그리기만 tkinter Canvas로 다시 그린다(테마/레이아웃은 hud.py와 동일하게 맞춤).

[폰 카메라 배경] (선택)
  폰을 윈도우에서 웹캠으로 잡고(안드 14+ 'USB 웹캠 모드' 또는 DroidCam/Iriun),
  ffmpeg로 프레임을 받아 배경에 깐다. cv2/pygame 불필요 — ffmpeg만 있으면 된다.
    pip install imageio-ffmpeg      # ffmpeg 바이너리 자동(또는 시스템 ffmpeg)
    python tools/ui_preview_tk.py --list-cameras          # 장치 이름 확인
    python tools/ui_preview_tk.py --camera "장치이름"      # 그 카메라를 배경으로

조작(키보드로 손동작 흉내):
  L=따봉(메뉴 열기, 1.5s 게이지)   1~4=항목 선택(꾹)   K=거꾸로 따봉(뒤로/닫기)
  R=녹화 토글   B=배터리--   ESC=종료
  ※ 모터 항목(Wheel/Lift 등)은 1.5s 후 키를 누른 채로 있으면 연속(카드 흰색).
  ※ 단발 항목은 테두리가 다 차면 0.1초 흰색 반짝 후 넘어간다.
"""
import argparse
import os
import tempfile
import shutil
import subprocess
import sys
import threading
import time
import tkinter as tk
import tkinter.font as tkfont
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ros2_gesture_node.menu import MenuStateMachine, build_menu

# ----- 테마 (hud.py와 동일) -----
BG      = "#0d0f12"
CARD    = "#1a1e24"
BASE    = "#3c465c"   # 기저 테두리
ACCENT  = "#1e90ff"   # 테마 블루(게이지+테두리)
FILL    = "#1e90ff"   # = ACCENT (테두리 차오름)
WHITE   = "#f4f6f8"
INK     = "#12141a"   # 흰 카드 위 글자색
DIM     = "#aeb4bd"
REC_RED = "#ff5a5a"
FLASH_SEC = 0.1

MODE_NAMES = {0: "IDLE", 1: "FOLLOW", 2: "ROTATE", 3: "FOLLOW2", 4: "ORBIT"}
KEY2GEST = {"l": "like", "1": "one", "2": "two", "3": "three", "4": "four",
            "k": "dislike"}
GNUM = {"one": "1", "two": "2", "three": "3", "four": "4"}

W, H = 1024, 600


# ============================================================================
#  카메라 스트림 (ffmpeg → rawvideo 파이프). cv2/numpy/pygame 불필요.
# ============================================================================
def find_ffmpeg(explicit=None):
    if explicit:
        return explicit
    p = shutil.which("ffmpeg")
    if p:
        return p
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except Exception:
        return None


def list_cameras(ffmpeg):
    """dshow(Windows) 카메라 장치 목록을 stderr로 출력."""
    if not ffmpeg:
        print("ffmpeg 없음: pip install imageio-ffmpeg 또는 ffmpeg 설치"); return
    # text=True 금지: 한글 윈도우(cp949)에서 ffmpeg UTF-8 출력 디코드 실패.
    #  바이트로 받아 UTF-8(replace)로 직접 디코드한다.
    r = subprocess.run([ffmpeg, "-hide_banner", "-list_devices", "true",
                        "-f", "dshow", "-i", "dummy"],
                       capture_output=True)
    out = (r.stderr or r.stdout or b"").decode("utf-8", errors="replace")
    print(out)


def probe_camera(ffmpeg, device):
    """해당 카메라가 지원하는 해상도/코덱(픽셀포맷)을 출력."""
    if not ffmpeg:
        print("ffmpeg 없음: pip install imageio-ffmpeg"); return
    r = subprocess.run([ffmpeg, "-hide_banner", "-f", "dshow",
                        "-list_options", "true", "-i", f"video={device}"],
                       capture_output=True)
    print((r.stderr or b"").decode("utf-8", errors="replace"))


class CameraStream:
    """ffmpeg 서브프로세스에서 고정크기 rgb24 프레임을 계속 읽어 보관."""

    def __init__(self, ffmpeg, device, w, h, fps, input_format, codec=None,
                 in_size=None, in_fps=None, in_pixfmt=None):
        self.w, self.h = w, h
        self.frame_bytes = w * h * 3
        self._latest = None
        self._seq = 0
        self._lock = threading.Lock()
        self._stop = False
        cmd = [ffmpeg, "-hide_banner", "-loglevel", "warning"]
        if input_format == "dshow":
            cmd += ["-rtbufsize", "100M"]
            # ★dshow는 장치가 지원하는 입력 모드를 정확히 열어야 함(--probe로 확인)
            if in_fps:
                cmd += ["-framerate", str(in_fps)]
            if in_size:
                cmd += ["-video_size", in_size]
            if in_pixfmt:
                cmd += ["-pixel_format", in_pixfmt]
            if codec:
                cmd += ["-vcodec", codec]
            cmd += ["-f", "dshow", "-i", f"video={device}"]
        else:
            cmd += ["-f", input_format, "-i", device]
        cmd += ["-s", f"{w}x{h}", "-pix_fmt", "rgb24", "-r", str(fps),
                "-f", "rawvideo", "-"]
        self._err = bytearray()
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                     stderr=subprocess.PIPE)
        threading.Thread(target=self._drain_err, daemon=True).start()
        threading.Thread(target=self._reader, daemon=True).start()

    def _drain_err(self):
        for line in iter(self.proc.stderr.readline, b""):
            self._err += line
            if len(self._err) > 8000:
                del self._err[:-8000]

    def error_text(self):
        return bytes(self._err).decode("utf-8", errors="replace")

    def alive(self):
        return self.proc.poll() is None

    def _reader(self):
        fb = self.frame_bytes
        while not self._stop:
            chunks = bytearray()
            while len(chunks) < fb and not self._stop:
                part = self.proc.stdout.read(fb - len(chunks))
                if not part:          # EOF/프로세스 종료
                    return
                chunks += part
            with self._lock:
                self._latest = bytes(chunks)
                self._seq += 1

    def get(self):
        with self._lock:
            return self._seq, self._latest

    def stop(self):
        self._stop = True
        try:
            self.proc.terminate()
        except Exception:
            pass


class Preview:
    def __init__(self, cam=None):
        adj = {"dist_step": 0.3, "heading_step_deg": 15, "lift_step": 0.05}
        self.sm = MenuStateMachine(build_menu(adj))
        self.mode = 1
        self.battery = 87
        self.recording = False        # 초기: 녹화 꺼짐 (R 또는 Rec 카드로 시작)
        self.rec_start = 0.0
        self.t0 = time.time()

        self.cam = cam
        self._photo = None
        self._photo_seq = -1
        self._cam_ok = cam is not None
        self._cam_t0 = time.time()
        self._cam_reported = False
        self._cam_tmp = os.path.join(tempfile.gettempdir(), "solcam_preview.ppm")

        self.held = None
        self._rel_job = None
        self._cards = {}                 # gesture -> (x,y,w,h)  (직전 프레임)
        self._prev_prog = 0.0
        self._prev_hold = ""
        self._prev_repeating = False
        self._flash_until = 0.0
        self._flash_rect = None

        self.win = tk.Tk()
        self.win.title("solcam LCD preview (tkinter)")
        self.win.configure(bg=BG)
        self.cv = tk.Canvas(self.win, width=W, height=H, bg=BG, highlightthickness=0)
        self.cv.pack()
        self.f_rec = tkfont.Font(family="Segoe UI", size=13, weight="bold")
        self.win.bind("<KeyPress>", self._press)
        self.win.bind("<KeyRelease>", self._release)
        self.win.protocol("WM_DELETE_WINDOW", self._quit)
        self._loop()

    def _quit(self):
        if self.cam:
            self.cam.stop()
        self.win.destroy()

    # ---- 입력 (자동반복 디바운스) ----
    def _press(self, e):
        k = e.keysym.lower()
        if k == "escape": self._quit(); return
        if k == "r": self._toggle_rec(); return
        if k == "b": self.battery = max(0, self.battery - 5); return
        g = KEY2GEST.get(k)
        if g is None: return
        if self._rel_job: self.win.after_cancel(self._rel_job); self._rel_job = None
        self.held = g

    def _release(self, e):
        if KEY2GEST.get(e.keysym.lower()) is None: return
        if self._rel_job: self.win.after_cancel(self._rel_job)
        self._rel_job = self.win.after(80, self._clear_held)

    def _clear_held(self): self.held = None; self._rel_job = None

    def _toggle_rec(self):
        self.recording = not self.recording
        if self.recording: self.rec_start = time.time()

    # ---- 메인 루프 (~30fps) ----
    def _loop(self):
        t = time.time() - self.t0
        for ev in self.sm.update(self.held, t):
            if ev.kind == "action" and ev.action and ev.action.kind == "mode":
                self.mode = ev.action.payload.get("mode", self.mode)
            elif (ev.kind == "action" and ev.action and ev.action.kind == "phone"
                  and ev.action.payload.get("cmd") == "record_toggle"):
                self._toggle_rec()   # 프리뷰: 폰 대신 직접 토글(라벨 ON/OFF 확인용)
        self._draw(self.sm.snapshot())
        self.win.after(33, self._loop)

    # ---- 그리기 ----
    def _rrect(self, x, y, w, h, r, **kw):
        self.cv.create_polygon(
            x+r,y, x+w-r,y, x+w,y, x+w,y+r, x+w,y+h-r, x+w,y+h,
            x+w-r,y+h, x+r,y+h, x,y+h, x,y+h-r, x,y+r, x,y,
            smooth=True, **kw)

    def _update_photo(self):
        """카메라 최신 프레임을 tk.PhotoImage 로 변환(메인스레드)."""
        if not self.cam:
            return False
        seq, raw = self.cam.get()
        if raw is None or seq == self._photo_seq:
            return self._photo is not None
        hdr = b"P6\n%d %d\n255\n" % (self.cam.w, self.cam.h)
        try:
            # Tk 의 data=base64 는 PPM 을 못 알아보는 빌드가 있어 임시파일 file= 사용
            with open(self._cam_tmp, "wb") as fp:
                fp.write(hdr)
                fp.write(raw)
            self._photo = tk.PhotoImage(file=self._cam_tmp)
            self._photo_seq = seq
            return True
        except Exception as e:
            if self._cam_ok:
                print(f"[cam] 프레임 표시 실패({e}) — 배경 자리표시로 폴백")
                self._cam_ok = False
            return self._photo is not None

    def _draw(self, snap):
        c = self.cv; c.delete("all")
        c.create_rectangle(0, 0, W, H, fill=BG, outline="")
        if self._update_photo() and self._photo is not None:
            c.create_image(W//2, H//2, image=self._photo, anchor="center")
        else:
            msg = "CAMERA"
            if self.cam:
                msg = "connecting camera..."
                # 몇 초째 프레임이 없으면 ffmpeg 에러를 콘솔에 한 번 출력
                if (not self._cam_reported and time.time() - self._cam_t0 > 4
                        and self._photo is None):
                    self._cam_reported = True
                    dead = not self.cam.alive()
                    print("[cam] 영상이 안 들어옵니다." +
                          (" (ffmpeg 종료됨)" if dead else ""))
                    print("----- ffmpeg 메시지 -----")
                    print(self.cam.error_text() or "(없음)")
                    print("-------------------------")
                    print("팁: --probe 로 지원 포맷 확인 후 --cam-codec mjpeg "
                          "와 --cam-size 를 맞춰보세요.")
                    msg = "camera failed - see console"
            c.create_text(W//2, H//2, text=msg, fill="#2a2f37",
                          font=("Segoe UI", 26, "bold"))
        self._detect_confirm(snap)
        self._topbar()
        if snap.get("state") == "MENU":
            self._dock(snap)
        else:
            c.create_text(W//2, H-58, text="show LIKE to open menu  (press L)",
                          fill=DIM, font=("Segoe UI", 12))
            if snap.get("hold_gesture") == "like":
                self._gauge(W//2, H-40, snap.get("hold_progress", 0.0))
        self._draw_flash()

    def _detect_confirm(self, snap):
        prog = snap.get("hold_progress", 0.0)
        hold = snap.get("hold_gesture", "")
        fired = (self._prev_prog >= 0.999 and not self._prev_repeating
                 and (prog < 0.5 or hold != self._prev_hold))
        if fired and self._prev_hold in self._cards:
            self._flash_rect = self._cards[self._prev_hold]
            self._flash_until = time.time() + FLASH_SEC
        self._prev_prog = prog
        self._prev_hold = hold
        self._prev_repeating = bool(snap.get("repeating", False))

    def _draw_flash(self):
        if self._flash_rect is None or time.time() >= self._flash_until:
            return
        x, y, w, h = self._flash_rect
        self._rrect(x, y, w, h, 12, fill=WHITE, outline="")

    def _topbar(self):
        c = self.cv
        c.create_oval(20, 18, 32, 30, fill=ACCENT, outline="")
        c.create_text(40, 24, text=MODE_NAMES.get(self.mode, "?"), anchor="w",
                      fill=WHITE, font=("Segoe UI", 16, "bold"))
        x = W - 20
        c.create_text(x, 24, text=f"{self.battery}%", anchor="e",
                      fill=WHITE, font=("Segoe UI", 12))
        x -= 40
        bw, bh = 28, 14
        bx0, by = x - bw, 17
        c.create_rectangle(bx0, by, x, by+bh, outline=DIM)
        c.create_rectangle(x, by+4, x+3, by+bh-4, outline=DIM, fill=DIM)
        fillw = int((bw-2) * self.battery / 100)
        col = REC_RED if self.battery <= 20 else WHITE
        c.create_rectangle(bx0+1, by+1, bx0+1+fillw, by+bh-1, fill=col, outline="")
        x = bx0 - 26
        if self.recording:
            secs = int(time.time() - self.rec_start)
            txt = f"REC {secs//60:02d}:{secs%60:02d}"
            c.create_text(x, 24, text=txt, anchor="e", fill=REC_RED, font=self.f_rec)
            dot_x = x - self.f_rec.measure(txt) - 11
            c.create_oval(dot_x-4, 20, dot_x+4, 28, fill=REC_RED, outline="")

    def _gauge(self, cx, cy, frac):
        if frac <= 0: return
        c = self.cv
        bw, bh = 160, 6
        x, y = cx - bw//2, cy
        self._rrect(x, y, bw, bh, 3, fill=BASE, outline="")
        c.create_rectangle(x, y, x+max(1, int(bw*min(1.0, frac))), y+bh,
                           fill=ACCENT, outline="")

    def _dock(self, snap):
        c = self.cv
        items = snap.get("items", [])
        if not items: return
        n = len(items)
        cw, ch, gap = 150, 64, 14
        x0 = (W - (n*cw + (n-1)*gap))//2
        y = H - ch - 28
        prog = snap.get("hold_progress", 0.0)
        hold_g = snap.get("hold_gesture", "")
        repeating = bool(snap.get("repeating", False))

        self._cards = {}
        for i, it in enumerate(items):
            x = x0 + i*(cw+gap)
            self._cards[it["gesture"]] = (x, y, cw, ch)
            active = (it["gesture"] == hold_g and prog > 0)
            if active and repeating:
                self._rrect(x, y, cw, ch, 12, fill=WHITE, outline="")
                txtcol = INK
            else:
                self._rrect(x, y, cw, ch, 12, fill=CARD, outline="")
                self._rrect(x, y, cw, ch, 12, fill="", outline=BASE, width=2)
                if active:
                    self._border_lr(x, y, cw, ch, min(1.0, prog))
                txtcol = WHITE
            label = it["label"]
            if label == "Rec":   # 녹화 토글: 상태에 따라 ON/OFF
                label = "Rec OFF" if self.recording else "Rec ON"
            c.create_text(x+cw//2, y+ch//2,
                          text=f"{GNUM.get(it['gesture'],'')}  {label}",
                          fill=txtcol, font=("Segoe UI", 15, "bold"))
        # 거꾸로 따봉(back) 게이지 — 카드 위 중앙
        if hold_g == "dislike":
            self._gauge(W//2, y-16, prog)
        path = " > ".join(snap.get("path", []))
        c.create_text(x0, y-18, text=path, anchor="w", fill=DIM,
                      font=("Segoe UI", 12))
        c.create_text(W//2, H-8, text="reverse thumbs-up (K) = back   (hold 1~4 to select)",
                      fill=DIM, font=("Segoe UI", 10))

    def _border_lr(self, x, y, w, h, frac):
        c = self.cv
        fw = w * frac
        c.create_line(x, y, x+fw, y, fill=FILL, width=3, capstyle="round")
        c.create_line(x, y+h, x+fw, y+h, fill=FILL, width=3, capstyle="round")
        c.create_line(x, y, x, y+h, fill=FILL, width=3, capstyle="round")
        if frac >= 0.999:
            c.create_line(x+w, y, x+w, y+h, fill=FILL, width=3, capstyle="round")


def main():
    ap = argparse.ArgumentParser(description="solcam LCD 프리뷰 (tkinter)")
    ap.add_argument("--camera", help="배경에 깔 카메라 장치 이름(dshow) 또는 경로(v4l2)")
    ap.add_argument("--list-cameras", action="store_true", help="카메라 장치 목록 출력 후 종료")
    ap.add_argument("--probe", action="store_true", help="--camera 의 지원 해상도/코덱 출력 후 종료")
    ap.add_argument("--cam-codec", help="입력 코덱 지정 (가상카메라는 보통 mjpeg)")
    ap.add_argument("--cam-insize", help="입력(장치) 해상도 WxH — --probe 로 확인 (예 1280x720)")
    ap.add_argument("--cam-infps", type=int, help="입력(장치) 프레임레이트 (예 30)")
    ap.add_argument("--cam-pixfmt", help="입력 픽셀포맷 (예 yuyv422, nv12)")
    ap.add_argument("--ffmpeg", help="ffmpeg 실행파일 경로(미지정 시 자동 탐색)")
    ap.add_argument("--cam-size", default="1024x576", help="WxH (기본 1024x576)")
    ap.add_argument("--cam-fps", type=int, default=12)
    ap.add_argument("--input-format", default="dshow",
                    help="ffmpeg 입력 포맷 (Windows=dshow, Linux=v4l2)")
    a = ap.parse_args()

    ffmpeg = find_ffmpeg(a.ffmpeg)
    if a.list_cameras:
        list_cameras(ffmpeg); return
    if a.probe:
        if not a.camera:
            print("--probe 는 --camera \"이름\" 과 함께 쓰세요"); return
        probe_camera(ffmpeg, a.camera); return

    cam = None
    if a.camera:
        if not ffmpeg:
            print("ffmpeg 없음: pip install imageio-ffmpeg  (또는 ffmpeg 설치)")
        else:
            try:
                w, h = (int(v) for v in a.cam_size.lower().split("x"))
                cam = CameraStream(ffmpeg, a.camera, w, h, a.cam_fps,
                                   a.input_format, a.cam_codec,
                                   a.cam_insize, a.cam_infps, a.cam_pixfmt)
                print(f"[cam] {a.camera} {w}x{h}@{a.cam_fps} 시작")
            except Exception as e:
                print(f"[cam] 시작 실패({e}) — 배경 없이 진행")
    Preview(cam=cam).win.mainloop()


if __name__ == "__main__":
    main()
