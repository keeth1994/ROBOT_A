from datetime import datetime
from pathlib import Path
import threading

from robot.paths import ROOT


class DashboardRecording:
    def __init__(self):
        try:
            from playwright.sync_api import sync_playwright
        except ImportError as error:
            raise RuntimeError("Need Playwright.") from error
        self.playwright = sync_playwright
        self.output = (
            ROOT / "results/dashboard" / datetime.now().strftime("%Y-%m-%d_%H-%M-%S_%f")
        )
        self.stopped = threading.Event()
        self.thread = None

    def start(self, url):
        self.thread = threading.Thread(
            target=self.record, args=(url,), name="dashboard-recording"
        )
        self.thread.start()

    def record(self, url):
        try:
            self.output.mkdir(parents=True)
            with self.playwright() as playwright:
                browser = playwright.chromium.launch(headless=True, handle_sigint=False)
                try:
                    size = dict(width=1920, height=1440)
                    context = browser.new_context(
                        viewport=size,
                        record_video_size=size,
                        record_video_dir=str(self.output),
                    )
                    try:
                        page = context.new_page()
                        page.goto(url + "/?recording=1", wait_until="domcontentloaded")
                        print(
                            f"Recording dashboard panels to {self.output}", flush=True
                        )
                        while not self.stopped.is_set():
                            page.wait_for_timeout(200)
                    finally:
                        # Close the context to finish saving the video.
                        context.close()
                    page.video.save_as(str(self.output / "dashboard.webm"))
                    page.video.delete()
                    print(
                        f"Recording saved: {self.output / 'dashboard.webm'}", flush=True
                    )
                finally:
                    browser.close()
        except Exception as error:
            print(f"Dashboard recording failed: {error}", flush=True)
            print(
                "For browser setup, run: python -m playwright install chromium",
                flush=True,
            )

    def close(self):
        self.stopped.set()
        if self.thread:
            self.thread.join()
