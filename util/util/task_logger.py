from __future__ import annotations

from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

# ── PDF 저장용 Non‑GUI 백엔드 지정 ────────────────────
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import pandas as pd
from pymongo import MongoClient


class TaskLogger:
    def __init__(
        self,
        uri: str = "mongodb://localhost:27017/",
        db_name: str = "central_supply",
        collection_name: str = "task_logs",
        report_dir: str | Path = "~/ros2_ws/report",
    ):
        # ── MongoDB 연결 ───────────────────────────────
        self.client = MongoClient(uri, serverSelectionTimeoutMS=3000)
        try:
            self.client.server_info()  # 연결 확인
            self.db_alive = True
        except Exception as e:
            print(f"[TaskLogger] MongoDB 연결 실패: {e}")
            self.db_alive = False

        if self.db_alive:
            self.db = self.client[db_name]
            self.collection = self.db[collection_name]

        # ── 리포트 디렉터리 ─────────────────────────────
        self.report_dir = Path(report_dir).expanduser()
        self.report_dir.mkdir(parents=True, exist_ok=True)

        # ── 상태 변수 ─────────────────────────────────
        self.current_task_id: Optional[str] = None
        self.task_logs: Dict[str, List[dict]] = {}
        self._last_rows: List[dict] = []  # DB 실패 대비 캐시

    # ── 1. 태스크 시작 ────────────────────────────────
    def start_task(self) -> None:
        self.current_task_id = datetime.utcnow().strftime("%Y%m%dT%H%M%S%fZ")
        self.task_logs[self.current_task_id] = []

    # ── 2. 단계별 로그 ────────────────────────────────
    def log_step(self, step: str, result: str, detail: str | None = None) -> None:
        if self.current_task_id is None:
            raise RuntimeError("start_task()를 먼저 호출하세요.")
        self.task_logs[self.current_task_id].append(
            {
                "timestamp": datetime.utcnow().isoformat(timespec="seconds") + "Z",
                "step": step,
                "result": result,
                "detail": detail or "",
            }
        )

    # ── 3. 태스크 완료 ────────────────────────────────
    def complete_task(self, final_result: str = "Done") -> None:
        if self.current_task_id is None:
            raise RuntimeError("start_task()를 먼저 호출하세요.")

        self.log_step("complete", final_result)
        self._last_rows = self.task_logs[self.current_task_id].copy()  # 캐시

        if self.db_alive:
            try:
                self.collection.insert_one(
                    {"task_id": self.current_task_id, "logs": self._last_rows}
                )
            except Exception as e:
                print(f"[TaskLogger] Mongo insert failed: {e}")

        del self.task_logs[self.current_task_id]
        self.current_task_id = None

    # ── 4. 리포트 생성 ────────────────────────────────
    def make_report(self, n: int = 1) -> None:
        """항상 최신 태스크 n건만 HTML·PDF로 덮어쓰기"""
        rows: List[dict] = []

        # 4-A. MongoDB에서 가져오기
        if self.db_alive:
            tasks = list(self.collection.find().sort("task_id", -1).limit(n))
            for t in tasks:
                rows.extend(
                    {
                        "task_id": t["task_id"],
                        "timestamp": log["timestamp"],
                        "step": log["step"],
                        "result": log["result"],
                        "detail": log.get("detail", ""),
                    }
                    for log in t["logs"]
                )

        # 4-B. Mongo가 비어 있으면 캐시 사용
        if not rows and self._last_rows:
            rows = [{"task_id": "cached_recent", **log} for log in self._last_rows]

        if not rows:
            print("[TaskLogger] No data for report.")
            return

        df = pd.DataFrame(rows)

        # 조건부 한글→영어 치환 (한글이 있을 때만 수행)
        if df["step"].str.contains("인식|집기|이동|놓기").any():
            df["step"].replace(
                {
                    "YOLO 인식": "YOLO Detection",
                    "집기": "Pick",
                    "이동": "Move",
                    "놓기": "Place",
                },
                inplace=True,
            )
        if df["result"].str.contains("성공|트레이 세팅 완료").any():
            df["result"].replace(
                {
                    "성공": "Success",
                    "트레이 세팅 완료": "Tray setup complete",
                },
                inplace=True,
            )

        # ── 기존 파일 삭제 후 새로 저장 ─────────────────
        self._remove_old_reports()

        html_path = self.report_dir / "task_report.html"
        df.sort_values(["timestamp"]).to_html(
            html_path, index=False, border=0, justify="center"
        )
        print(f"HTML saved → {html_path}")

        plt.figure(figsize=(8, 4))
        df["result"].value_counts().plot(kind="bar", width=0.4)
        plt.title("Distribution of Task Results")
        plt.xlabel("Result")
        plt.ylabel("Count")
        plt.xticks(rotation=0)
        plt.tight_layout()
        pdf_path = self.report_dir / "task_report.pdf"
        plt.savefig(pdf_path)
        plt.close()
        print(f"PDF saved  → {pdf_path}")

    # ── 내부: 기존 리포트 삭제 ──────────────────────────
    def _remove_old_reports(self):
        for fn in ("task_report.html", "task_report.pdf"):
            try:
                (self.report_dir / fn).unlink()
            except FileNotFoundError:
                pass


# ── 싱글턴 헬퍼 ───────────────────────────────────
_logger_instance: Optional[TaskLogger] = None


def get_logger() -> TaskLogger:
    global _logger_instance
    if _logger_instance is None:
        _logger_instance = TaskLogger()
    return _logger_instance


# ── CLI 테스트 ────────────────────────────────────
# if __name__ == "__main__":
#     log = get_logger()
#     log.start_task()
#     for s in ["move_home", "move_target", "move_tray"]:
#         log.log_step(s, "Start")
#         log.log_step(s, "Success")
#     log.complete_task("Tray setup complete")
#     log.make_report()
