"""Pure Python unit tests for kiosk AI. No ROS, Qt, GPU or external API needed."""
import json
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from robot_ui.kiosk.knowledge import KnowledgeRetriever, chunk_document
from robot_ui.kiosk.llm_client import LLMWorker, call_llm, format_robot_context


class FakeEncoder:
    def encode(self, texts, **kwargs):
        return [[1.0 if "pin" in text.casefold() else 0.0,
                 1.0 if "cảm biến" in text.casefold() else 0.0]
                for text in texts]


class FakeResponse:
    def __enter__(self):
        return self
    def __exit__(self, *args):
        return False
    def read(self):
        return b'{"choices":[{"message":{"content":"Xin chao"}}]}'


class TestKioskAI(unittest.TestCase):
    def test_chunking_with_overlap(self):
        text = " ".join(["Robot co cam bien va pin."] * 90)
        chunks = chunk_document(text, "robot.txt", size=170, overlap=30)
        self.assertGreater(len(chunks), 3)
        self.assertTrue(all(0 < len(c.text) <= 170 for c in chunks))
        self.assertEqual([c.number for c in chunks], list(range(1, len(chunks) + 1)))

    def test_lexical_search(self):
        with tempfile.TemporaryDirectory() as folder:
            Path(folder, "pin.txt").write_text("Pin robot cần sạc khi mức pin thấp.", encoding="utf8")
            Path(folder, "map.txt").write_text("Bản đồ robot để dẫn đường.", encoding="utf8")
            retriever = KnowledgeRetriever(Path(folder))
            matches = retriever.retrieve("Pin robot", top_k=1)
            self.assertEqual(matches[0].source, "pin.txt")
            self.assertIn("Nguồn: pin.txt", retriever.context("Pin robot"))

    def test_optional_dense_embeddings(self):
        with tempfile.TemporaryDirectory() as folder:
            Path(folder, "pin.txt").write_text("Độ đầy của pin.", encoding="utf8")
            Path(folder, "sensors.txt").write_text("Hệ thống cảm biến LiDAR.", encoding="utf8")
            retriever = KnowledgeRetriever(Path(folder), embedder=FakeEncoder())
            self.assertEqual(retriever.retrieve("Cảm biến", top_k=1)[0].source, "sensors.txt")

    def test_stale_or_missing_robot_state(self):
        fresh = format_robot_context({"battery_percent": 42}, 98, now=100, max_age=4)
        stale = format_robot_context({"battery_percent": 42}, 90, now=100, max_age=4)
        self.assertIn("42", fresh)
        self.assertNotIn("42", stale)
        self.assertIn("KHÔNG CÒN MỚI", stale)
        self.assertIn("KHÔNG CÓ DỮ LIỆU", format_robot_context({}, None, now=100))

    def test_llm_history_and_source_payload(self):
        class StubRetriever:
            def context(self, q):
                return "[Nguồn: test.txt#1] Robot phục vụ"
        captured = {}
        def fake_urlopen(req, timeout):
            captured.update(json.loads(req.data.decode("utf8")))
            return FakeResponse()
        with patch("robot_ui.kiosk.llm_client.urllib.request.urlopen", fake_urlopen):
            result = call_llm(
                "Pin sao?", {"battery_percent": 80},
                api_key="test", model="test",
                history=[{"role": "user", "content": "Chào"},
                         {"role": "assistant", "content": "Chào bạn"}],
                status_updated_at=time.monotonic(),
                retriever=StubRetriever())
        self.assertEqual(result, "Xin chao")
        self.assertEqual([x["role"] for x in captured["messages"]],
                         ["system", "user", "assistant", "user"])
        self.assertIn("80", captured["messages"][0]["content"])
        self.assertIn("test.txt", captured["messages"][0]["content"])

    def test_reset_blocks_late_response(self):
        started, release, finished = threading.Event(), threading.Event(), threading.Event()
        received = []
        def slow_llm(*args, **kwargs):
            started.set()
            release.wait(2)
            return "Old user answer"
        worker = LLMWorker(
            on_response=lambda session, txt: (received.append(txt), finished.set()),
            on_error=lambda session, err: (received.append(err), finished.set()),
            api_key="test")
        with patch("robot_ui.kiosk.llm_client.call_llm", slow_llm):
            self.assertTrue(worker.send("Hello"))
            self.assertTrue(started.wait(1))
            worker.reset_session()
            release.set()
            self.assertFalse(finished.wait(.2))
        self.assertEqual(received, [])
        self.assertEqual(worker.session_id, 1)


if __name__ == "__main__":
    unittest.main()
