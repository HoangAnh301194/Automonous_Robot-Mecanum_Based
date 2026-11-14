"""Pure Python unit tests for kiosk AI. No ROS, Qt, GPU or external API needed."""
import argparse
import ast
import json
import os
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from robot_ui.kiosk.knowledge import KnowledgeRetriever, chunk_document
from robot_ui.kiosk.llm_client import LLMWorker, call_llm, format_robot_context
from robot_ui.kiosk.llm_env import load_llm_env_file
from robot_ui.kiosk.router_process import _local_endpoint, start_local_9router, stop_local_9router


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
    def test_kiosk_cli_9router_command_has_matching_attribute(self):
        """Check actual CLI parser without importing ROS 2 or PyQt on CI."""
        main_path = Path(__file__).resolve().parents[1] / "robot_ui" / "kiosk" / "main.py"
        tree = ast.parse(main_path.read_text(encoding="utf-8"))
        functions = [
            node for node in tree.body
            if isinstance(node, ast.FunctionDef)
            and node.name in {"boolean_argument", "parse_args"}
        ]
        self.assertEqual(len(functions), 2)
        scope = {"argparse": argparse, "os": os}
        exec(compile(ast.Module(body=functions, type_ignores=[]), str(main_path), "exec"), scope)
        opts = scope["parse_args"]([
            "robot_ui_kiosk", "--windowed", "true",
            "--hide-cursor", "false",
            "--auto-start-9router", "true",
            "--9router-command", "",
        ])
        self.assertTrue(opts.windowed)
        self.assertFalse(opts.hide_cursor)
        self.assertTrue(opts.auto_start_9router)
        self.assertEqual(opts.router_command, "")
        opts = scope["parse_args"]([
            "robot_ui_kiosk", "--9router-command", "/usr/bin/9router --no-browser"
        ])
        self.assertEqual(opts.router_command, "/usr/bin/9router --no-browser")
        # ROS launch may omit an empty LaunchConfiguration substitution,
        # leaving the flag without a following value.
        opts = scope["parse_args"]([
            "robot_ui_kiosk", "--windowed", "true",
            "--auto-start-9router", "true",
            "--9router-command",
        ])
        self.assertEqual(opts.router_command, "")

    def test_9router_env_file_autoload_and_override(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder, "llm.local.env")
            path.write_text(
                "# 9router configuration\n"
                "export LLM_BASE_URL='http://127.0.0.1:20128/v1'\n"
                'LLM_MODEL="my-model" # inline comment\n'
                "LLM_API_KEY='test#key'\n",
                encoding="utf-8",
            )
            environ = {
                "LLM_MODEL": "previous-model",
                "LLM_API_KEY": "previous-key",
                "UNRELATED": "unchanged",
            }
            values = load_llm_env_file(path, environ=environ)
            self.assertEqual(values["LLM_BASE_URL"], "http://127.0.0.1:20128/v1")
            self.assertEqual(environ["LLM_MODEL"], "my-model")
            self.assertEqual(environ["LLM_API_KEY"], "test#key")
            self.assertEqual(environ["UNRELATED"], "unchanged")
            path.write_text("LLM_MODEL=updated-model\n", encoding="utf-8")
            load_llm_env_file(path, environ=environ)
            self.assertEqual(environ["LLM_MODEL"], "updated-model")

    def test_9router_env_file_rejects_shell_execution(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder, "llm.local.env")
            path.write_text("LLM_API_KEY=$(echo_secret)\n", encoding="utf-8")
            before = {"LLM_API_KEY": "original"}
            with self.assertRaises(ValueError):
                load_llm_env_file(path, environ=before)
            self.assertEqual(before["LLM_API_KEY"], "original")

    def test_9router_env_file_missing_and_malformed_is_atomic(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder, "llm.local.env")
            with self.assertRaises(FileNotFoundError):
                load_llm_env_file(path, environ={})
            path.write_text("LLM_MODEL=test\nLLM_API_KEY='unterminated\n", encoding="utf-8")
            environment = {"LLM_MODEL": "old-model"}
            with self.assertRaises(ValueError):
                load_llm_env_file(path, environ=environment)
            self.assertEqual(environment["LLM_MODEL"], "old-model")

    def test_9router_autostart_only_local(self):
        self.assertEqual(_local_endpoint("http://127.0.0.1:20128/v1"),
                         ("127.0.0.1", 20128))
        self.assertEqual(_local_endpoint("http://localhost:20128/v1"),
                         ("localhost", 20128))
        self.assertIsNone(_local_endpoint("http://10.42.0.1:20128/v1"))
        self.assertIsNone(_local_endpoint("not-a-url"))
        with patch("robot_ui.kiosk.router_process.subprocess.Popen") as popen:
            self.assertIsNone(start_local_9router("http://10.42.0.1:20128/v1"))
            self.assertIsNone(start_local_9router("http://localhost:20128/v1",
                                                 enabled=False))
            popen.assert_not_called()

    def test_9router_autostart_reuses_existing_server(self):
        with patch("robot_ui.kiosk.router_process._port_open", return_value=True), \
             patch("robot_ui.kiosk.router_process.subprocess.Popen") as popen:
            self.assertIsNone(start_local_9router("http://localhost:20128/v1"))
            popen.assert_not_called()

    def test_9router_autostart_cli_and_owned_shutdown(self):
        proc = Mock()
        proc.pid = 4321
        proc.poll.return_value = None
        with tempfile.TemporaryDirectory() as folder, \
             patch("robot_ui.kiosk.router_process.Path.home", return_value=Path(folder)), \
             patch("robot_ui.kiosk.router_process._port_open",
                   side_effect=[False, True]), \
             patch("robot_ui.kiosk.router_process.shutil.which",
                   side_effect=lambda name: "/usr/local/bin/9router"
                   if name == "9router" else None), \
             patch("robot_ui.kiosk.router_process.subprocess.Popen",
                   return_value=proc) as popen, \
             patch("robot_ui.kiosk.router_process.os.killpg") as killpg:
            owned = start_local_9router("http://localhost:20128/v1",
                                        wait_seconds=1)
            self.assertIs(owned, proc)
            argv = popen.call_args.args[0]
            self.assertEqual(argv, ["/usr/local/bin/9router",
                                    "--tray", "--no-browser", "--port", "20128"])
            stop_local_9router(owned)
            killpg.assert_called_once_with(4321, __import__("signal").SIGTERM)
            proc.wait.assert_called_once_with(timeout=3)

    def test_9router_autostart_existing_docker_container(self):
        inspect = Mock(returncode=0, stdout="false\n")
        with patch("robot_ui.kiosk.router_process._port_open",
                   side_effect=[False, True]), \
             patch("robot_ui.kiosk.router_process.shutil.which",
                   side_effect=lambda name: "/usr/bin/docker"
                   if name == "docker" else None), \
             patch("robot_ui.kiosk.router_process.subprocess.run",
                   side_effect=[inspect, Mock(returncode=0)]) as run, \
             patch("robot_ui.kiosk.router_process.subprocess.Popen") as popen:
            self.assertIsNone(start_local_9router("http://localhost:20128/v1",
                                                 wait_seconds=1))
            self.assertEqual(run.call_args_list[1].args[0],
                             ["/usr/bin/docker", "start", "9router"])
            popen.assert_not_called()

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

    def test_ptit_faculty_local_corpus(self):
        """The checked-in PTIT/FEE1 knowledge is discoverable without embeddings."""
        docs = Path(__file__).resolve().parents[1] / "robot_ui" / "kiosk" / "rag_docs"
        expected = {
            "ptit_hanoi_overview.txt",
            "fee1_overview_contact.txt",
            "fee1_programs_departments.txt",
            "fee1_labs_research_students.txt",
            "fee1_quick_faq.txt",
        }
        self.assertTrue(all((docs / filename).is_file() for filename in expected))
        retriever = KnowledgeRetriever(docs, embedding_model="")
        labs = retriever.retrieve("Lab Rạng Đông phòng 402 A3", top_k=4)
        self.assertIn("fee1_labs_research_students.txt", [item.source for item in labs])
        courses = retriever.retrieve("Kỹ thuật Điều khiển và Tự động hóa 7520216", top_k=4)
        self.assertIn("fee1_programs_departments.txt", [item.source for item in courses])
        self.assertIn("Nguồn:", retriever.context("Khoa Kỹ thuật Điện tử 1 ở đâu?"))

    def test_hoang_son_rag_profile_and_faq(self):
        """Teacher facts are retrievable by lexical search without external models."""
        docs = Path(__file__).resolve().parents[1] / "robot_ui" / "kiosk" / "rag_docs"
        names = ("fee1_faculty_hoang_son.txt", "fee1_faculty_hoang_son_faq.txt")
        self.assertTrue(all((docs / name).is_file() for name in names))
        profile = (docs / names[0]).read_text(encoding="utf-8")
        faq = (docs / names[1]).read_text(encoding="utf-8")
        self.assertIn("https://sea.ptit.edu.vn/ts-hoang-son/", profile)
        self.assertIn("hoangson@ptit.edu.vn", profile)
        self.assertIn("Cobot", faq)
        retriever = KnowledgeRetriever(docs, embedding_model="")
        questions = [
            "Thầy Hoàng Sơn nghiên cứu điều khiển Cobot phân tán?",
            "TS. Hoàng Sơn dạy môn Robot công nghiệp không?",
            "Hoàng Sơn bảo vệ tiến sĩ tại Đại học Tứ Xuyên năm nào?",
        ]
        for question in questions:
            sources = [part.source for part in retriever.retrieve(question, top_k=6)]
            self.assertTrue(any(name in sources for name in names), (question, sources))

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

    def test_multilingual_rag_name_meaning(self):
        """Verify RAG retrieves name meaning across 5 languages."""
        docs = Path(__file__).resolve().parents[1] / "robot_ui" / "kiosk" / "rag_docs"
        retriever = KnowledgeRetriever(docs, embedding_model="")
        queries = [
            ("ソータの名前の意味は何ですか？", "広大な空"),
            ("What does the name Souta mean?", "vast sky"),
            ("Ý nghĩa tên của robot Souta là gì?", "bầu trời rộng lớn"),
            ("飒太的名字寓意是什么？", "辽阔的天空"),
            ("소타 이름의 의미가 무엇인가요?", "드넓은 하늘"),
        ]
        for query, expected_phrase in queries:
            ctx = retriever.context(query)
            self.assertIn(expected_phrase, ctx, f"Failed to retrieve '{expected_phrase}' for query '{query}'")

    def test_llm_temperature_and_humorous_system_prompt(self):
        from robot_ui.kiosk.llm_client import SYSTEM_PROMPT, DEFAULT_TEMPERATURE
        # Temperature defaults to 0.75 for balanced humor & accuracy
        self.assertEqual(DEFAULT_TEMPERATURE, 0.75)
        # System prompt contains explicit dual-mode guidance
        self.assertIn("CÂU HỎI NGHIÊM TÚC", SYSTEM_PROMPT)
        self.assertIn("CÂU HỎI PHIẾM", SYSTEM_PROMPT)
        self.assertIn("VÍ DỤ ĐỐI THOẠI MINH HỌA", SYSTEM_PROMPT)
        self.assertIn("Mecanum", SYSTEM_PROMPT)


if __name__ == "__main__":
    unittest.main()
