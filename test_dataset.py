"""Headless tests: WAV storage, labels and privacy-preserving ZIP."""
import tempfile
import unittest
import wave
from pathlib import Path
from zipfile import ZipFile
from export_zip import make_dataset_zip
from recorder_gui import RecorderStore


class DatasetTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.store = RecorderStore(self.root / "data")
        self.speech = b"\x10\x27" * 16000

    def test_save_and_zip(self):
        member = self.store.register("speakers", "Người A")
        a = self.store.save_pcm("positive", member["id"], self.speech)
        b = self.store.save_pcm("negative", member["id"], self.speech)
        self.assertEqual(a.parent.name, b.parent.name)
        out = self.root / "out.zip"
        count, _ = make_dataset_zip("c01", out, self.store.data_dir / "raw")
        self.assertEqual(count, 2)
        with ZipFile(out) as archive:
            self.assertEqual(set(archive.namelist()), {
                "dataset/raw/positive/speaker_c01_001/001.wav",
                "dataset/raw/negative/speaker_c01_001/001.wav",
            })
            self.assertIsNone(archive.testzip())
        with self.assertRaises(FileExistsError):
            make_dataset_zip("c01", out, self.store.data_dir / "raw")

    def test_no_registry_in_zip_and_no_overwrite_wav(self):
        person = self.store.register("speakers", "Tên riêng tư")
        first = self.store.save_pcm("positive", person["id"], self.speech)
        second = self.store.save_pcm("positive", person["id"], self.speech)
        self.assertNotEqual(first, second)
        count, dest = make_dataset_zip("c01", self.root / "out.zip", self.store.data_dir / "raw")
        self.assertEqual(count, 2)
        with ZipFile(dest) as archive:
            self.assertFalse(any("registry" in name for name in archive.namelist()))
            with wave.open(archive.open("dataset/raw/positive/speaker_c01_001/001.wav"), "rb") as wav:
                self.assertEqual((wav.getnchannels(), wav.getframerate()), (1, 16000))


if __name__ == "__main__":
    unittest.main()
