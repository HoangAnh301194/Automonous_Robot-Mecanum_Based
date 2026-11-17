"""Headless regression tests for suggested wakeword prompts."""
import unittest
from prompt_bank import PROMPTS, PROMPT_GUIDANCE, PROMPT_SEQUENCE, POSITIVE_FIRST_25, PromptCycler

class PromptTests(unittest.TestCase):
    def test_labels_and_coverage(self):
        self.assertEqual(set(PROMPTS), {"positive", "negative", "hard_negative", "ambient"})
        self.assertGreaterEqual(len(PROMPTS["positive"]), 20)
        self.assertGreaterEqual(len(PROMPTS["hard_negative"]), 20)
        self.assertGreaterEqual(len(PROMPTS["negative"]), 20)
        self.assertTrue(all(PROMPTS[label] for label in PROMPTS))
        self.assertEqual(set(PROMPTS), set(PROMPT_GUIDANCE))

    def test_positive_contains_keyword_and_negatives_exclude_it(self):
        self.assertTrue(all("sô ta" in text.casefold() for text in PROMPTS["positive"]))
        for label in ("negative", "hard_negative"):
            self.assertTrue(all("sô ta" not in text.casefold() for text in PROMPTS[label]))

    def test_positive_first_25_has_balanced_wake_phrases(self):
        self.assertEqual(len(POSITIVE_FIRST_25), 25)
        self.assertEqual(POSITIVE_FIRST_25.count(0), 10)
        self.assertEqual(POSITIVE_FIRST_25.count(1), 5)
        self.assertEqual(len(set(POSITIVE_FIRST_25) - {0, 1}), 10)
        self.assertEqual(set(range(len(PROMPTS["positive"]))), set(PROMPT_SEQUENCE["positive"]))

    def test_align_to_another_speaker_saved_count(self):
        p = PromptCycler()
        p.advance("positive")
        self.assertEqual(p.set_position("positive", 0), "Sô ta")
        self.assertEqual(p.set_position("positive", 2), "Sô ta ơi")
        self.assertEqual(p.set_position("positive", len(PROMPT_SEQUENCE["positive"])), "Sô ta")
        with self.assertRaises(ValueError):
            p.set_position("positive", -1)

    def test_no_duplicates(self):
        for label, phrases in PROMPTS.items():
            self.assertEqual(len(phrases), len(set(s.casefold() for s in phrases)), label)

    def test_ambient_is_not_read_aloud(self):
        self.assertTrue(all(text.startswith("Không đọc lời:") for text in PROMPTS["ambient"]))

    def test_independent_sequences_and_wraparound(self):
        p = PromptCycler()
        first_positive = p.current("positive")
        first_negative = p.current("negative")
        p.advance("positive")
        self.assertEqual(p.current("negative"), first_negative)
        for _ in range(len(PROMPT_SEQUENCE["positive"]) - 1):
            p.advance("positive")
        self.assertEqual(p.current("positive"), first_positive)
        self.assertEqual(p.position("positive"), (1, len(PROMPT_SEQUENCE["positive"])))

if __name__ == "__main__":
    unittest.main()
