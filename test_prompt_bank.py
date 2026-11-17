"""Headless regression tests for suggested wakeword prompts."""
import unittest
from prompt_bank import PROMPTS, PROMPT_GUIDANCE, PromptCycler

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
        for _ in range(len(PROMPTS["positive"]) - 1):
            p.advance("positive")
        self.assertEqual(p.current("positive"), first_positive)
        self.assertEqual(p.position("positive"), (1, len(PROMPTS["positive"])))

if __name__ == "__main__":
    unittest.main()
