"""Chunked local knowledge search with optional offline dense embeddings."""
from __future__ import annotations
import logging
import math
import os
import re
import threading
from collections import Counter
from dataclasses import dataclass
from pathlib import Path

LOG = logging.getLogger(__name__)


@dataclass(frozen=True)
class Chunk:
    source: str
    number: int
    text: str


def chunk_document(text: str, source: str, size: int = 700, overlap: int = 100) -> list[Chunk]:
    """Bounded character windows; prefer splitting at whitespace/paragraphs."""
    if size < 100 or not 0 <= overlap < size:
        raise ValueError("Expected size >= 100 and 0 <= overlap < size")
    text = text.strip()
    chunks: list[Chunk] = []
    start = 0
    while start < len(text):
        end = min(len(text), start + size)
        if end < len(text):
            cut = max(text.rfind("\n", start + size // 2, end),
                      text.rfind(" ", start + size // 2, end))
            if cut > start:
                end = cut + 1
        fragment = text[start:end].strip()
        if fragment:
            chunks.append(Chunk(source, len(chunks) + 1, fragment))
        if end == len(text):
            break
        start = max(start + 1, end - overlap)
    return chunks


_TOKEN_PATTERN = re.compile(
    r"[\u30a0-\u30ff]+"     # Katakana loanwords (e.g. ソータ, ロボット)
    r"|[\u4e00-\u9fff]"     # Kanji / Hanzi individual semantic characters
    r"|[\u3040-\u309f]"     # Hiragana characters
    r"|[\uac00-\ud7af]+"    # Hangul syllables/words (e.g. 소타, 로봇)
    r"|\w+",                # Latin, Vietnamese with diacritics, numbers
    flags=re.UNICODE,
)


def tokenize(text: str) -> Counter[str]:
    return Counter(_TOKEN_PATTERN.findall(text.casefold()))


class KnowledgeRetriever:
    """Index .txt files only once per process; no network access or downloads."""
    def __init__(self, docs_dir: Path, embedding_model: str | None = None,
                 embedder=None, chunk_size: int = 700, overlap: int = 100):
        self.docs_dir = Path(docs_dir)
        self.embedding_model = (embedding_model if embedding_model is not None
                                else os.getenv("RAG_EMBEDDING_MODEL", ""))
        self.embedder = embedder
        self.chunk_size = chunk_size
        self.overlap = overlap
        self._lock = threading.Lock()
        self._loaded = False
        self._chunks: list[Chunk] = []
        self._tokens: list[Counter[str]] = []
        self._vectors: list[list[float]] | None = None

    def _load(self) -> None:
        if self._loaded:
            return
        with self._lock:
            if self._loaded:
                return
            if self.docs_dir.is_dir():
                for path in sorted(self.docs_dir.glob("*.txt")):
                    try:
                        self._chunks.extend(chunk_document(
                            path.read_text(encoding="utf-8"), path.name,
                            self.chunk_size, self.overlap))
                    except OSError as exc:
                        LOG.warning("Skipping %s: %s", path.name, exc)
            self._tokens = [tokenize(chunk.text) for chunk in self._chunks]
            if self._chunks and (self.embedder is not None or self.embedding_model):
                try:
                    if self.embedder is None:
                        model_dir = Path(self.embedding_model).expanduser()
                        if not model_dir.is_dir():
                            raise FileNotFoundError("RAG_EMBEDDING_MODEL must be an existing directory")
                        from sentence_transformers import SentenceTransformer
                        self.embedder = SentenceTransformer(
                            str(model_dir), device="cpu", trust_remote_code=False)
                    raw = self.embedder.encode(
                        [chunk.text for chunk in self._chunks],
                        normalize_embeddings=True, show_progress_bar=False)
                    self._vectors = [list(map(float, v)) for v in raw]
                except Exception as exc:
                    LOG.warning("Embeddings unavailable; fallback to lexical retrieval: %s", exc)
                    self.embedder, self._vectors = None, None
            self._loaded = True

    def retrieve(self, question: str, top_k: int = 4) -> list[Chunk]:
        self._load()
        if not question.strip() or top_k <= 0 or not self._chunks:
            return []
        query = tokenize(question)
        lexical = [
            sum(min(freq, doc.get(term, 0)) for term, freq in query.items()) /
            math.sqrt(sum(query.values()) * sum(doc.values()))
            if query and doc else 0.0 for doc in self._tokens
        ]
        dense = None
        if self._vectors is not None and self.embedder is not None:
            try:
                vector = list(map(float, self.embedder.encode(
                    [question], normalize_embeddings=True, show_progress_bar=False)[0]))
                dense = [sum(a * b for a, b in zip(vector, doc)) for doc in self._vectors]
            except Exception as exc:
                LOG.warning("Query embedding failed; lexical fallback: %s", exc)
        scores = []
        for i, lex in enumerate(lexical):
            sim = dense[i] if dense is not None else 0.0
            if lex <= 0 and sim < 0.25:
                continue
            score = (0.7 * max(0.0, sim) + 0.3 * lex) if dense is not None else lex
            scores.append((score, i))
        scores.sort(key=lambda pair: (-pair[0], pair[1]))
        return [self._chunks[i] for _, i in scores[:top_k]]

    def context(self, question: str, top_k: int = 4, max_chars: int = 3500) -> str:
        matches = self.retrieve(question, top_k)
        if not matches:
            return "(Không tìm thấy đoạn tài liệu phù hợp; không tự suy đoán.)"
        parts = []
        used = 0
        for chunk in matches:
            label = f"[Nguồn: {chunk.source}#{chunk.number}]\n"
            allowance = max_chars - used - len(label)
            if allowance <= 0:
                break
            fragment = chunk.text[:allowance]
            parts.append(label + fragment)
            used += len(label) + len(fragment)
        return "\n\n".join(parts)
