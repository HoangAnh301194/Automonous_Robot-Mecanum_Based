# Kiosk AI: session memory, fresh telemetry, retrieval

Scope: feature/main_interface, branched from main. Neither Nav2 configuration nor
the navigation action interface is changed.

## What changed
- The chatbot sends up to 6 completed question/answer pairs from the current visitor.
- An idle kiosk resets its chat, discards model history, and returns to Home
  after 180 seconds without mouse, touch or keyboard input.
- Generation-tagged Qt signals reject responses from prior visitor sessions,
  even if a prior LLM request finishes after a reset.
- Status is injected only if /robot_status was received within 4 seconds;
  stale and missing telemetry is marked unavailable in the prompt.
- TXT knowledge documents are split into 700-character overlapping chunks
  (100 characters overlap) and searched at query time with source IDs.
- Lexical retrieval is the default and works offline without new dependencies.
  Optional local SentenceTransformer embeddings add semantic scoring.

## Optional embedding model on the Jetson
Install a compatible sentence-transformers build in the existing Python
environment after checking JetPack, Torch and Python versions. Download a
multilingual SentenceTransformer model to local storage *before* launching
the robot (for instance paraphrase-multilingual-MiniLM-L12-v2). Then export:

    export RAG_EMBEDDING_MODEL=/absolute/path/to/local/model/directory

The code refuses to fetch a model automatically, defaults to CPU encoding,
and falls back to lexical retrieval on loading/inference errors. Because the
existing robot_info.txt is small, lexical retrieval is the recommended default
until more documents are added. For the larger corpus, profile retrieval
latency, memory and response quality on the actual Jetson Orin.

## Configuration
- KIOSK_CHAT_MAX_TURNS=6, 1-20 allowed
- KIOSK_IDLE_TIMEOUT_SECONDS=180, minimum 30 seconds
- ROBOT_STATUS_MAX_AGE_SECONDS=4, minimum 1 second
- RAG_EMBEDDING_MODEL= (blank: lexical retrieval only)

Environment variables must be exported in the shell launching the kiosk.
Nothing is written to disk as session history; UI and in-memory history reset.
ROS freshness is measured from arrival at kiosk, not from each sensor's source
timestamp: verify the publisher's own sensor health flags separately.

## Test (without ROS / Qt / GPU)
From repository root:

    PYTHONPATH=src/robot_ui python3 -m unittest discover \
       -s src/robot_ui/test -p 'test_kiosk_ai.py' -v

Then perform an integration test on the real kiosk: ask a follow-up question,
wait for the idle timeout, confirm Home/chat reset, reconnect after a delayed
LLM request and verify that no earlier reply leaks into the new session.

## Still not implemented
Streaming token output, citations rendered as clickable UI links, persistent
vector indexing, reranking, semantic evaluation dataset, speech I/O and LLM
tool-based navigation. Do not use LLM text as a robot command.


## Local PTIT / Faculty of Electronics Engineering 1 knowledge (09 Nov 2026)

The knowledge directory now contains these *additional* offline text documents:
- rag_docs/ptit_hanoi_overview.txt: PTIT history, Hanoi campuses and official portals
- rag_docs/fee1_overview_contact.txt: Faculty introduction, A2/Floor 9 location, contacts
- rag_docs/fee1_programs_departments.txt: majors, departments and leadership
- rag_docs/fee1_labs_research_students.txt: laboratories, research areas, electronics club
- rag_docs/fee1_quick_faq.txt: quick answers for recurring visitor questions

Each file includes its original PTIT/SEA website URLs and a verification date.
The faculty is the focus; the PTIT overview serves as background. When official
pages disagree on contact methods, the texts preserve both and request checking
the most current source. Do not present changing admissions/people/office-hours
information as a live lookup.

These files match the existing setup.py package_data pattern
kiosk/rag_docs/*.txt; no new Python dependency is needed. KnowledgeRetriever
builds its local index once per kiosk process; after git pull and colcon build,
restart the Kiosk process so new documents are included. Optionally configure
RAG_EMBEDDING_MODEL to a local multilingual embedding model; otherwise keyword
retrieval works automatically. The UI cannot browse these source URLs at run
time and is not guaranteed to render document citations as clickable links.


## Lecturer knowledge: Dr. Hoang Son (checked 09 Nov 2026)
- rag_docs/fee1_faculty_hoang_son.txt: verified lecturer profile, department,
  doctoral bio (with a source inconsistency noted), teaching, research,
  representative publications and 2026 ARL Cobot research role.
- rag_docs/fee1_faculty_hoang_son_faq.txt: short visitor-facing answers,
  including uncertainty about unverified lab leadership and live schedules.
- Primary sources:
  https://sea.ptit.edu.vn/ts-hoang-son/
  https://sea.ptit.edu.vn/gioi-thieu/doi-ngu-giang-vien/
  https://sea.ptit.edu.vn/gioi-thieu/co-cau-to-chuc/
  https://sea.ptit.edu.vn/wp-content/uploads/sites/18/2026/04/ARL-Lab-profi.pdf

To make these documents available, pull the feature branch, rebuild robot_ui,
and restart the kiosk process. No 9router endpoint or model configuration
changes are required solely for updating the local RAG corpus.
