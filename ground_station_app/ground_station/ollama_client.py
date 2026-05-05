"""
ground_station/ollama_client.py — Ollama HTTP API client.

Responsibilities
----------------
- Check whether the Ollama server is reachable.
- Verify the target model is available locally; pull it if missing.
- Send a prompt (system + user message) and return the response text.
- Enforce timeouts: connect 5 s, read 90 s (covers cold model load on CPU).
- Retry transient failures up to MAX_RETRIES times with exponential backoff.
- All failures raise OllamaError (a subclass of RuntimeError) so the caller
  can catch a single exception type without importing httpx.

API used
--------
Ollama's OpenAI-compatible endpoint: POST /v1/chat/completions
This is preferred over the native /api/generate because it supports the
standard messages[] format and is stable across Ollama versions.

References
----------
- Ollama OpenAI compatibility:
  https://github.com/ollama/ollama/blob/main/docs/openai.md
- Ollama native API (pull endpoint):
  https://github.com/ollama/ollama/blob/main/docs/api.md#pull-a-model
- httpx timeouts:
  https://www.python-httpx.org/advanced/timeouts/

Design constraints
------------------
- No main() in this module.
- DRONEPI_OLLAMA_HOST and DRONEPI_OLLAMA_MODEL are the only required env vars.
- OllamaError is non-fatal to the daemon; the caller logs and skips.
"""

import logging
import os
import time
from typing import Optional

import httpx

log = logging.getLogger(__name__)

# ── Configuration (all overridable via environment variables) ─────────────────

DEFAULT_HOST    = "http://localhost:11434"
DEFAULT_MODEL   = "qwen2.5:7b"
DEFAULT_TIMEOUT = 90        # seconds — covers cold model load on 32 GB CPU
CONNECT_TIMEOUT = 5         # seconds
MAX_RETRIES     = 3
RETRY_BASE_S    = 2.0       # exponential backoff base


def _ollama_host() -> str:
    return os.environ.get("DRONEPI_OLLAMA_HOST", DEFAULT_HOST).rstrip("/")


def _ollama_model() -> str:
    return os.environ.get("DRONEPI_OLLAMA_MODEL", DEFAULT_MODEL)


def _read_timeout() -> float:
    try:
        return float(os.environ.get("DRONEPI_OLLAMA_TIMEOUT_S", DEFAULT_TIMEOUT))
    except ValueError:
        return DEFAULT_TIMEOUT


# ── Exception ─────────────────────────────────────────────────────────────────

class OllamaError(RuntimeError):
    """Raised for all Ollama client failures. Non-fatal to the daemon."""


# ══════════════════════════════════════════════════════════════════════════════
# OllamaClient
# ══════════════════════════════════════════════════════════════════════════════

class OllamaClient:
    """
    Thin wrapper over the Ollama OpenAI-compatible HTTP API.

    Usage
    -----
        client = OllamaClient()
        client.ensure_ready()          # call once at daemon startup
        text = client.chat(system_prompt, user_prompt)

    All methods are synchronous. The daemon calls them from a background
    thread (not the main event loop), so blocking I/O is acceptable here.
    """

    def __init__(
        self,
        host: Optional[str] = None,
        model: Optional[str] = None,
    ) -> None:
        self.host  = host  or _ollama_host()
        self.model = model or _ollama_model()
        self._timeout = httpx.Timeout(
            connect=CONNECT_TIMEOUT,
            read=_read_timeout(),
            write=10.0,
            pool=5.0,
        )

    # ── Public API ────────────────────────────────────────────────────────────

    def is_reachable(self) -> bool:
        """
        Return True if the Ollama server responds to GET /api/tags.

        Uses a short timeout — this is a liveness probe, not a model call.
        Never raises; returns False on any error.
        """
        try:
            with httpx.Client(timeout=httpx.Timeout(5.0, connect=3.0)) as c:
                r = c.get(f"{self.host}/api/tags")
                return r.status_code == 200
        except Exception:
            return False

    def model_available(self) -> bool:
        """
        Return True if self.model is present in the local Ollama model list.

        Parses GET /api/tags response — list of {name, size, ...} objects.
        Matches by name prefix so 'qwen2.5:7b' matches 'qwen2.5:7b-instruct-q4_K_M'.

        Reference: https://github.com/ollama/ollama/blob/main/docs/api.md#list-local-models
        """
        try:
            with httpx.Client(timeout=httpx.Timeout(10.0, connect=3.0)) as c:
                r = c.get(f"{self.host}/api/tags")
                r.raise_for_status()
                models = r.json().get("models", [])
                for m in models:
                    if m.get("name", "").startswith(self.model.split(":")[0]):
                        return True
                return False
        except Exception as exc:
            log.warning(f"[OLLAMA] model_available check failed: {exc}")
            return False

    def pull_model(self) -> None:
        """
        Pull self.model from the Ollama registry.

        Streams the pull response and logs progress every ~500 MB.
        This can take several minutes on first run (~4.4 GB for qwen2.5:7b).

        Uses the native /api/pull endpoint (not OpenAI-compatible) because
        the OpenAI-compatible layer does not expose a pull operation.

        Raises OllamaError if the pull fails.

        Reference: https://github.com/ollama/ollama/blob/main/docs/api.md#pull-a-model
        """
        log.info(f"[OLLAMA] Pulling model '{self.model}' — this may take several minutes...")
        try:
            # Long timeout for pull — model download can take 5-10 min
            pull_timeout = httpx.Timeout(connect=10, read=600, write=30, pool=10)
            with httpx.Client(timeout=pull_timeout) as c:
                with c.stream(
                    "POST",
                    f"{self.host}/api/pull",
                    json={"name": self.model, "stream": True},
                ) as resp:
                    resp.raise_for_status()
                    last_logged_pct = -1
                    for line in resp.iter_lines():
                        if not line:
                            continue
                        import json as _json
                        try:
                            obj = _json.loads(line)
                        except ValueError:
                            continue
                        status = obj.get("status", "")
                        completed = obj.get("completed", 0)
                        total = obj.get("total", 0)
                        if total > 0:
                            pct = int(100 * completed / total)
                            if pct >= last_logged_pct + 10:
                                log.info(
                                    f"[OLLAMA] Pull progress: {pct}% "
                                    f"({completed // 1_048_576} / {total // 1_048_576} MB)"
                                )
                                last_logged_pct = pct
                        elif status:
                            log.info(f"[OLLAMA] Pull: {status}")
            log.info(f"[OLLAMA] Model '{self.model}' ready.")
        except OllamaError:
            raise
        except Exception as exc:
            raise OllamaError(f"Model pull failed: {exc}") from exc

    def ensure_ready(self) -> None:
        """
        Verify Ollama is reachable and the model is available.
        Pull the model automatically if missing.
        Raises OllamaError if Ollama is not reachable after this check.

        Called once at daemon startup (cli.py: ground-station start).
        """
        if not self.is_reachable():
            raise OllamaError(
                f"Ollama is not running at {self.host}. "
                "Start it with: ollama serve"
            )
        log.info(f"[OLLAMA] Server reachable at {self.host}")

        if not self.model_available():
            log.info(f"[OLLAMA] Model '{self.model}' not found locally — pulling.")
            self.pull_model()
        else:
            log.info(f"[OLLAMA] Model '{self.model}' is available.")

    def chat(
        self,
        system_prompt: str,
        user_prompt: str,
        temperature: float = 0.2,
    ) -> str:
        """
        Send a two-message conversation (system + user) and return the
        assistant's response text.

        Temperature 0.2 — low but not zero — produces consistent structured
        output while tolerating minor phrasing variation in the report.
        Zero temperature can produce repetitive loops on some models.

        Retries up to MAX_RETRIES times on transient HTTP/network errors.
        Does not retry on HTTP 4xx (model not found, bad request).

        Raises OllamaError on all permanent failures.

        Reference: https://github.com/ollama/ollama/blob/main/docs/openai.md
        """
        url = f"{self.host}/v1/chat/completions"
        payload = {
            "model": self.model,
            "temperature": temperature,
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user",   "content": user_prompt},
            ],
        }

        last_exc: Optional[Exception] = None

        for attempt in range(1, MAX_RETRIES + 1):
            try:
                with httpx.Client(timeout=self._timeout) as c:
                    resp = c.post(url, json=payload)

                    # 4xx errors are not retried — they indicate a
                    # configuration problem (wrong model name, bad request).
                    if 400 <= resp.status_code < 500:
                        raise OllamaError(
                            f"Ollama returned HTTP {resp.status_code}: {resp.text[:200]}"
                        )

                    resp.raise_for_status()
                    data = resp.json()

                    # Extract response text from OpenAI-compatible response shape:
                    # {"choices": [{"message": {"content": "..."}}]}
                    try:
                        content = data["choices"][0]["message"]["content"]
                    except (KeyError, IndexError) as exc:
                        raise OllamaError(
                            f"Unexpected Ollama response shape: {exc}. "
                            f"Raw: {str(data)[:300]}"
                        ) from exc

                    if not content or not content.strip():
                        raise OllamaError("Ollama returned an empty response.")

                    log.info(
                        f"[OLLAMA] chat() OK (attempt={attempt}, "
                        f"chars={len(content)}, model={self.model})"
                    )
                    return content.strip()

            except OllamaError:
                raise  # never retry OllamaError — it is already a terminal failure
            except (httpx.TimeoutException, httpx.NetworkError) as exc:
                last_exc = exc
                wait = RETRY_BASE_S ** attempt
                log.warning(
                    f"[OLLAMA] Transient error (attempt={attempt}/{MAX_RETRIES}): "
                    f"{type(exc).__name__}: {exc}. Retrying in {wait:.1f}s..."
                )
                time.sleep(wait)
            except Exception as exc:
                last_exc = exc
                wait = RETRY_BASE_S ** attempt
                log.warning(
                    f"[OLLAMA] Unexpected error (attempt={attempt}/{MAX_RETRIES}): "
                    f"{exc}. Retrying in {wait:.1f}s..."
                )
                time.sleep(wait)

        raise OllamaError(
            f"Ollama chat() failed after {MAX_RETRIES} attempts. "
            f"Last error: {last_exc}"
        )
