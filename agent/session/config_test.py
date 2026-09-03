"""Tests for thinking level resolution."""

import unittest

from app.config import resolve_thinking_level
from session import config as session_config


class ThinkingLevelResolutionTest(unittest.TestCase):

  def test_model_default_level_is_used(self):
    for model_name, spec in session_config.KNOWN_MODELS.items():
      with self.subTest(model=model_name):
        self.assertEqual(
            spec.thinking_level,
            resolve_thinking_level(model_name),
        )

  def test_session_override_takes_precedence(self):
    model = next(iter(session_config.KNOWN_MODELS))
    override = "high"
    self.assertEqual(
        override,
        resolve_thinking_level(model, session_override=override),
    )

  def test_unknown_model_gets_global_default(self):
    self.assertEqual(
        session_config.DEFAULT_THINKING_LEVEL,
        resolve_thinking_level("some-unknown-model"),
    )

  def test_low_level_is_default(self):
    """Ensure 'low' is the default thinking level."""
    model = next(iter(session_config.KNOWN_MODELS))
    self.assertEqual(
        "low",
        resolve_thinking_level(model),
    )

  def test_models_prefix_stripped(self):
    """Ensure models/ prefix is stripped for lookup."""
    model = next(iter(session_config.KNOWN_MODELS))
    self.assertEqual(
        session_config.KNOWN_MODELS[model].thinking_level,
        resolve_thinking_level(f"models/{model}"),
    )

  def test_all_levels_are_valid(self):
    """All THINKING_LEVELS should be accepted."""
    for level in session_config.THINKING_LEVELS:
      with self.subTest(level=level):
        result = resolve_thinking_level("any-model", session_override=level)
        self.assertEqual(level, result)


if __name__ == "__main__":
  unittest.main()
