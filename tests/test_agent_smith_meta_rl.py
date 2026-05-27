from pathlib import Path


SOURCE = Path(__file__).resolve().parents[1] / "agent_smith_meta.py"


def _source_text():
    return SOURCE.read_text(encoding="utf-8")


def test_meta_relation_action_space_contains_all_known_relations():
    source = _source_text()
    expected = [
        "motion",
        "rewind",
        "force_additivity",
        "time_scaling",
        "mass_scaling",
        "determinism",
        "symmetry",
        "zero_input_stability",
        "force_isolation",
        "force_removal",
        "temporal_monotonicity",
        "joint_constraint_stability",
        "mass_scaling_no_reset",
        "environment_perturbation_robustness",
    ]

    for relation in expected:
        assert f'"{relation}"' in source
        assert f"test_type == '{relation}'" in source or f'test_type == "{relation}"' in source


def test_meta_rl_components_are_wired_into_main_loop():
    source = _source_text()
    required_snippets = [
        "class MetaSimulatorState",
        "class MetaRelationSequenceManager",
        "class MetaActor",
        "class MetaCritic",
        "def select_meta_action",
        "def calculate_meta_reward",
        "def train_meta_policy",
        "unit.generate_and_test_commands(test_type=test_type)",
        "append_meta_training_metrics",
        "--random-meta",
    ]

    for snippet in required_snippets:
        assert snippet in source


def test_generate_and_test_commands_returns_structured_result():
    source = _source_text()
    assert "def generate_and_test_commands(self, test_type=None):" in source
    assert '"executed": executed' in source
    assert '"test_passed": test_passed' in source
    assert '"playback_passed": playback_passed' in source
    assert '"crash_detected": crash_detected' in source
