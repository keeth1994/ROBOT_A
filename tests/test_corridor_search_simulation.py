"""Run full-physics search regressions: python -m tests.test_corridor_search_simulation."""
from robot.paths import ROOT
from robot.simulation.corridor_search import CASES, evaluate


def main():
    output = ROOT / 'results/corridor_search_validation'
    output.mkdir(parents=True, exist_ok=True)
    for case in CASES:
        result = evaluate(case, seconds=180, output=output)
        assert result['turn_degrees'] >= 360, case + ': did not complete the scan'
        assert result['succeeded'], case + ': did not confirm entry'
        assert result['inside_and_centered'], case + ': reported success outside the entrance'
        assert not result['collision'], case + ': contacted a wall'
        assert result['warnings'] == 0, case + ': physics warnings'
    print('PASS: all sensor-driven corridor searches entered without wall contact.')


if __name__ == '__main__':
    main()
