import os

os.environ.setdefault('MPLBACKEND', 'Agg')

import matplotlib
matplotlib.use('Agg', force=True)


def test_demo_all_examples_run():
    from demo import (
        example_1_basic_adrc,
        example_2_adrc_with_td,
        example_3_adrc_with_delay,
    )

    example_1_basic_adrc()
    example_2_adrc_with_td()
    example_3_adrc_with_delay()


def test_demo_standard_vs_cascaded_runs():
    from demo_standard_vs_cascaded import example_standard_vs_cascaded

    example_standard_vs_cascaded()