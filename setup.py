import setuptools

setuptools.setup(
    name="sensor_degradation_filter_py",
    version="2.0.0",
    description= "Python library for the sensor degradation filter",
    author="Khai Yi Chin",
    author_email="khaiyichin@gmail.com",
    url="https://github.com/khaiyichin/sensor-degradation-filter",
    packages=["notebooks", "scripts.python"],
    scripts=[
        "scripts/python/run_static_degradation_experiment.py",
        "scripts/python/extract_convergence_accuracy_data.py",
        "scripts/python/extract_decision_data.py",
        "scripts/python/extract_dynamic_degradation_data.py",
        "scripts/python/compute_rmsd_df.py",
        "scripts/python/extract_isolated_rmsd_data.py"
    ]
)