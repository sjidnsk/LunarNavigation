from setuptools import setup

setup(name='lunar_nav1cm_verification', version='0.1.0',
      packages=['lunar_nav1cm_verification'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/lunar_nav1cm_verification']),
                  ('share/lunar_nav1cm_verification', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='Navigation verification', maintainer_email='verification@example.invalid',
      description='Isolated Humble asynchronous navigation verification', license='Apache-2.0',
      tests_require=['pytest'],
      entry_points={'console_scripts': [
          'nav1cm_fixture=lunar_nav1cm_verification.runtime:fixture_main',
          'nav1cm_navigator=lunar_nav1cm_verification.runtime:navigator_main',
          'nav1cm_probe=lunar_nav1cm_verification.runtime:probe_main',
          'nav1cm_run=lunar_nav1cm_verification.launcher:main']})
