^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package multires_image
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.4.6 (2025-04-02)
------------------

2.4.5 (2025-01-20)
------------------

2.4.4 (2024-12-09)
------------------

2.4.3 (2024-09-18)
------------------

2.4.2 (2024-08-12)
------------------

2.4.1 (2024-08-09)
------------------

2.4.0 (2024-08-09)
------------------

2.3.0 (2023-08-24)
------------------

2.2.2 (2023-06-07)
------------------
* Iron Compatibility (`#779 <https://github.com/swri-robotics/mapviz/issues/779>`_)
* Contributors: David Anthony

2.2.1 (2023-05-30)
------------------
* Updating maintainers list (`#778 <https://github.com/swri-robotics/mapviz/issues/778>`_)
* Merge pull request `#754 <https://github.com/swri-robotics/mapviz/issues/754>`_ from cottsay/python3-shebang
* Use python3 in mapviz_tile_loader shebang
* Contributors: David Anthony, Scott K Logan

2.1.0 (2020-10-22)
------------------
* ROS Foxy support (`#695 <https://github.com/swri-robotics/mapviz/issues/695>`_)
* Contributors: P. J. Reed

2.0.0 (2020-05-13)
------------------
* Port mapviz to ROS 2 (`#672 <https://github.com/swri-robotics/mapviz/issues/672>`_)
* Remove OpenGL warning (`#667 <https://github.com/swri-robotics/mapviz/issues/667>`_)
* Contributors: Daniel D'Souza, P. J. Reed, Jacob Hassold, Kevin Nickels, Roger Strain

1.2.0 (2019-09-04)
------------------
* Use local_xy_origin for loading tiles of GPSFix not available (`#634 <https://github.com/swri-robotics/mapviz/issues/634>`_)
* Contributors: agyoungs

1.1.1 (2019-05-17)
------------------

1.1.0 (2019-02-20)
------------------

1.0.1 (2019-01-25)
------------------

1.0.0 (2019-01-23)
------------------
* Sharing tf_manager\_ between main app and plugins (`#555 <https://github.com/swri-robotics/mapviz/issues/555>`_)
* Contributors: Davide Faconti

0.3.0 (2018-11-16)
------------------
* Merge all -devel branches into a single master branch
* Contributors: P. J. Reed

0.2.6 (2018-07-31)
------------------

0.2.5 (2018-04-12)
------------------
* Add ability to set offset for multires image (`#565 <https://github.com/swri-robotics/mapviz/issues/565>`_)
* Fix multires image scale when projection is WGS84.
* update to use non deprecated pluginlib macro
* Mapviz tile loader (Kinetic) (`#509 <https://github.com/swri-robotics/mapviz/issues/509>`_)
* Change package.xml dep order
* Support transparent tiles in multires_image
* Contributors: Marc Alban, Mikael Arguedas, P. J. Reed, jgassaway

0.2.4 (2017-08-11)
------------------

0.2.3 (2016-12-10)
------------------

0.2.2 (2016-12-07)
------------------
* Migrated OpenCV to 3.1 (default in Kinetic)
* Contributors: Brian Holt

0.2.1 (2016-10-23)
------------------

0.2.0 (2016-06-23)
------------------
* Update Qt to version 5
* Contributors: Ed Venator

0.1.3 (2016-05-20)
------------------
* Implement service for adding and modifying mapviz displays.
* Fix for `#339 <https://github.com/swri-robotics/mapviz/issues/339>`_; explicitly depending on OpenCV 2
* Contributors: Marc Alban, P. J. Reed

0.1.2 (2016-01-06)
------------------
* Enables the possibility to load a one-layer tile set
* Contributors: Vincent Rousseau

0.1.1 (2015-11-17)
------------------
* Use extension from geo file
* Contributors: Vincent Rousseau

0.1.0 (2015-09-29)
------------------

0.0.3 (2015-09-28)
------------------

0.0.2 (2015-09-27)
------------------
* Adds missing qt4_opengl dependency

0.0.1 (2015-09-27)
------------------
* Renames all marti_common packages that were renamed.
  (See http://github.com/swri-robotics/marti_common/issues/231)
* Fixes catkin_lint problems that could prevent installation.
* Cleans up dependencies
* Adds find_package(OpenCV REQUIRED) to cmake config
* fixes lint issues
* updates cmake version to squash the CMP0003 warning
* removes dependencies on build_tools
* uses format 2 package definition
* fix missing organization in license text
* exports the multires_image library
* catkinizes mapviz
* changes license to BSD
* adds license and readme files
* Contributors: Ed Venator, Edward Venator, Jerry Towler, Marc Alban, P. J. Reed
