# Download

# Releases

- __OMPL__ 0.12.2, released Apr 29, 2013 ([release notes](releaseNotes.html))

  <a href="https://bitbucket.org/ompl/ompl/downloads/ompl-0.12.2-Source.tar.gz" class="btn btn-primary btn-small">TGZ</a>
  <a href="https://bitbucket.org/ompl/ompl/downloads/ompl-0.12.2-Source.zip" class="btn btn-primary btn-small">ZIP</a><br><br>
- [Installation instructions.](installation.html)
- [Older releases.](https://bitbucket.org/ompl/ompl/downloads) See the [release notes](releaseNotes.html) for a brief a description of changes for each release.


# Source Code

- The [latest source](https://bitbucket.org/ompl/ompl/src) is available via Mercurial. For anonymous checkout use:

      $ hg clone https://bitbucket.org/ompl/ompl

- Alternatively, get a snapshot for the latest source code:

  <a href="https://bitbucket.org/ompl/ompl/get/tip.tar.bz2" class="btn btn-primary btn-small">TBZ</a>
  <a href="https://bitbucket.org/ompl/ompl/get/tip.tar.gz" class="btn btn-primary btn-small">TGZ</a>
  <a href="https://bitbucket.org/ompl/ompl/get/tip.zip" class="btn btn-primary btn-small">ZIP</a><br><br>
- [Installation instructions](installation.html) are the same as for a release.


# Debian Packages

- Debian Packages are are generated periodically for __OMPL__ as part of the ROS release process. On Ubuntu, you can use this:

      sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
      wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
      sudo apt-get update
      sudo apt-get install ros-`rosversion -d`-ompl

- The .deb files are also available on [http://packages.ros.org](http://packages.ros.org/ros/ubuntu/pool/main/r/).
- Notes:
  - The binary files are installed in /opt/ros/`rosversion -d`/, so yo will need to include that path when searching for OMPL headers and libs.
  - The version for the .deb files is of the form _X.Y.Z00R_, where _X.Y.Z_ is the most recent OMPL release version and _R_ is the revision number in the Mercurial repository for that .deb file.


# OMPL in ROS

- __OMPL__ is used as a system dependency in [ROS](http://www.ros.org), and the correct .deb file is installed automatically as needed.
- A ROS package providing the latest version of this library is also available at:

      $ svn co https://rice-ros-pkg.svn.sourceforge.net/svnroot/rice-ros-pkg/ompl ompl
