# The Open Motion Planning Library

\htmlonly
<div id="fb-root"></div>
<script>(function(d, s, id) {
  var js, fjs = d.getElementsByTagName(s)[0];
  if (d.getElementById(id)) return;
  js = d.createElement(s); js.id = id;
  js.src = "//connect.facebook.net/en_US/all.js#xfbml=1";
  fjs.parentNode.insertBefore(js, fjs);
}(document, 'script', 'facebook-jssdk'));</script>

<div class="row"><div class="span6">\endhtmlonly

The Open Motion Planning Library (__OMPL__) consists of a set of sampling-based motion planning algorithms. The content of the library is limited to these algorithms, which means there is no environment specification, no collision detection or visualization. The library is designed so it can be easily integrated into systems that provide the additional needed components. For examples of complete systems using OMPL, see [OMPL.app](http://ompl.kavrakilab.org) and [ROS](http://www.ros.org/wiki/ompl). We have also developed a [educational module on motion planning](education.html) that is centered around OMPL.app. We are looking for educational partners to use and further develop the material. Please contact us for more information.

OMPL is intended to be efficient, thread safe, easy to use, easily extensible and freely available (visit this project on [Bitbucket](https://bitbucket.org/ompl/ompl)).

\htmlonly
<a href="download.html" class="btn btn-primary">Current version: 0.12.2
<br>
<small>Released: Apr 29, 2013</small></a>

<a href="citations.html" class="btn btn-primary btn-primary">Click for citation,<br><small>if you use OMPL in your work</small></a>
</p>

<div class="fb-like" data-href="http://www.facebook.com/pages/OMPL/320018418039567" data-send="true" data-layout="button_count" data-width="450" data-show-faces="false"></div>

</div><div class="span6">
  <div class="carousel slide" id="omplCarousel">
    <div class="carousel-inner">
      <div class="item active">
      <div class="pagination-centered"><img src="../images/mp.jpg"></div>
      </div>
      <!--<div class="item">
        <img src="../images/mp.jpg" class="hidden" style="margin-top: 1px">
        <div class="carousel-caption carousel-caption-inset"><h4>New in 0.11!</h4>
        <ul>
           <li>PlannerData now uses the Boost Graph Library; each planner can store arbitrary metadata in a graph.  All this PlannerData can easily be (de)serialized for messaging or storing/loading of planner data.
           <li>Implementation of PRM is now threaded (one thread for growing the roadmap, one thread for monitoring whether the problem is solved).
        </ul>
        <p>See <a href="releaseNotes.html">release notes</a> for details</p>
      </div>-->
      <div class="item">
        <div class="pagination-centered"><img src="../images/T-RRT.jpg" style="padding-bottom: 2px"></div>
        <div class="carousel-caption"><h4>Planning with costs</h4>
        <p>Visualization of the cost map explored by the T-RRT planner.</p>
        </div>
      </div>
      <div class="item">
        <div class="pagination-centered"><a href="http://ompl.kavrakilab.org/gui.html"><img src="../images/gui_path-small.jpg"></a></div>
        <div class="carousel-caption"><h4>OMPL.app GUI</h4></div>
      </div>
      <div class="item">
        <div class="pagination-centered"><a href="http://www.ros.org/wiki/ompl"><img src="../images/pr2.jpg" class="pagination-centered"></a></div>
        <div class="carousel-caption"><h4>OMPL Inside</h4><p>OMPL is used inside ROS to plan motions for the PR2 and many other robots.</p></div>
      </div>
    </div>
    <a class="carousel-control left" href="#omplCarousel" data-slide="prev">&lsaquo;</a>
    <a class="carousel-control right" href="#omplCarousel" data-slide="next">&rsaquo;</a>
  </div>
</div></div><div class="row"><div class="span4">\endhtmlonly


## Contents of This Library

- OMPL contains implementations of many sampling-based algorithms such as PRM, RRT, EST, SBL, KPIECE, SyCLOP, and several variants of these planners. See [available planners](planners.html) for a complete list.
- All these planners operate on very abstractly defined state spaces. Many commonly used [state spaces](spaces.html) are already implemented (e.g., SE2, SE3, R<sup>n</sup>, etc.).
- For any state space, different [state samplers](samplers.html) can be used (e.g., uniform, Gaussian, obstacle based, etc.).
- [API overview](api_overview.html)

\htmlonly</div><div class="span4">\endhtmlonly


## Getting Started

- The [OMPL primer](http://ompl.kavrakilab.org/OMPL_Primer.pdf) provides a brief background on sampling-based motion planning, and an overview of OMPL.
- [Download](download.html) and [install](installation.html) OMPL.
- [Demos](demos.html) and [tutorials](tutorials.html).
- [Frequently Asked Questions](FAQ)
- Familiarize yourself with the [Boost structures](boost.html) used throughout OMPL.
- Learn how to integrate your own code with [OMPL's build system](buildSystem.html).
- If interested in using Python, make sure to read [the documentation for the Python bindings](python.html).

\htmlonly</div><div class="span4">\endhtmlonly


## Other Resources

- [OMPL for education](education.html)
- [Gallery of example uses of OMPL](gallery.html).
- If you are interested in the [ROS interface to OMPL](http://www.ros.org/wiki/ompl_ros_interface), please read the [tutorial on using OMPL within ROS](http://kavrakilab.org/OMPLtutorial).
- [Third-party contributions.](thirdparty.html) ([Contribute your own extensions!](contrib.html))

\htmlonly</div></div>
<div class="row"><div class="span12">\endhtmlonly


## News & Events

- [OMPL has won the 2012 Open Source Software World Grand Challenge!](http://ompl.kavrakilab.org/blog/?p=178)
- [An article about OMPL](../ieee-ram-2012-ompl.pdf) has been accepted for publication in IEEE's Robotics & Automation Magazine! It will appear in the December 2012 issue.
- <a href="http://www.youtube.com/watch?v=r1zbuLc8RhI">At ROSCON, Sachin Chitta and Ioan Șucan gave a talk about MoveIt!</a>, the new motion planning stack in ROS. It provides a common interface to motion planning libraries in ROS (including OMPL). It will eventually replace the arm navigation stack.
- <a href="http://kavrakilab.org/OMPLtutorial">IROS 2011 Tutorial on Motion Planning for Real Robots</a>. This hands-on tutorial describes how to use the ROS and OMPL, but it also gives some background on sampling-based motion planning.

\htmlonly</div></div></div>\endhtmlonly
