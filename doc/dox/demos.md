# Demos

\defgroup demos Demos
\ingroup demos
@{

- [Rigid body planning](RigidBodyPlanning_8cpp_source.html) [[Python version]](RigidBodyPlanning_8py_source.html). This demo is essentially the same example described in the first tutorial. It illustrates how to use the main classes.
- [State sampling](StateSampling_8cpp_source.html) [[Python version]](StateSampling_8py_source.html). This is the demo program used in the last beginner tutorial.
- [Analyze and visualize planner data in Python.](PlannerData_8py_source.html) This demo relies on the [graph-tool](http://projects.skewed.de/graph-tool) package.
- [Rigid body planning with controls](RigidBodyPlanningWithControls_8cpp_source.html) [[Python version]](RigidBodyPlanningWithControls_8py_source.html). This demo shows how to perform planning under differential constraints for a simple car-like vehicle.
- [Rigid body planning with integration and controls.](RigidBodyPlanningWithIntegrationAndControls_8cpp_source.html) This example extends the previous example by showing how one can plan for systems of ordinary differential equations in a generic way. This example uses simple Euler integration. For higher accuracy it is recommended to use the ODESolver class described in the next demo.
- [Rigid body planning with ODESolver and controls.](RigidBodyPlanningWithODESolverAndControls_8cpp_source.html)  [[Python version]](RigidBodyPlanningWithODESolverAndControls_8py_source.html) This example compares and contrasts the previous demo of planning with integration and planning using the ODESolver class, which wraps around [Boost.Numeric.Odeint](http://odeint.org).  Code showing the same model being planned with a user-implemented numerical integration technique as well as the ODESolver is presented.
- [Planning for a simple hybrid system.](HybridSystemPlanning_8cpp_source.html) This demo shows how one could plan for a car with gears. The gear is a discrete state variable, while its pose is continuous. The car needs to make a sharp turn and is forced to change gears. This is not the best way to plan for hybrid systems, since this approach ignores completely the structure that exist in the system. Nevertheless, it demonstrates that the planners in OMPL are state space agnostic and can plan in discrete or hybrid state spaces.
- [Rigid body planning with an Inverse Kinematics solver generating goal states in a separate thread.](RigidBodyPlanningWithIK_8cpp_source.html) This demo shows off two neat features of OMPL: a genetic algorithm-based Inverse Kinematics solver and a lazy goal state sampler. In a separate thread goal states are computed by the IK solver. While solving a motion planning problem, the planning algorithms select a random goal state from the ones computed so far.
- [Rigid body planning using the Open Dynamics Engine (OpenDE).](OpenDERigidBodyPlanning_8cpp_source.html) When OpenDE is installed, OMPL will compile an extension that makes is easier to use OpenDE for forward propagation of models of motion. In this example, a box is pushed around in the plane from a start position to a goal position.
- [Planning for Dubins and Reeds-Shepp cars.](GeometricCarPlanning_8cpp_source.html) This demo illustrates the use of the ompl::base::DubinsStateSpace and ompl::base::ReedsSheppStateSpace. The demo can solve two simple planning problems, print trajectories from the origin to a user-specified state, or print a discretized distance field.

@}
