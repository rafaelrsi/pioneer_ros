/*
* ompl_ltldublin.cpp - 
*
* Plans a trajectory (state and input) of a Dublin's vehicle which 
* satisfies a LTLcosafe and a LTL safety formulas.
* 
* Requires: OMLP (http://ompl.kavrakilab.org) and 
*           SPOT (https://spot.lrde.epita.fr/)
*
* Usage : from MATLAB
*      >> traj = ompl_ltldublin(O, P, phi1, phi2, x0, x_lb, x_ub, u_lb, u_ub)
*
*      * O is a cell vector with vertices of obstacles, i.e. O(i) is a 
*        numeric matrix with a row for each 2D vertices.
*
*      * P is a cell vector with vertices of propositions, i.e. P(i) is a 
*        numeric matrix with a row for each 2D vertices.
*
*      * phi1, phi2 are cosafety LTL and safety LTL formulas, respectively,
*        i.e. these arguments are strings. It uses SPOT syntax.
*
*      * x0 is the initial state, i.e., a numeric vector.
*
*      * x_lb,x_ub are numeric vectors with lower and upper bounds of the
*        state space, i.e. a vector with 2 rows (px,py).
*
*      * u_lb,u_ub are numeric vectors with lower and upper bounds of the
*        input space, i.e. a vector with 2 rows (v,w).
*
* This is a C++ MEX-file for MATLAB.
* This code is based on the following demo:
* http://ompl.kavrakilab.org/LTLWithTriangulation_8cpp_source.html
*
*/

 #include "mex.h"
 #include "matrix.h"


 #include <ompl/control/SpaceInformation.h>
 #include <ompl/base/spaces/SE2StateSpace.h>
 #include <ompl/control/spaces/RealVectorControlSpace.h>
 #include <ompl/control/SimpleSetup.h>
 #include <ompl/config.h>
 #include <iostream>
 #include <vector>
 
 #include <ompl/extensions/triangle/PropositionalTriangularDecomposition.h>
 #include <ompl/control/planners/ltl/PropositionalDecomposition.h>
 #include <ompl/control/planners/ltl/Automaton.h>
 #include <ompl/control/planners/ltl/ProductGraph.h>
 #include <ompl/control/planners/ltl/LTLPlanner.h>
 #include <ompl/control/planners/ltl/LTLProblemDefinition.h>

 namespace ob = ompl::base;
 namespace oc = ompl::control;

 using Polygon = oc::PropositionalTriangularDecomposition::Polygon;
 using Vertex = oc::PropositionalTriangularDecomposition::Vertex;

#define IS_2D_CELL(P) (mxIsCell(P) && mxGetNumberOfDimensions(P) == 2)
#define IS_REAL_2D_FULL_DOUBLE(P) (!mxIsComplex(P) && mxGetNumberOfDimensions(P) == 2 && !mxIsSparse(P) && mxIsDouble(P))
#define IS_REAL_SCALAR(P) (IS REAL 2D FULL DOUBLE(P) && mxGetNumberOfElements(P) == 1)


 // a decomposition is only needed for SyclopRRT and SyclopEST
 // use TriangularDecomp
 class MyDecomposition : public oc::PropositionalTriangularDecomposition {
 public:
     MyDecomposition(const ob::RealVectorBounds& bounds)
         : oc::PropositionalTriangularDecomposition(bounds) { }
     ~MyDecomposition() override = default;

     void project(const ob::State* s, std::vector<double>& coord) const override
     {
         coord.resize(2);
         coord[0] = s->as<ob::SE2StateSpace::StateType>()->getX();
         coord[1] = s->as<ob::SE2StateSpace::StateType>()->getY();
     }

     void sampleFullState(const ob::StateSamplerPtr& sampler, const std::vector<double>& coord, ob::State* s) const override
     {
        sampler->sampleUniform(s);
        auto* ws = s->as<ob::SE2StateSpace::StateType>();
        ws->setXY(coord[0], coord[1]);
     }
};

 /* Returns whether a point (x,y) is within a given polygon.
    We are assuming that the polygon is a axis-aligned rectangle, with vertices stored
    in counter-clockwise order, beginning with the bottom-left vertex. */
 bool polyContains(const Polygon& poly, double x, double y)
 {
     return x >= poly.pts[0].x && x <= poly.pts[2].x
         && y >= poly.pts[0].y && y <= poly.pts[2].y;
 }

 /* Our state validity checker queries the decomposition for its obstacles,
    and checks for collisions against them.
    This is to prevent us from having to redefine the obstacles in multiple places. */
 bool isStateValid(
     const oc::SpaceInformation *si,
     const std::shared_ptr<oc::PropositionalTriangularDecomposition> &decomp,
     const ob::State *state)
 {
     if (!si->satisfiesBounds(state))
         return false;
     const auto* se2 = state->as<ob::SE2StateSpace::StateType>();

     double x = se2->getX();
     double y = se2->getY();
     const std::vector<Polygon>& obstacles = decomp->getHoles();
     for (const auto & obstacle : obstacles)
     {
         if (polyContains(obstacle, x, y))
             return false;
     }
     return true;
 }

 void propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result)
 {
     const auto* se2 = start->as<ob::SE2StateSpace::StateType>();
     const auto* rctrl = control->as<oc::RealVectorControlSpace::ControlType>();

     double xout = se2->getX() + rctrl->values[0]*duration*cos(se2->getYaw());
     double yout = se2->getY() + rctrl->values[0]*duration*sin(se2->getYaw());
     double yawout = se2->getYaw() + duration*rctrl->values[1];

     auto* se2out = result->as<ob::SE2StateSpace::StateType>();
     se2out->setXY(xout, yout);
     se2out->setYaw(yawout);

     auto* so2out = se2out->as<ob::SO2StateSpace::StateType>(1);
     ob::SO2StateSpace SO2;
     SO2.enforceBounds (so2out);
 }

void mexFunction(int nlhs, mxArray *plhs[], /* Output variables */
				 int nrhs, const mxArray *prhs[]) /* Input variables */
{
	char usage[] = "\n\nUsage: traj = ompl_ltldublin(O, P, phi1, phi2, x0, x_lb, x_ub, u_lb, u_ub)\n";
    mexPrintf("Starting the control design from LTL for a Dublin's vehicle ...\n");
	if ((nlhs!=1)||(nrhs!=9)) {
		mexPrintf(usage);
		mexErrMsgTxt("Invalid number of arguments.");
	}
#define O			prhs[0]
#define P	 		prhs[1]
#define phi1	 	prhs[2]
#define phi2	 	prhs[3]
#define x0  	 	prhs[4]
#define x_lb	 	prhs[5]
#define x_ub	 	prhs[6]
#define u_lb	 	prhs[7]
#define u_ub	 	prhs[8]
#define traj        plhs[0]

	int M, N, m, n;
	double *A;
	mxArray *V;
	char *phi_buf;

    // construct the state space we are planning in
    auto space(std::make_shared<ob::SE2StateSpace>());

    // set the bounds for the R^2 part of SE(2)
    ob::RealVectorBounds bounds(2);

    if ((!IS_REAL_2D_FULL_DOUBLE(x_lb))||(mxGetM(x_lb)!=2)||(mxGetN(x_lb)!=1)) {
		mexPrintf(usage);
		mexErrMsgTxt("x_lb must be a real full double vector with 2 elements.");
    }
    A = mxGetPr(x_lb);
    bounds.setLow(0,A[0]);
    bounds.setLow(1,A[1]);


    if ((!IS_REAL_2D_FULL_DOUBLE(x_ub))||(mxGetM(x_ub)!=2)||(mxGetN(x_ub)!=1)) {
		mexPrintf(usage);
		mexErrMsgTxt("x_ub must be a real full double vector with 2 elements.");
    }
    A = mxGetPr(x_ub);
    bounds.setHigh(0,A[0]);
    bounds.setHigh(1,A[1]);

    space->setBounds(bounds);
    // create triangulation that ignores obstacle and respects propositions
    std::shared_ptr<oc::PropositionalTriangularDecomposition> ptd = std::make_shared<MyDecomposition>(bounds);

    if (!IS_2D_CELL(O)) {
		mexPrintf(usage);
		mexErrMsgTxt("O must be a column cell vector.");
    }
    N = mxGetNumberOfElements(O);
    for(n=0;n<N;n++) {
    	V = mxGetCell(O,n);
        if ((!IS_REAL_2D_FULL_DOUBLE(V))||(mxGetM(V)<3)||(mxGetN(V)!=2)) {
    		mexPrintf(usage);
    		mexErrMsgTxt("Each obstacle in O must be a V-representation of a bounded and full-dimensional 2D polytope.");
        }
        M = mxGetM(V);
    	A = mxGetPr(V);
        Polygon obstacle(M);
    	for(m=0;m<M;m++) {
            obstacle.pts[m] = Vertex(A[m],A[m+M]);
    	}
        ptd->addHole(obstacle);
    }

    if (!IS_2D_CELL(P)) {
		mexPrintf(usage);
		mexErrMsgTxt("P must be a column cell vector.");
    }
    N = mxGetNumberOfElements(P);
    for(n=0;n<N;n++) {
    	V = mxGetCell(P,n);
        if ((!IS_REAL_2D_FULL_DOUBLE(V))||(mxGetM(V)<3)||(mxGetN(V)!=2)) {
    		mexPrintf(usage);
    		mexErrMsgTxt("Each proposition in P must be a V-representation of a bounded and full-dimensional 2D polytope.");
        }
        M = mxGetM(V);
    	A = mxGetPr(V);
        Polygon p(M);
    	for(m=0;m<M;m++) {
            p.pts[m] = Vertex(A[m],A[m+M]);
    	}
        ptd->addProposition(p);
    }

    ptd->setup();

    // create a control space
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(2);

    if ((!IS_REAL_2D_FULL_DOUBLE(u_lb))||(mxGetM(u_lb)!=2)||(mxGetN(u_lb)!=1)) {
		mexPrintf(usage);
		mexErrMsgTxt("u_lb must be a real full double vector with 2 elements.");
    }
    A = mxGetPr(u_lb);
    cbounds.setLow(0,A[0]);
    cbounds.setLow(1,A[1]);


    if ((!IS_REAL_2D_FULL_DOUBLE(u_ub))||(mxGetM(u_ub)!=2)||(mxGetN(u_ub)!=1)) {
		mexPrintf(usage);
		mexErrMsgTxt("u_ub must be a real full double vector with 2 elements.");
    }
    A = mxGetPr(u_ub);
    cbounds.setHigh(0,A[0]);
    cbounds.setHigh(1,A[1]);

    cspace->setBounds(cbounds);

    oc::SpaceInformationPtr si(std::make_shared<oc::SpaceInformation>(space, cspace));
    si->setStateValidityChecker(
        [&si, ptd](const ob::State *state)
        {
            return isStateValid(si.get(), ptd, state);
        });
    si->setStatePropagator(propagate);
    si->setPropagationStepSize(0.025);    

    //LTL co-safety sequencing formula: visit p2,p0 in that order
#if OMPL_HAVE_SPOT
    if ( !mxIsChar(phi1)) {
    	mexPrintf(usage);
    	mexErrMsgTxt("phi1 must be a string.");
    }

    /* input must be a row vector */
    if (mxGetM(phi1)!=1) {
    	mexPrintf(usage);
    	mexErrMsgTxt("phi1 must be a row vector.");
    }
    phi_buf = mxArrayToString(phi1);
    // This shows off the capability to construct an automaton from LTL-cosafe formula using Spot
    auto cosafety = std::make_shared<oc::Automaton>(3, phi_buf);
    mxFree(phi_buf);
#else
	mexPrintf(usage);
	mexErrMsgTxt("SPOT not found.");
#endif
    //LTL safety avoidance formula: never visit p1
#if OMPL_HAVE_SPOT
    if ( !mxIsChar(phi2)) {
    	mexPrintf(usage);
    	mexErrMsgTxt("phi2 must be a string.");
    }

    /* input must be a row vector */
    if (mxGetM(phi2)!=1) {
    	mexPrintf(usage);
    	mexErrMsgTxt("phi2 must be a row vector.");
    }
    phi_buf = mxArrayToString(phi2);
    // This shows off the capability to construct an automaton from LTL-safe formula using Spot
    auto safety = std::make_shared<oc::Automaton>(3, phi_buf, false);
    mxFree(phi_buf);
#else
	mexPrintf(usage);
	mexErrMsgTxt("SPOT not found.");
#endif

    // construct product graph (propDecomp x A_{cosafety} x A_{safety})
    auto product(std::make_shared<oc::ProductGraph>(ptd, cosafety, safety));

    // LTLSpaceInformation creates a hybrid space of robot state space x product graph.
    // It takes the validity checker from SpaceInformation and expands it to one that also
    // rejects any hybrid state containing rejecting automaton states.
    // It takes the state propagator from SpaceInformation and expands it to one that
    // follows continuous propagation with setting the next decomposition region
    // and automaton states accordingly.
    //
    // The robot state space, given by SpaceInformation, is referred to as the "lower space".
    auto ltlsi(std::make_shared<oc::LTLSpaceInformation>(si, product));

    // LTLProblemDefinition creates a goal in hybrid space, corresponding to any
    // state in which both automata are accepting
    auto pdef(std::make_shared<oc::LTLProblemDefinition>(ltlsi));

    // create a start state
    ob::ScopedState<ob::SE2StateSpace> start(space);
    if ((!IS_REAL_2D_FULL_DOUBLE(x0))||(mxGetM(x0)!=3)||(mxGetN(x0)!=1)) {
		mexPrintf(usage);
		mexErrMsgTxt("x0 must be a real full double vector with 3 elements.");
    }
    A = mxGetPr(x0);
    start->setX(A[0]);
    start->setY(A[1]);
    start->setYaw(A[2]);

    // addLowerStartState accepts a state in lower space, expands it to its
    // corresponding hybrid state (decomposition region containing the state, and
    // starting states in both automata), and adds that as an official start state.
    pdef->addLowerStartState(start.get());

    //LTL planner (input: LTL space information, product automaton)
    oc::LTLPlanner ltlPlanner(ltlsi, product);
    ltlPlanner.setProblemDefinition(pdef);

    // attempt to solve the problem within thirty seconds of planning time
    // considering the above cosafety/safety automata, a solution path is any
    // path that visits p2 followed by p0 while avoiding obstacles and avoiding p1.
    ob::PlannerStatus solved = ltlPlanner.ob::Planner::solve(30.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // The path returned by LTLProblemDefinition is through hybrid space.
        // getLowerSolutionPath() projects it down into the original robot state space
        // that we handed to LTLSpaceInformation.

        M = static_cast<oc::PathControl &>(*pdef->getLowerSolutionPath()).getStateCount ();
        N = 6;
        traj = mxCreateDoubleMatrix(M,N,mxREAL);
        
        A = mxGetPr(traj);

        std::stringstream ss;
        static_cast<oc::PathControl &>(*pdef->getLowerSolutionPath()).printAsMatrix(ss);
        std::string line, elem;
        m = 0;
        while (std::getline(ss, line)) {
      	    std::stringstream ssline(line);
      	    n=0;
            while (std::getline(ssline, elem, ' ')) {
          	  if ((0<=n)&&(n<N)) {
          		  A[m+n*M] = std::stod(elem);
          	  }
          	  n++;
            }
            m++;
        }
    }
    else {
    	std::cout << "No solution found" << std::endl;
                traj = mxCreateDoubleMatrix(M,N,mxREAL);
        traj = mxCreateDoubleMatrix(0,0,mxREAL);

    }
}
