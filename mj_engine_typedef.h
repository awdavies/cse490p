/*******************************************
This file is part of the MuJoCo software.
(C) 2012 Emo Todorov. All rights reserved.
*******************************************/


// disable padding of structure fields, so we can load/save/copy structures
#pragma pack(push, 1)


//---------------------------- global constants -----------------------------------------

#define mjPI			3.14159265358979323846

#define mjMINVAL_F		1E-10		// minimum value allowed in any denominator, float
#define mjMINVAL_D		1E-14		// minimum value allowed in any denominator, double
#define mjMINMU			1E-5		// minimum friction coefficient
#define mjNUMDISABLE	16			// number of disable flags, see mjtDisableBit
#define mjGEOMTYPES		8			// number of geom types
#define mjMAXCONPAIR	4			// maximum number of contacts per geom pair
#define mjMAXCONVIS		50			// maximum number of contacts to visualize
#define mjMAXGROUP		5			// maximum number of impulse groups
#define mjMAXWARNING	10			// maximum number of warning types
#define mjOBJVIS		10			// additional objects to visualize
#define mjTXTVIS		100			// text to visualize
#define mjDPRM			2			// number of actuator dynamics parameters
#define mjTPRM			5			// number of actuator transmission parameters
#define mjGPRM			2			// number of actuator gain parameters


//---------------------------- floating-point precision ---------------------------------

#ifdef mjUSEDOUBLE
	typedef double mjtNum;			// numeric data type (float or double)

	static const double mjMINVAL = mjMINVAL_D;
	#define mju_sqrt sqrt
	#define mju_exp exp
	#define mju_sin sin
	#define mju_tan tan
	#define mju_cos cos
	#define mju_acos acos
	#define mju_atan2 atan2

#else
	typedef float mjtNum;

	static const float mjMINVAL = mjMINVAL_F;
	#define mju_sqrt sqrtf
	#define mju_exp expf
	#define mju_sin sinf
	#define mju_cos cosf
	#define mju_tan tanf
	#define mju_acos acosf
	#define mju_atan2 atan2f
#endif


//---------------------------- ptimitive types (mjt) ------------------------------------
	
typedef unsigned char mjtByte;		// used for true/false


typedef enum _mjtWarning			// warning types
{
	mjWARN_INERTIA	= 0,			// singular inertia matrix
	mjWARN_NOCOLFUNC= 1,			// undefined collision function for geom pair
	mjWARN_NOADIAG	= 2,			// diagonal of A is needed but not computed
	mjWARN_CONTACT	= 3,			// too many contacts added
	mjWARN_JACFULL	= 4,			// too many Jacobian elements added
	mjWARN_NOSOLVER					// user impulse solver is missing
} mjtWarning;


typedef enum _mjtDisableBit			// disable bitflags
{
	mjCONSTRAINT	= 1<<0,			// equality constraints
	mjIMPULSE		= 1<<1,			// entire impulse solver
	mjLIMIT			= 1<<2,			// joint limits	
	mjCONTACT		= 1<<3,			// contacts
	mjSPARSEJAC		= 1<<4,			// use sparse impulse Jacobian
	mjPASSIVE		= 1<<5,			// passive forces
	mjGRAVITY		= 1<<6,			// gravitational foces
	mjBIAS			= 1<<7,			// rne bias forces (Coriolis, centrifugal, gravity)
	mjJNTFRICTION	= 1<<8,			// joint friction
	mjCLAMPVEL		= 1<<9,			// clamp qvel_next to maxvel
	mjCLAMPACT		= 1<<10,		// clamp actuator force
	mjIMPLCTDAMP	= 1<<11,		// add damping*dt to inertia
	mjENERGY		= 1<<12,		// compute kinetic and potential energy
	mjFILTERPARENT	= 1<<13,		// remove collisions with parent body
	mjCBCONTROL		= 1<<14,		// use control callback
	mjCBPASSIVE		= 1<<15			// use passive callback
} mjtDisableBit;


typedef enum _mjtJoint				// type of degree of freedom
{
	mjFREE			= 0,			// global position and orientation (quat)		(7)
	mjBALL			= 1,			// orientation (quat) relative to parent		(4)
	mjSLIDE			= 2,			// sliding distance along body-fixed axis		(1)
	mjHINGE			= 3,			// rotation angle (rad) around body-fixed axis	(1)
	mjNOJOINT		= 1001
} mjtJoint;


typedef enum _mjtGeom				// type of geometric shape
{
	mjPLANE			= 0,
	mjSPHERE		= 1,
	mjCAPSULE		= 2,
	mjELLIPSOID		= 3,
	mjCYLINDER		= 4,
	mjBOX			= 5,
	mjCAGE			= 6,			// box that keeps objects inside
	mjMESH			= 7,			// mesh can be: convex, heightfield, general
	mjARROW			= 100,			// visualization only, cannot be used in model
	mjNOGEOM		= 1001
} mjtGeom;


typedef enum _mjtMesh				// type of mesh
{
	mjMESH_GENERAL = 0,
	mjMESH_HEIGHTFIELD,
	mjMESH_CONVEX
} mjtMesh;


typedef enum _mjtIntegrator			// integrator mode
{
	mjINT_EXPLICIT = 0,				// explicit Euler
	mjINT_MIDPOINT,					// midpoint Euler
	mjINT_SEMIIMPLICIT,				// semi-implicit Euler
	mjINT_RK4						// 4th order Runge Kutta
} mjtIntegrator;


typedef enum _mjtCollision			// collision mode for selecting geom pairs
{
	mjCOL_NONE = 0,					// no collisions are generated
	mjCOL_BROADPHASE,				// test geom pairs determined by broadphase
	mjCOL_PAIR,						// test precomputed geom pairs
	mjCOL_ALL						// test all pairs (except filtered)
} mjtCollision;


typedef enum _mjtAlgorithm			// algorithm used for impulse computation
{
	mjALG_DIAGONAL = 0,				// diagonal solver: f = project(b/diag(A))
	mjALG_JACOBI,					// projected Jacobi method
	mjALG_GS,						// projected Gauss-Seidel method
	mjALG_CG,						// projected conjugate gradient method
	mjALG_PCG,						// projected preconditioned conjugate gradient
	mjALG_NEWTON,					// projected Newton method
	mjALG_USER						// user-defined solver
} mjtAlgorithm;


typedef enum _mjtEq					// type of equality constraint
{
	mjEQ_POINT = 0,					// force two 3D points to coincide
	mjEQ_JOINT,						// couple two scalar joints with polynomial
	mjEQ_TENDON,					// set tendon length to constant
	mjEQ_USER						// user constraint
} mjtEq;


typedef enum _mjtWrap				// type of tendon wrap object
{
	mjWRAP_NONE = 0,				// null object
	mjWRAP_JOINT,					// constant moment arm
	mjWRAP_PULLEY,					// pulley used to split tendon
	mjWRAP_SITE,					// pass through site
	mjWRAP_SPHERE,					// wrap around sphere
	mjWRAP_CYLINDER					// wrap around (infinite) cylinder
} mjtWrap;


typedef enum _mjtDyn				// type of actuator dynamics
{
	mjDYN_NONE = 0,					// no internal dynamics; ctrl specifies force
	mjDYN_INTEGRATOR,				// simple integrator: adot = u
	mjDYN_FILTER,					// linear filter: adot = tau * (u-a)
	mjDYN_PNEUMATIC,				// model of air dynamics
	mjDYN_USER						// user dynamics model
} mjtDyn;


typedef enum _mjtTrn				// type of actuator transmission
{
	mjTRN_JOINT = 0,				// act on joint directly
	mjTRN_SLIDERCRANK,				// act on joint via slider-crank linkage
	mjTRN_TENDON,					// act on tendon
	mjTRN_SITE						// act on site z-axis
} mjtTrn;


typedef enum _mjtGain				// type of actuator gain
{
	mjGAIN_FIXED = 0,				// fixed gain
	mjGAIN_MUSCLE,					// model of muscle FVL curve
	mjGAIN_USER						// user gain model
} mjtGain;


typedef enum _mjtObj				// type of MujoCo object (for user constraints)
{
	mjOBJ_UNKNOWN = 0,
	mjOBJ_BODY,
	mjOBJ_JOINT,
	mjOBJ_GEOM,
	mjOBJ_MESH,
	mjOBJ_COLLISION,
	mjOBJ_SITE,
	mjOBJ_CONSTRAINT,
	mjOBJ_TENDON,
	mjOBJ_ACTUATOR,
	mjOBJ_CUSTOM,
} mjtObj;


typedef enum _mjtAType				// type of A-matrix for each group
{
	mjA_NONE = 0,					// do not compute A
	mjA_FAST,						// fast diagonal approximation to A
	mjA_DIAGONAL,						// exact diagonal of A
	mjA_FULL						// full A
} mjtAType;


typedef enum _mjtMouseMode			// mouse perturbation
{
	mjMOUSE_NONE = 0,				// no mouse (only space navigator)
	mjMOUSE_VERTICAL,				// push object in vertical (view) plane
	mjMOUSE_HORIZONAL				// push object in horizontal plane
} mjtMouseMode;


// forward declarations
struct _mjContact;
struct _mjContactPair;
struct _mjOption;
struct _mjModel;
struct _mjData;
struct _mjPerturb;
struct _mjGeomVis;
struct _mjDataVis;
typedef struct _mjContact mjContact;
typedef struct _mjContactPair mjContactPair;
typedef struct _mjOption mjOption;
typedef struct _mjModel mjModel;
typedef struct _mjData mjData;
typedef struct _mjPerturb mjPerturb;
typedef struct _mjGeomVis mjGeomVis;
typedef struct _mjDataVis mjDataVis;


//---------------------------------- function types (mjf) -------------------------------

typedef int (*mjfCallback)			// callback function type
	(const mjModel* model, mjData* data, void* custom);


typedef int (*mjfConFn)				// collision function type
	(const mjModel* m, const mjData* d, mjContact* con, int g1, int g2, mjtNum mindist);


//---------------------------------- mjContact and mjOption -----------------------------

// result of collision detection functions
struct _mjContact			
{
	int type;						// impulse group, between 1 and mjMAXGROUP-1
	int dim;						// contact space dimensionality: 1, 3, 4 or 6
	int obj1;						// geomid's for regular contacts
	int obj2;						//   bodyid's for consolidated contacts (in group 0)
	int flc_address;				// address in impulse Jacobian; -1: not set
	mjtNum dist;					// distance between nearest points; neg: penetration
	mjtNum mindist;					// mindist used to detect contact
	mjtNum pos[3];					// position of contact point: midpoint between geoms
	mjtNum frame[9];				// normal is in [0-2]
	mjtNum friction[5];				// tangent1, 2, spin, roll1, 2
};


// predefined contact pair
struct _mjContactPair			
{
	int type;						// same as in mjContact
	int dim;						// same as in mjContact
	int geom1;						// id of geom1
	int geom2;						// id of geom2
	mjtNum mindist;					// safety distance for collision detection
	mjtNum friction[3];				// tangent1/2, spin, roll1/2
};


// algorithm options
struct _mjOption
{
	// real-valued parameters
	mjtNum timestep;				// simulation timestep, used to init mjData.dt
	mjtNum gravity[3];				// gravitational acceleration
	mjtNum viscosity;				// cartesian viscosity, applied to bodies
	mjtNum wind[3];					// reference velocity for viscosity
	mjtNum eqerrreduce;				// error reduction for constraint
	mjtNum imperrreduce;			// error reduction for impulse
	mjtNum posthoccoef;				// M + coef*Je'*Je
	mjtNum softclamp;				// beta^2 for soft clamp function; 0: hard clamp
	mjtNum reginv;					// regularizer for inverse actuation
	mjtNum regimpulse;				// regularizer for A matrix in impulse solver
	mjtNum mindistscl;				// scale all mindist values

	// discrete flags
	int disableflags;				// bit flags for disabling features
	int integrator;					// integration mode (mjtIntegrator)
	int collisionmode;				// collision mode (mjtCollision)
	int sweeps;						// number of sweeps over impulse groups
	mjtByte posthoc;				// when enabled, apply control/external after impulse
	mjtByte consolidate;			// consolidate contacts on the same body pair
	mjtByte remotecontact;			// contact force from distance (f_scale, vmin_clamp)
	mjtByte linesearch;				// use linesearch in iterative impulse solvers

	// impulse group options
	int algorithm[mjMAXGROUP];		// impulse algorithm (mjtAlgorithm)
	int atype[mjMAXGROUP];			// type of A matrix (mjtAType)
	int maxiter[mjMAXGROUP];		// number of iterations for iterative algorithms
};


//---------------------------------- mjModel --------------------------------------------

struct _mjModel
{
	// sizes needed at mjModel construction
	int nq;							// number of generalized coordinates = dim(qpos)
	int nv;							// number of degrees of freedom = dim(qvel)
	int nu2;						// number of 2nd-order actuators/controls
	int nu3;						// number of 3rd-order actuators/controls
	int nbody;						// number of bodies
	int njnt;						// number of joints
	int ngeom;						// number of geoms
	int nsite;						// number of sites
	int nmesh;						// number of meshes
	int nmeshvert;					// number of vertices in all meshes
	int nmeshface;					// number of triangular faces in all meshes
	int nmeshaux;					// number of shorts in mesh auxiliary data
	int	ncpair;						// number of geom pairs in pair array
	int neq;						// number of equality constraints
	int ntendon;					// number of tendons
	int nwrap;						// number of wrap objects in all tendon paths
	int ncustom;					// number of custom fields
	int ncustomdata;				// number of mjtNums in all custom fields
	int nnames;						// number of chars in all names

	// sizes set after mjModel construction (only affect mjData)
	int nM;							// number of non-zeros in sparse inertia matrix
	int nemax;						// number of potential (scalar) constraints
	int	nlmax;						// number of potential limits
	int	ncmax;						// number of potential contacts
	int njmax;						// number of available rows in impulse Jacobian
	int ngrp;						// number of impulse groups
	int nuserdata;					// number of extra fields in mjData
	int nuserstack;					// number of extra fields in mjData stack

	// size computed during mjModel construction
	int nbuffer;					// number of bytes in buffer

	// algorithm options
	mjOption option;				// options that can be changed in runtime

	// ------------------------------- end of info header

	// buffer allocated on heap
	int		  npad;					// alignment offset, set by mj_setPtrModel
	void*	  buffer;				// main buffer (all pointers point in it)	(nbuffer+7)

	// generalized coordinates
	mjtNum*	  qpos0;				// qpos values at default pose				(nq x 1)
	mjtNum*	  qpos_spring;			// reference pose for springs				(nq x 1)

	// bodies
	int*	  body_parentid;		// id of body's parent						(nbody x 1)
	int*  	  body_rootid;			// id of (free or world) root above body	(nbody x 1)
	int*	  body_jntnum;			// number of joints for this body			(nbody x 1)
	int*	  body_jntadr;			// start addr of joints; -1: no joints		(nbody x 1)
	int*	  body_dofnum;			// number of motion degrees of freedom		(nbody x 1)
	int*	  body_dofadr;			// start addr of dofs; -1: no dofs			(nbody x 1)
	int*	  body_geomnum;			// number of geoms							(nbody x 1)
	int*	  body_geomadr;			// start addr of geoms; -1: no geoms		(nbody x 1)
	int*  	  body_tag;				// user tag									(nbody x 1)
	mjtNum*   body_pos;				// position offset rel. to parent body		(nbody x 3)
	mjtNum*   body_quat;			// orientation offset rel. to parent body	(nbody x 4)
	mjtNum*   body_ipos;			// local position of center of mass			(nbody x 3)
	mjtNum*   body_iquat;			// local orientation of inertia ellipsoid	(nbody x 4)
	mjtNum*   body_mass;			// mass										(nbody x 1)
	mjtNum*   body_inertia;			// diagonal inertia in ipos/iquat frame		(nbody x 3)
	mjtNum*   body_viscoef;			// viscosity coefficients in local frame	(nbody x 6)
	mjtNum*   body_invweight;		// mean inv inert in ref pose (trn, rot)	(nbody x 2)

	// joints
	int*	  jnt_type;				// type of joint (mjtJoint)					(njnt x 1)
	int*	  jnt_limtype;			// joint limit impulse type/group			(njnt x 1)
	int*	  jnt_qposadr;			// start addr in 'qpos' for joint's data	(njnt x 1)
	int*	  jnt_dofadr;			// start addr in 'qvel' for joint's data	(njnt x 1)
	int*	  jnt_bodyid;			// id of joint's body						(njnt x 1)
	int*	  jnt_tag;				// user tag									(njnt x 1)
	mjtNum*	  jnt_pos;				// local anchor position					(njnt x 3)
	mjtNum*	  jnt_axis;				// local joint axis							(njnt x 3)
	mjtNum*	  jnt_stiffness;		// stiffness coefficient					(njnt x 1)
	mjtNum*	  jnt_limit;			// joint limits								(njnt x 2)
	mjtNum*	  jnt_mindist;			// min distance for limit detection			(njnt x 1)

	// dofs
	int*	  dof_fritype;			// dof friction impulse type/group			(nv x 1)
	int*	  dof_bodyid;			// id of dof's body							(nv x 1)
	int*	  dof_jntid;			// id of dof's joint						(nv x 1)
	int*	  dof_parentid;			// id of dof's parent; -1: none				(nv x 1)
	int*	  dof_Madr;				// dof address in M-diagonal				(nv x 1)
	mjtNum*   dof_armature;			// dof armature inertia/mass				(nv x 1)
	mjtNum*	  dof_damping;			// damping coefficient						(nv x 1)
	mjtNum*   dof_frictionloss;		// dof friction loss						(nv x 1)
	mjtNum*   dof_maxvel;			// dof max velocity (neg: undefined)		(nv x 1)
	mjtNum*   dof_invweight;		// inv. diag. inertia in qpos0/spring		(nv x 1)

	// geoms
	int*	  geom_type;			// geometric type (mjtGeom)					(ngeom x 1)
	int*	  geom_contype;			// geom contact impulse type/group			(ngeom x 1)
	int*	  geom_condim;			// contact dimensionality (1, 3, 4, 6)		(ngeom x 1)
	int*	  geom_bodyid;			// id of geom's body						(ngeom x 1)
	int*	  geom_meshid;			// id of geom's mesh (-1: none)				(ngeom x 1)
	int*	  geom_colmask;			// collision masks							(ngeom x 2)
	int*	  geom_tag;				// user tag									(ngeom x 1)
	mjtNum*   geom_size;			// geom-specific size parameters			(ngeom x 3)
	mjtNum*   geom_rbound;			// radius of bounding sphere				(ngeom x 1)
	mjtNum*   geom_pos;				// local position offset rel. to body		(ngeom x 3)
	mjtNum*   geom_quat;			// local orientation offset rel. to body	(ngeom x 4)
	mjtNum*   geom_rgba;			// RGBA color								(ngeom x 4)
	mjtNum*   geom_friction;		// friction for (slide, roll, spin)			(ngeom x 3)
	mjtNum*   geom_mindist;			// min distance for contact detection		(ngeom x 1)

	// sites
	int*	  site_bodyid;			// id of site's body						(nsite x 1)
	int*	  site_group;			// used for rendering and user functions	(nsite x 1)
	int*	  site_tag;				// user tag									(nsite x 1)
	mjtNum*   site_pos;				// local position offset rel. to body		(nsite x 3)
	mjtNum*   site_quat;			// local orientation offset rel. to body	(nsite x 4)

	// mesh descriptors
	int*	  mesh_type;			// mesh type (mjtMesh)						(nmesh x 1)
	int*	  mesh_faceadr;			// first face address						(nmesh x 1)
	int*	  mesh_facenum;			// number of faces (heightfield: 0)			(nmesh x 1)
	int*	  mesh_auxadr;			// auxiliary data address					(nmesh x 1)
	int*	  mesh_auxnum;			// number of auxiliary data fields			(nmesh x 1)

	// mesh data
	float*	  mesh_vert;			// all vertex data							(nmeshvert x 3)
	int*	  mesh_face;			// all triangle face data					(nmeshface x 3)
	int*	  mesh_aux;				// all auxiliary data						(nmeshaux x 1)

	// collision pairs
	mjContactPair* pair;			// geom pairs checked for collision			(ncpair x 1)

	// equality constraints
	int*	  eq_type;				// constraint type (mjtEq)					(neq x 1)
	int*	  eq_obj1type;			// type of object 1 (mjtObj)				(neq x 1)
	int*	  eq_obj2type;			// type of object 2 (mjtObj)				(neq x 1)
	int*	  eq_obj1id;			// id of object 1							(neq x 1)
	int*	  eq_obj2id;			// id of object 2							(neq x 1)
	int*	  eq_size;				// number of scalar constraints				(neq x 1)
	int*	  eq_ndata;				// size of eq_data							(neq x 1)
	int*	  eq_tag;				// user tag									(neq x 1)
	mjtByte*  eq_isactive;			// enable/disable constraint				(neq x 1)
	mjtNum*	  eq_data;				// numeric data for constraint				(neq x 6)

	// tendons
	int*	  tendon_limtype;		// length limit impulse type/group			(ntendon x 1)
	int*      tendon_adr;			// address of first object in tendon's path (ntendon x 1)
	int*	  tendon_num;			// number of objects in tendon's path		(ntendon x 1)
	int*	  tendon_tag;			// user tag									(ntendon x 1)
	mjtNum*	  tendon_limit;			// tendon length limits						(ntendon x 2)
	mjtNum*	  tendon_mindist;		// min distance for limit detection			(ntendon x 1)
	mjtNum*	  tendon_stiffness;		// stiffness coefficient					(ntendon x 1)
	mjtNum*	  tendon_damping;		// damping coefficient						(ntendon x 1)
	mjtNum*	  tendon_reflen;		// length in qpos0/spring					(ntendon x 1)
	mjtNum*	  tendon_invweight;		// average inv. weight in qpos0/spring		(ntendon x 1)

	// list of all wrap objects in tendon paths
	int*	  wrap_type;			// wrap object type (mjtWrap)				(nwrap x 1)
	int*	  wrap_objid;			// object id: geom, site, joint				(nwrap x 1)
	mjtNum*	  wrap_prm;				// divisor, joint coef, or site id			(nwrap x 1)

	// actuators (2nd-order followed by 3rd-order)
	int*	  actuator_dyntype;		// dynamics type (mjtDyn)					(nu2+nu3 x 1)
	int*	  actuator_trntype;		// transmission type (mjtTrn)				(nu2+nu3 x 1)
	int*	  actuator_gaintype;	// gain type (mjtGain)						(nu2+nu3 x 1)
	int*	  actuator_trnid;		// transmission id: joint, tendon, site		(nu2+nu3 x 2)
	int*	  actuator_tag;			// user tag									(nu2+nu3 x 1)
	mjtByte*  actuator_islimited;	// is ctrl/act limited						(nu2+nu3 x 1)
	mjtNum*	  actuator_dynprm;		// dynamics parameters						(nu2+nu3 x mjDPRM)
	mjtNum*	  actuator_trnprm;		// transmission parameters					(nu2+nu3 x mjTPRM)
	mjtNum*	  actuator_gainprm;		// gain: scaling from act/ctrl to force		(nu2+nu3 x mjGPRM)
	mjtNum*	  actuator_limit;		// limits on ctrl (2nd) or act (3rd-order)	(nu2+nu3 x 2)
	mjtNum*	  actuator_reflen;		// actuator length in qpos0/spring			(nu2+nu3 x 1)

	// custom parameters
	int*	  custom_adr;			// address of custom field in custom_data	(ncustom)
	int*	  custom_size;			// size of custom field						(ncustom)
	int*	  custom_tag;			// user tag									(ncustom)
	mjtNum*   custom_data;			// array of all custom fields				(ncustomdata)

	// names
	int*	  name_bodyadr;			// body name pointers					    (nbody x 1)
	int*	  name_jntadr;			// joint name pointers					    (njnt x 1)
	int*	  name_geomadr;			// geom name pointers						(ngeom x 1)
	int*	  name_siteadr;			// site name pointers						(nsite x 1)
	int*	  name_meshadr;			// mesh name pointers						(nmesh x 1)
	int*	  name_eqadr;			// equality constraint name pointers		(neq x 1)
	int*	  name_tendonadr;		// tendon name pointers					    (ntendon x 1)
	int*	  name_actuatoradr;		// actuator name pointers				    (nu2+nu3 x 1)
	int*	  name_customadr;		// custom name pointers					    (ncustom x 1)
	char*	  names;				// names of all objects, 0-terminated		(nnames x 1)
};


//---------------------------------- mjData ---------------------------------------------

struct _mjData
{
	// constant sizes
	int  nstack;					// number of mjtNums that can fit in stack
	int  nbuffer;					// size of main buffer in bytes

	// variable sizes
	int  ne;						// size of equality constraint Jacobian
	int  ncon;						// total number of detected contacts
	int  nflc;						// number of rows in flc_J
	int  nnz;						// number of non-zeros in flc_J
	int  nwarning[mjMAXWARNING];	// how many times is each warning type generated

	// numeric variables
	mjtNum dt;						// actual dt; user can make it different from m->timestep
	mjtNum time;					// simulation time
	mjtNum com[3];					// center of mass
	mjtNum energy[2];				// kinetic and potential energy

	//-------------------------------- end of info header

	// buffer allocated on heap
	int		  npad;					// buffer alignment offset, set by mj_setPtrData
	void*	  buffer;				// main buffer; all pointers point in it	(nbuffer bytes)

	// stack
	int		  pstack;				// first available mjtNum address in stack
	int		  nspad;				// stack alignment offset, set by mj_setPtrData
	mjtNum*	  stack;				// stack buffer								(nstack mjtNums)

	//-------------------------------- buffer content starts here

	// persistent user data
	mjtNum*	  userdata;				// user buffer								(nuserdata x 1)

	// 2nd-order state
	mjtNum*	  qpos;					// generalized position						(nq x 1)
	mjtNum*   qvel;					// generalized velocity						(nv x 1)
	mjtNum*   qvel_next;			// next-step generalized velocity			(nv x 1)

	// 3rd-order state
	mjtNum*	  act;					// actuator activation						(nu3 x 1)
	mjtNum*	  act_dot;				// time-derivative of activation			(nu3 x 1)

	// control (2nd-order followed by 3rd-order)
	mjtNum*	  ctrl;					// control vector							(nu2+nu3 x 1)

	// generalized forces
	mjtNum*   qfrc_bias;			// interaction and gravitational forces		(nv x 1)
	mjtNum*   qfrc_passive;			// passive forces							(nv x 1)
	mjtNum*   qfrc_actuation;		// actuation forces							(nv x 1)
	mjtNum*   qfrc_external;		// external forces							(nv x 1)
	mjtNum*   qfrc_impulse;			// impulse forces (divided by dt)			(nv x 1)

	// computed by 'mj_kinematics'
	mjtNum*   xpos;					// Cartesian position of body frame			(nbody x 3)
	mjtNum*   xquat;				// Cartesian orientation of body frame		(nbody x 4)
	mjtNum*   xmat;					// Cartesian orientation of body frame		(nbody x 9)
	mjtNum*   xipos;				// Cartesian position of body com			(nbody x 3)
	mjtNum*   ximat;				// Cartesian orientation of body inertia	(nbody x 9)
	mjtNum*   xanchor;				// Cartesian position of joint anchor		(njnt x 3)
	mjtNum*   xaxis;				// Cartesian joint axis						(njnt x 3)
	mjtNum*   geom_xpos;			// Cartesian geom position					(ngeom x 3)
	mjtNum*   geom_xmat;			// Cartesian geom orientation				(ngeom x 9)
	mjtNum*   site_xpos;			// Cartesian site position					(nsite x 3)
	mjtNum*   site_xmat;			// Cartesian site orientation				(nsite x 9)

	// computed by 'mj_com'
	mjtNum*   cdof;					// com-based motion axis of each dof		(nv x 6)
	mjtNum*   cinert;				// com-based body inertia and mass			(nbody x 10)

	// computed by 'mj_tendon'
	int*	  ten_wrapadr;			// start address of tendon's path			(ntendon x 1)
	int*	  ten_wrapnum;			// number of wrap points in path			(ntendon x 1)
	mjtNum*	  ten_length;			// tendon lengths							(ntendon x 1)
	mjtNum*	  ten_moment;			// tendon moment arms						(ntendon x nv)
	int*	  wrap_obj;				// geom id; -1: site; -2: pulley			(nwrap*2 x 1)
	mjtNum*	  wrap_xpos;			// Cartesian 3D points in all path			(nwrap*2 x 3)

	// computed by 'mj_transmission'
	mjtNum*	  actuator_length;		// actuator lengths							(nu2+nu3 x 1)
	mjtNum*	  actuator_moment;		// actuator moment arms						(nu2+nu3 x nv)

	// computed by 'mj_solveActuation'
	mjtNum*	  actuator_force;		// actuator force							(nu2+nu3 x 1)

	// computed by 'mj_rne'... replace with ivel ???
	mjtNum*   cvel;					// com-based velocity [3D rot; 3D trans]	(nbody x 6)

	// computed by 'mj_crb'
	mjtNum*   qM;					// generalized inertia matrix				(nM x 1)

	// computed by 'mj_factorM'
	mjtNum*   qLD;					// L'*D*L factorization of M				(nM x 1)
	mjtNum*   qDiag;				// 1/sqrt(diag(D))							(nv x 1)

	// computed by 'mj_makeConstraint' 
	mjtNum*	  eq_err;				// equality constraint violations			(nemax x 1)
	mjtNum*	  eq_J;					// equality constraint Jacobian				(nemax x nv)

	// computed by 'mj_solveVelocity'
	mjtNum*	  eq_vdes;				// desired next-step constraint velocity	(nemax x 1)

	// computed by 'mj_projectConstraint'
	mjtNum*	  eq_JMi;				// Je * inv(M)								(nemax x nv)
	mjtNum*	  eq_Achol;				// Cholesky(Je * inv(M) * Je')				(nemax x nemax)

	// computed by 'mj_addContact'
	mjContact* contact;				// list of all detected contacts			(ncmax x 1)

	// computed by 'mj_makeImpulse'
	int*	  adr_fri;				// start address in fri_id					(ngrp+1 x 1)
	int*	  adr_lim;				// start address in lim_id					(ngrp+1 x 1)
	int*	  adr_con;				// start address in con_id					(ngrp+1 x 1)
	int*	  adr_lc;				// start address in lc_						(ngrp+1 x 1)
	int*	  adr_flc;				// start address in flc_					(ngrp+1 x 1)
	int*	  adr_A;				// start address in flc_A (el. number)		(ngrp+1 x 1)
	int*	  fri_id;				// frictional dof id						(nv x 1)
	int*	  lim_id;				// joint | njnt+tendon | njnt+nten+contact	(nlmax x 1)
	int*	  con_id;				// contact id (index in contact)			(ncmax x 1)
	int*	  lc_ind;				// limit and contact normal indices in flc	(nlmax+ncmax x 1)
	mjtNum*   lc_dist;				// normal distance (neg: penetration)		(nlmax+ncmax x 1)
	mjtNum*   lc_mindist;			// mindist used to detect limit/contact		(nlmax+ncmax x 1)
	int*	  flc_Jrowadr;			// start address of each row in colind		(njmax+1 x 1)
	int*	  flc_Jcolind;			// column indices in sparse Jacobian		(njmax x nv)
	mjtNum*   flc_Jdata;			// elements of sparse Jacobian				(njmax x nv)
	mjtNum*   flc_scale;			// flc_f *= flc_scale						(njmax x 1)

	// computed by 'mj_projectImpulse'
	mjtNum*   flc_A;				// Jc * P(M,Je) * Jc'; sqrt(diag) for PCG	(njmax x njmax)

	// computed by 'mj_impulseVel'
	mjtNum*   lc_vmin;				// minimal next-step normal velocity		(nlmax+ncmax x 1)
	mjtNum*   flc_v0;				// impulse-space velocity before impulse	(njmax x 1)
	mjtNum*   flc_b;				// b = vdes-v0; solve Af = b s.t. Cone(f)	(njmax x 1)

	// computed by 'mj_solveImpulse'
	mjtNum*   flc_f;				// impluse force							(njmax x 1)
};


//--------------------------- mjPerturb, mjGeomVis, mjDataVis ---------------------------

struct _mjPerturb
{
	// selection and mode info
	int		seltype;				// type of selected object (mjtObj)
	int		selid;					// id of selected object

	// perturbation info
	mjtNum	mouse[3];				// mouse position
	mjtNum	target[3];				// point where 3D force/torque is applied
	mjtNum	force[3];				// 3D force
	mjtNum	torque[3];				// 3D torque
};


struct _mjGeomVis
{
	int		type;					// geom type (mjtGeom)
	int		meshid;					// meshid for mesh, -1 for built-in types
	mjtNum	pos[3];					// Cartesian position
	mjtNum	mat[9];					// Cartesian orientation
	mjtNum	size[3];				// size parameters
	mjtNum	rgba[4];				// color and transparency
	mjtByte	visible;				// is geom visible
	mjtByte shadow;					// does geom cast shadow
	mjtByte reflection;				// does geom cast reflection
	mjtByte selectable;				// is geom selectable
};


struct _mjDataVis
{
	// constants
	int nbuffer;					// size of main buffer in bytes
	int nconmax;					// allocated contacts = min(m->ncmax, mjMAXCONVIS)

	// variables set directly by user
	char text[mjTXTVIS];			// text info
	mjGeomVis object[mjOBJVIS];		// additional geoms to visualize
	mjPerturb perturb;				// perturbation info

	// variables from mjData
	int	ncon;						// number of contacts
	mjtNum com[3];					// center of mass

	// buffer allocated on heap
	void*	  buffer;				// main buffer; all pointers point in it	(nbuffer bytes)

	// quantities copied from mjData
	mjtNum*   xpos;					// Cartesian position of body frame			(nbody x 3)
	mjtNum*   xmat;					// Cartesian orientation of body frame		(nbody x 9)
	mjtNum*   xipos;				// Cartesian position of body CoM			(nbody x 3)
	mjtNum*   ximat;				// Cartesian orientation of body inertia	(nbody x 9)
	mjtNum*   xanchor;				// Cartesian position of joint anchor		(njnt x 3)
	mjtNum*   xaxis;				// Cartesian joint axis						(njnt x 3)
	mjtNum*   geom_xpos;			// Cartesian geom position					(ngeom x 3)
	mjtNum*   geom_xmat;			// Cartesian geom orientation				(ngeom x 9)
	mjtNum*   site_xpos;			// Cartesian site position					(nsite x 3)
	mjtNum*   site_xmat;			// Cartesian site orientation				(nsite x 9)
	int*	  ten_wrapadr;			// start address of tendon's path			(ntendon x 1)
	int*	  ten_wrapnum;			// number of wrap points in path			(ntendon x 1)
	int*	  wrap_obj;				// geom id; -1: site; -2: pulley			(nwrap*2 x 1)
	mjtNum*	  wrap_xpos;			// Cartesian 3D points in all path			(nwrap*2 x 3)
	mjtNum*	  actuator_force;		// actuator forces							(nu2+nu3 x 1)
	mjtNum*	  actuator_line;		// lines of action on selected object		(nu2+nu3 x 3)

	// impulse-related quantities extracted from mjData
	mjtByte*  jnt_islimit;			// is joint limit active					(njnt x 1)
	mjtByte*  ten_islimit;			// is tendon limit active					(ntendon x 1)
	int*	  con_type;				// contact type/group						(nconmax x 1)
	int*	  con_dim;				// contact dimensionality					(nconmax x 1)
	int*	  con_obj1;				// id of contacting geom/body 1				(nconmax x 1)
	int*	  con_obj2;				// id of contacting geom/body 2				(nconmax x 1)
	mjtNum*	  con_pos;				// contact position							(nconmax x 3)
	mjtNum*	  con_frame;			// contact frame							(nconmax x 9)
	mjtNum*	  con_force;			// contact force, in local frame			(nconmax x 6)
};


//---------------------------------- macros ---------------------------------------------

// direct stack access; used in mj_engine_util
#define mjSTACKPTR d->stack+d->pstack, d->nstack-d->pstack

// mark and free stack
#define mjMARKSTACK int _mark = d->pstack;
#define mjFREESTACK d->pstack = _mark;

// check disabled flag
#define mjDISABLED(x) (m->option.disableflags & (x))


#pragma pack(pop)
