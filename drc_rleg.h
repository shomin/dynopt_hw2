/************************************************************************/
/* SDFAST constants for right leg only model drc_rleg.sd */

/* Sizes */
#define RLEG_N_Q 6
#define RLEG_N_U 6
#define RLEG_N_STATE (RLEG_N_Q+RLEG_N_U)

/* Indices for sdfast state */
#define S_RLEG_R_LEG_LAX 0
#define S_RLEG_R_LEG_UAY 1
#define S_RLEG_R_LEG_KNY 2
#define S_RLEG_R_LEG_LHY 3
#define S_RLEG_R_LEG_MHX 4
#define S_RLEG_R_LEG_UHZ 5

// velocities
#define S_RLEG_R_LEG_LAXD 6
#define S_RLEG_R_LEG_UAYD 7
#define S_RLEG_R_LEG_KNYD 8
#define S_RLEG_R_LEG_LHYD 9
#define S_RLEG_R_LEG_MHXD 10
#define S_RLEG_R_LEG_UHZD 11

/**************/

// Body index definitions (for sdpointf, etc.)
#define RLEG_BODY_GROUND -1
#define RLEG_BODY_R_FOOT 0
#define RLEG_BODY_R_TALUS 1
#define RLEG_BODY_R_RLEG 2
#define RLEG_BODY_R_ULEG 3
#define RLEG_BODY_R_LGLUT 4
#define RLEG_BODY_R_UGLUT 5
#define RLEG_BODY_PELVIS 6

// Joint index definitions (for sdhinget, etc.)
#define RLEG_JOINT_R_GROUND 0
#define RLEG_JOINT_R_LEG_LAX 1
#define RLEG_JOINT_R_LEG_UAY 2
#define RLEG_JOINT_R_LEG_KNY 3
#define RLEG_JOINT_R_LEG_LHY 4
#define RLEG_JOINT_R_LEG_MHX 5
#define RLEG_JOINT_R_LEG_UHZ 6
#define RLEG_N_JOINTS 7

/************************************************************************/

/************************************************************************/

void drc_rleg_init(void);

void drc_rleg_stab(double velin,
		double posin);

void drc_rleg_printerr(FILE *fnum);

void drc_rleg_clearerr(void);

void drc_rleg_state(double timein,
		 double qin[RLEG_N_Q],
		 double uin[RLEG_N_U]);

void drc_rleg_pos(int body,
	       double pt[3],
	       double loc[3]);

void drc_rleg_vel(int body,
	       double pt[3],
	       double velo[3]);

void drc_rleg_orient(int body,
		  double dircos[3][3]);

void drc_rleg_angvel(int body,
		  double avel[3]);

void drc_rleg_trans(int frbod,
		 double ivec[3],
		 int tobod,
		 double ovec[3]);

void drc_rleg_getbtj(int joint,
		  double btjout[3]);

void drc_rleg_dc2quat(double dircos[3][3],
		   double *e1,
		   double *e2,
		   double *e3,
		   double *e4);

void drc_rleg_fmotion(double *time,
		   double state[RLEG_N_STATE],
		   double dstate[RLEG_N_STATE],
		   double dt,
		   double ctol,
		   int *flag,
		   double *errest,
		   int *err);

void drc_rleg_pointf(int body,
		  double point[3],
		  double force[3]);

void drc_rleg_hinget(int joint,
		  int axis,
		  double torque);

void drc_rleg_bodyt(int body,
		 double torque[3]);

void drc_rleg_mom(double lm[3],
	       double am[3],
	       double *ke);

void drc_rleg_sys(double *mtoto,
	       double cm[3],
	       double icm[3][3]);

void drc_rleg_getitj(int joint,
		    double itjout[3]);

void drc_rleg_itj(int joint,
		 double itjin[3]);

void drc_rleg_assemble(double time,
		      double state[RLEG_N_STATE],
		      int lock[RLEG_N_U],
		      double tol,
		      int maxevals,
		      int *fcnt,
		      int *err);

void drc_rleg_initvel(double time,
		     double state[RLEG_N_STATE],
		     int lock[RLEG_N_U],
		     double tol,
		     int maxevals,
		     int *fcnt,
		     int *err);

void drc_rleg_reac(double force[RLEG_N_U][3],
		  double torque[RLEG_N_U][3]);

/************************************************************************/
