/************************************************************************/
/* SDFAST constants for left leg only model drc_lleg.sd */

/* Sizes */
#define LLEG_N_Q 6
#define LLEG_N_U 6
#define LLEG_N_STATE (LLEG_N_Q+LLEG_N_U)

/* Indices for sdfast state */
#define S_LLEG_L_LEG_LAX 0
#define S_LLEG_L_LEG_UAY 1
#define S_LLEG_L_LEG_KNY 2
#define S_LLEG_L_LEG_LHY 3
#define S_LLEG_L_LEG_MHX 4
#define S_LLEG_L_LEG_UHZ 5

// velocities
#define S_LLEG_L_LEG_LAXD 6
#define S_LLEG_L_LEG_UAYD 7
#define S_LLEG_L_LEG_KNYD 8
#define S_LLEG_L_LEG_LHYD 9
#define S_LLEG_L_LEG_MHXD 10
#define S_LLEG_L_LEG_UHZD 11

/**************/

// Body index definitions (for sdpointf, etc.)
#define LLEG_BODY_GROUND -1
#define LLEG_BODY_L_FOOT 0
#define LLEG_BODY_L_TALUS 1
#define LLEG_BODY_L_LLEG 2
#define LLEG_BODY_L_ULEG 3
#define LLEG_BODY_L_LGLUT 4
#define LLEG_BODY_L_UGLUT 5
#define LLEG_BODY_PELVIS 6

// Joint index definitions (for sdhinget, etc.)
#define LLEG_JOINT_L_GROUND 0
#define LLEG_JOINT_L_LEG_LAX 1
#define LLEG_JOINT_L_LEG_UAY 2
#define LLEG_JOINT_L_LEG_KNY 3
#define LLEG_JOINT_L_LEG_LHY 4
#define LLEG_JOINT_L_LEG_MHX 5
#define LLEG_JOINT_L_LEG_UHZ 6
#define LLEG_N_JOINTS 7

/************************************************************************/

/************************************************************************/

void drc_lleg_init(void);

void drc_lleg_stab(double velin,
		double posin);

void drc_lleg_printerr(FILE *fnum);

void drc_lleg_clearerr(void);

void drc_lleg_state(double timein,
		 double qin[LLEG_N_Q],
		 double uin[LLEG_N_U]);

void drc_lleg_pos(int body,
	       double pt[3],
	       double loc[3]);

void drc_lleg_vel(int body,
	       double pt[3],
	       double velo[3]);

void drc_lleg_orient(int body,
		  double dircos[3][3]);

void drc_lleg_angvel(int body,
		  double avel[3]);

void drc_lleg_trans(int frbod,
		 double ivec[3],
		 int tobod,
		 double ovec[3]);

void drc_lleg_getbtj(int joint,
		  double btjout[3]);

void drc_lleg_dc2quat(double dircos[3][3],
		   double *e1,
		   double *e2,
		   double *e3,
		   double *e4);

void drc_lleg_fmotion(double *time,
		   double state[LLEG_N_STATE],
		   double dstate[LLEG_N_STATE],
		   double dt,
		   double ctol,
		   int *flag,
		   double *errest,
		   int *err);

void drc_lleg_pointf(int body,
		  double point[3],
		  double force[3]);

void drc_lleg_hinget(int joint,
		  int axis,
		  double torque);

void drc_lleg_bodyt(int body,
		 double torque[3]);

void drc_lleg_mom(double lm[3],
	       double am[3],
	       double *ke);

void drc_lleg_sys(double *mtoto,
	       double cm[3],
	       double icm[3][3]);

void drc_lleg_getitj(int joint,
		    double itjout[3]);

void drc_lleg_itj(int joint,
		 double itjin[3]);

void drc_lleg_assemble(double time,
		      double state[LLEG_N_STATE],
		      int lock[LLEG_N_U],
		      double tol,
		      int maxevals,
		      int *fcnt,
		      int *err);

void drc_lleg_initvel(double time,
		     double state[LLEG_N_STATE],
		     int lock[LLEG_N_U],
		     double tol,
		     int maxevals,
		     int *fcnt,
		     int *err);

void drc_lleg_reac(double force[LLEG_N_U][3],
		  double torque[LLEG_N_U][3]);

/************************************************************************/
