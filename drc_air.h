/************************************************************************/
/* SDFAST constants for "single support" model drc_ssl.sd */

/* Sizes */
#define AIR_N_Q 35
#define AIR_N_U 34
#define AIR_N_STATE (AIR_N_Q+AIR_N_U)

/* Indices for sdfast state */
#define S_AIR_ROOT_X 0
#define S_AIR_ROOT_Y 1
#define S_AIR_ROOT_Z 2
#define S_AIR_ROOT_Q0 3
#define S_AIR_ROOT_Q1 4
#define S_AIR_ROOT_Q2 5
#define S_AIR_ROOT_Q3 34

#define S_AIR_BACK_LBZ 6
#define S_AIR_BACK_MBY 7
#define S_AIR_BACK_UBX 8
#define S_AIR_NECK_AY 9

#define S_AIR_L_LEG_UHZ 10
#define S_AIR_L_LEG_MHX 11
#define S_AIR_L_LEG_LHY 12
#define S_AIR_L_LEG_KNY 13
#define S_AIR_L_LEG_UAY 14
#define S_AIR_L_LEG_LAX 15

#define S_AIR_R_LEG_UHZ 16
#define S_AIR_R_LEG_MHX 17
#define S_AIR_R_LEG_LHY 18
#define S_AIR_R_LEG_KNY 19
#define S_AIR_R_LEG_UAY 20
#define S_AIR_R_LEG_LAX 21

#define S_AIR_L_ARM_USY 22
#define S_AIR_L_ARM_SHX 23
#define S_AIR_L_ARM_ELY 24
#define S_AIR_L_ARM_ELX 25
#define S_AIR_L_ARM_UWY 26
#define S_AIR_L_ARM_MWX 27

#define S_AIR_R_ARM_USY 28
#define S_AIR_R_ARM_SHX 29
#define S_AIR_R_ARM_ELY 30
#define S_AIR_R_ARM_ELX 31
#define S_AIR_R_ARM_UWY 32
#define S_AIR_R_ARM_MWX 33

// velocities
#define S_AIR_ROOT_XD 35
#define S_AIR_ROOT_YD 36
#define S_AIR_ROOT_ZD 37
#define S_AIR_ROOT_WX 38
#define S_AIR_ROOT_WY 39
#define S_AIR_ROOT_YZ 40

#define S_AIR_BACK_LBZD 41
#define S_AIR_BACK_MBYD 42
#define S_AIR_BACK_UBXD 43
#define S_AIR_NECK_AYD 44

#define S_AIR_L_LEG_UHZD 45
#define S_AIR_L_LEG_MHXD 46
#define S_AIR_L_LEG_LHYD 47
#define S_AIR_L_LEG_KNYD 48
#define S_AIR_L_LEG_UAYD 49
#define S_AIR_L_LEG_LAXD 50

#define S_AIR_R_LEG_UHZD 51
#define S_AIR_R_LEG_MHXD 52
#define S_AIR_R_LEG_LHYD 53
#define S_AIR_R_LEG_KNYD 54
#define S_AIR_R_LEG_UAYD 55
#define S_AIR_R_LEG_LAXD 56

#define S_AIR_L_ARM_USYD 57
#define S_AIR_L_ARM_SHXD 58
#define S_AIR_L_ARM_ELYD 59
#define S_AIR_L_ARM_ELXD 60
#define S_AIR_L_ARM_UWYD 61
#define S_AIR_L_ARM_MWXD 62

#define S_AIR_R_ARM_USYD 63
#define S_AIR_R_ARM_SHXD 64
#define S_AIR_R_ARM_ELYD 65
#define S_AIR_R_ARM_ELXD 66
#define S_AIR_R_ARM_UWYD 67
#define S_AIR_R_ARM_MWXD 68

/**************/

// Body index definitions (for sdpointf, etc.)
#define AIR_BODY_GROUND -1

#define AIR_BODY_PELVIS 0
#define AIR_BODY_LTORSO 1
#define AIR_BODY_MTORSO 2
#define AIR_BODY_UTORSO 3
#define AIR_BODY_HEAD 4

#define AIR_BODY_L_UGLUT 5
#define AIR_BODY_L_LGLUT 6
#define AIR_BODY_L_ULEG 7
#define AIR_BODY_L_LLEG 8
#define AIR_BODY_L_TALUS 9
#define AIR_BODY_L_FOOT 10

#define AIR_BODY_R_UGLUT 11
#define AIR_BODY_R_LGLUT 12
#define AIR_BODY_R_ULEG 13
#define AIR_BODY_R_LLEG 14
#define AIR_BODY_R_TALUS 15
#define AIR_BODY_R_FOOT 16

#define AIR_BODY_L_CLAV 17
#define AIR_BODY_L_SCAP 18
#define AIR_BODY_L_UARM 19
#define AIR_BODY_L_LARM 20
#define AIR_BODY_L_FARM 21
#define AIR_BODY_L_HAND 22

#define AIR_BODY_R_CLAV 23
#define AIR_BODY_R_SCAP 24
#define AIR_BODY_R_UARM 25
#define AIR_BODY_R_LARM 26
#define AIR_BODY_R_FARM 27
#define AIR_BODY_R_HAND 28
// #define AIR_BODY_LOOP 29

#define AIR_N_BODY 29

// Joint index definitions (for sdhinget, etc.)
#define AIR_JOINT_ROOT 0

#define AIR_JOINT_BACK_LBZ 1
#define AIR_JOINT_BACK_MBY 2
#define AIR_JOINT_BACK_UBX 3
#define AIR_JOINT_NECK_AY 4

#define AIR_JOINT_L_LEG_UHZ 5
#define AIR_JOINT_L_LEG_MHX 6
#define AIR_JOINT_L_LEG_LHY 7
#define AIR_JOINT_L_LEG_KNY 8
#define AIR_JOINT_L_LEG_UAY 9
#define AIR_JOINT_L_LEG_LAX 10

#define AIR_JOINT_R_LEG_UHZ 11
#define AIR_JOINT_R_LEG_MHX 12
#define AIR_JOINT_R_LEG_LHY 13
#define AIR_JOINT_R_LEG_KNY 14
#define AIR_JOINT_R_LEG_UAY 15
#define AIR_JOINT_R_LEG_LAX 16

#define AIR_JOINT_L_ARM_USY 17
#define AIR_JOINT_L_ARM_SHX 18
#define AIR_JOINT_L_ARM_ELY 19
#define AIR_JOINT_L_ARM_ELX 20
#define AIR_JOINT_L_ARM_UWY 21
#define AIR_JOINT_L_ARM_MWX 22

#define AIR_JOINT_R_ARM_USY 23
#define AIR_JOINT_R_ARM_SHX 24
#define AIR_JOINT_R_ARM_ELY 25
#define AIR_JOINT_R_ARM_ELX 26
#define AIR_JOINT_R_ARM_UWY 27
#define AIR_JOINT_R_ARM_MWX 28

#define AIR_N_JOINTS 29

/************************************************************************/

/************************************************************************/

void drc_air_init(void);

void drc_air_stab(double velin,
		double posin);

void drc_air_printerr(FILE *fnum);

void drc_air_clearerr(void);

void drc_air_state(double timein,
		 double qin[AIR_N_Q],
		 double uin[AIR_N_U]);

void drc_air_pos(int body,
	       double pt[3],
	       double loc[3]);

void drc_air_vel(int body,
	       double pt[3],
	       double velo[3]);

void drc_air_orient(int body,
		  double dircos[3][3]);

void drc_air_angvel(int body,
		  double avel[3]);

void drc_air_trans(int frbod,
		 double ivec[3],
		 int tobod,
		 double ovec[3]);

void drc_air_getbtj(int joint,
		  double btjout[3]);

void drc_air_dc2quat(double dircos[3][3],
		   double *e1,
		   double *e2,
		   double *e3,
		   double *e4);

void drc_air_fmotion(double *time,
		   double state[AIR_N_STATE],
		   double dstate[AIR_N_STATE],
		   double dt,
		   double ctol,
		   int *flag,
		   double *errest,
		   int *err);

void drc_air_pointf(int body,
		  double point[3],
		  double force[3]);

void drc_air_hinget(int joint,
		  int axis,
		  double torque);

void drc_air_bodyt(int body,
		 double torque[3]);

void drc_air_mom(double lm[3],
	       double am[3],
	       double *ke);

void drc_air_sys(double *mtoto,
	       double cm[3],
	       double icm[3][3]);

void drc_air_getitj(int joint,
		    double itjout[3]);

void drc_air_itj(int joint,
		 double itjin[3]);

void drc_air_assemble(double time,
		      double state[AIR_N_STATE],
		      int lock[AIR_N_U],
		      double tol,
		      int maxevals,
		      int *fcnt,
		      int *err);

void drc_air_initvel(double time,
		     double state[AIR_N_STATE],
		     int lock[AIR_N_U],
		     double tol,
		     int maxevals,
		     int *fcnt,
		     int *err);

void drc_air_reac(double force[AIR_N_U][3],
		  double torque[AIR_N_U][3]);

/************************************************************************/
