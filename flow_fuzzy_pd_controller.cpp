/*++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/*                                                      */
/* File:  flow_fuzzy_pd_controller.c                    */
/*                                                      */
/* Author: Automatically generated by Xfuzzy            */
/*                                                      */
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <stdio.h>
#include <math.h>
#include "flow_fuzzy_pd_controller.h"

/*======================================================*/
/*  MembershipFunction FMF_xfl_triangular               */
/*======================================================*/

/*------------------------------------------------------*/
/* Function to compute an equal relationship            */
/*------------------------------------------------------*/

static double FMF_xfl_triangular_equal(double x, int i, double min, double max, double step, double *p, int length) {
      double a = (i==0? min-1 : (i==1 ? min : p[i-2]));
      double b = (i==0? min : (i==length+1? max : p[i-1]));
      double c = (i==length? max : (i==length+1? max+1 : p[i]));
      return (a<x && x<=b? (x-a)/(b-a) : (b<x && x<c? (c-x)/(c-b) : 0));
}

/*======================================================*/
/*  MembershipFunction MF_xfl_singleton                 */
/*======================================================*/

/*------------------------------------------------------*/
/* Function to compute an equal relationship            */
/*------------------------------------------------------*/

static double MF_xfl_singleton_equal(double x, double min, double max, double step, double a) {
    return (x==a? 1 : 0); 

}

/*======================================================*/
/*  Operatorset OP_fuzzy_pd_ope                         */
/*======================================================*/

/*------------------------------------------------------*/
/* Description of the operator AND                      */
/*------------------------------------------------------*/

static double OP_fuzzy_pd_ope_And(double a, double b) {
    return a*b; 

}

/*------------------------------------------------------*/
/* Description of the defuzzification method            */
/*------------------------------------------------------*/

static double OP_fuzzy_pd_ope_Defuz(FuzzyNumber mf) {
 double min = mf.min;
 double max = mf.max;
     double num=0, denom=0;
     int i;
     for(i=0; i<mf.length; i++) {
      num += mf.degree[i] * mf.conc[i].center();
      denom += mf.degree[i];
     }
     if(denom==0) return (min+max)/2;
     return num/denom;

}


/*======================================================*/
/*  TypeLV TP_error_t                                   */
/*======================================================*/

/*------------------------------------------------------*/
/* Description of the family fam                        */
/*------------------------------------------------------*/

static double TP_error_t_fam_equal(double x, int i){
   double list[7] = {-0.75,-0.5,-0.25,0.0,0.25,0.5,0.75};
   return FMF_xfl_triangular_equal(x,i,-1.0,1.0,0.00784313725490196,list,7);
}

/*------------------------------------------------------*/
/* Description of the label NVB                         */
/*------------------------------------------------------*/

static double TP_error_t_NVB_equal(double x){
   return TP_error_t_fam_equal(x,0);}

/*------------------------------------------------------*/
/* Description of the label NB                          */
/*------------------------------------------------------*/

static double TP_error_t_NB_equal(double x){
   return TP_error_t_fam_equal(x,1);}

/*------------------------------------------------------*/
/* Description of the label NM                          */
/*------------------------------------------------------*/

static double TP_error_t_NM_equal(double x){
   return TP_error_t_fam_equal(x,2);}

/*------------------------------------------------------*/
/* Description of the label NS                          */
/*------------------------------------------------------*/

static double TP_error_t_NS_equal(double x){
   return TP_error_t_fam_equal(x,3);}

/*------------------------------------------------------*/
/* Description of the label Z                           */
/*------------------------------------------------------*/

static double TP_error_t_Z_equal(double x){
   return TP_error_t_fam_equal(x,4);}

/*------------------------------------------------------*/
/* Description of the label PS                          */
/*------------------------------------------------------*/

static double TP_error_t_PS_equal(double x){
   return TP_error_t_fam_equal(x,5);}

/*------------------------------------------------------*/
/* Description of the label PM                          */
/*------------------------------------------------------*/

static double TP_error_t_PM_equal(double x){
   return TP_error_t_fam_equal(x,6);}

/*------------------------------------------------------*/
/* Description of the label PB                          */
/*------------------------------------------------------*/

static double TP_error_t_PB_equal(double x){
   return TP_error_t_fam_equal(x,7);}

/*------------------------------------------------------*/
/* Description of the label PVB                         */
/*------------------------------------------------------*/

static double TP_error_t_PVB_equal(double x){
   return TP_error_t_fam_equal(x,8);}

/*======================================================*/
/*  TypeLV TP_change_error_t                            */
/*======================================================*/

/*------------------------------------------------------*/
/* Description of the family fam                        */
/*------------------------------------------------------*/

static double TP_change_error_t_fam_equal(double x, int i){
   double list[7] = {-0.75,-0.5,-0.25,0.0,0.25,0.5,0.75};
   return FMF_xfl_triangular_equal(x,i,-1.0,1.0,0.00784313725490196,list,7);
}

/*------------------------------------------------------*/
/* Description of the label NVB                         */
/*------------------------------------------------------*/

static double TP_change_error_t_NVB_equal(double x){
   return TP_change_error_t_fam_equal(x,0);}

/*------------------------------------------------------*/
/* Description of the label NB                          */
/*------------------------------------------------------*/

static double TP_change_error_t_NB_equal(double x){
   return TP_change_error_t_fam_equal(x,1);}

/*------------------------------------------------------*/
/* Description of the label NM                          */
/*------------------------------------------------------*/

static double TP_change_error_t_NM_equal(double x){
   return TP_change_error_t_fam_equal(x,2);}

/*------------------------------------------------------*/
/* Description of the label NS                          */
/*------------------------------------------------------*/

static double TP_change_error_t_NS_equal(double x){
   return TP_change_error_t_fam_equal(x,3);}

/*------------------------------------------------------*/
/* Description of the label Z                           */
/*------------------------------------------------------*/

static double TP_change_error_t_Z_equal(double x){
   return TP_change_error_t_fam_equal(x,4);}

/*------------------------------------------------------*/
/* Description of the label PS                          */
/*------------------------------------------------------*/

static double TP_change_error_t_PS_equal(double x){
   return TP_change_error_t_fam_equal(x,5);}

/*------------------------------------------------------*/
/* Description of the label PM                          */
/*------------------------------------------------------*/

static double TP_change_error_t_PM_equal(double x){
   return TP_change_error_t_fam_equal(x,6);}

/*------------------------------------------------------*/
/* Description of the label PB                          */
/*------------------------------------------------------*/

static double TP_change_error_t_PB_equal(double x){
   return TP_change_error_t_fam_equal(x,7);}

/*------------------------------------------------------*/
/* Description of the label PVB                         */
/*------------------------------------------------------*/

static double TP_change_error_t_PVB_equal(double x){
   return TP_change_error_t_fam_equal(x,8);}

/*======================================================*/
/*  TypeLV TP_output_t                                  */
/*======================================================*/

/*------------------------------------------------------*/
/* Description of the label NB                          */
/*------------------------------------------------------*/

static double TP_output_t_NB_equal(double x){
   return MF_xfl_singleton_equal(x,-1.0,1.0,0.00784313725490196,-1.0);
}

static double TP_output_t_NB_center(){
   return -1.0;
}

/*------------------------------------------------------*/
/* Description of the label NM                          */
/*------------------------------------------------------*/

static double TP_output_t_NM_equal(double x){
   return MF_xfl_singleton_equal(x,-1.0,1.0,0.00784313725490196,-0.6666666666666667);
}

static double TP_output_t_NM_center(){
   return -0.6666666666666667;
}

/*------------------------------------------------------*/
/* Description of the label NS                          */
/*------------------------------------------------------*/

static double TP_output_t_NS_equal(double x){
   return MF_xfl_singleton_equal(x,-1.0,1.0,0.00784313725490196,-0.33333333333333337);
}

static double TP_output_t_NS_center(){
   return -0.33333333333333337;
}

/*------------------------------------------------------*/
/* Description of the label Z                           */
/*------------------------------------------------------*/

static double TP_output_t_Z_equal(double x){
   return MF_xfl_singleton_equal(x,-1.0,1.0,0.00784313725490196,0.0);
}

static double TP_output_t_Z_center(){
   return 0.0;
}

/*------------------------------------------------------*/
/* Description of the label PS                          */
/*------------------------------------------------------*/

static double TP_output_t_PS_equal(double x){
   return MF_xfl_singleton_equal(x,-1.0,1.0,0.00784313725490196,0.33333333333333326);
}

static double TP_output_t_PS_center(){
   return 0.33333333333333326;
}

/*------------------------------------------------------*/
/* Description of the label PM                          */
/*------------------------------------------------------*/

static double TP_output_t_PM_equal(double x){
   return MF_xfl_singleton_equal(x,-1.0,1.0,0.00784313725490196,0.6666666666666665);
}

static double TP_output_t_PM_center(){
   return 0.6666666666666665;
}

/*------------------------------------------------------*/
/* Description of the label PB                          */
/*------------------------------------------------------*/

static double TP_output_t_PB_equal(double x){
   return MF_xfl_singleton_equal(x,-1.0,1.0,0.00784313725490196,1.0);
}

static double TP_output_t_PB_center(){
   return 1.0;
}

/*======================================================*/
/*  Rulebase RL_fuzzy_pd_rules                          */
/*======================================================*/

static void RL_fuzzy_pd_rules(double de, double e, double *out) {
 double _rl;

 double _out_degree[81];
 Consequent _out_conc[81];
 FuzzyNumber _out;
 _out.min = -1.0;
 _out.max = 1.0;
 _out.step = 0.00784313725490196;
 _out.length = 81;
 _out.degree = _out_degree;
 _out.conc = _out_conc;
 int _out_i = 0;

 double _de_eq[9];
 _de_eq[0] = TP_change_error_t_NVB_equal(de);
 _de_eq[1] = TP_change_error_t_NB_equal(de);
 _de_eq[2] = TP_change_error_t_NM_equal(de);
 _de_eq[3] = TP_change_error_t_NS_equal(de);
 _de_eq[4] = TP_change_error_t_Z_equal(de);
 _de_eq[5] = TP_change_error_t_PS_equal(de);
 _de_eq[6] = TP_change_error_t_PM_equal(de);
 _de_eq[7] = TP_change_error_t_PB_equal(de);
 _de_eq[8] = TP_change_error_t_PVB_equal(de);

 double _e_eq[9];
 _e_eq[0] = TP_error_t_NVB_equal(e);
 _e_eq[1] = TP_error_t_NB_equal(e);
 _e_eq[2] = TP_error_t_NM_equal(e);
 _e_eq[3] = TP_error_t_NS_equal(e);
 _e_eq[4] = TP_error_t_Z_equal(e);
 _e_eq[5] = TP_error_t_PS_equal(e);
 _e_eq[6] = TP_error_t_PM_equal(e);
 _e_eq[7] = TP_error_t_PB_equal(e);
 _e_eq[8] = TP_error_t_PVB_equal(e);

 _rl = OP_fuzzy_pd_ope_And(_de_eq[0],_e_eq[0]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_NB_equal;
 _out_conc[_out_i].center = TP_output_t_NB_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[0],_e_eq[1]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_NB_equal;
 _out_conc[_out_i].center = TP_output_t_NB_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[0],_e_eq[2]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_NB_equal;
 _out_conc[_out_i].center = TP_output_t_NB_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[0],_e_eq[3]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_NM_equal;
 _out_conc[_out_i].center = TP_output_t_NM_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[0],_e_eq[4]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_NS_equal;
 _out_conc[_out_i].center = TP_output_t_NS_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[0],_e_eq[5]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_Z_equal;
 _out_conc[_out_i].center = TP_output_t_Z_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[0],_e_eq[6]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_PS_equal;
 _out_conc[_out_i].center = TP_output_t_PS_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[0],_e_eq[7]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_PM_equal;
 _out_conc[_out_i].center = TP_output_t_PM_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[0],_e_eq[8]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_PB_equal;
 _out_conc[_out_i].center = TP_output_t_PB_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[1],_e_eq[0]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_NB_equal;
 _out_conc[_out_i].center = TP_output_t_NB_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[1],_e_eq[1]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_NB_equal;
 _out_conc[_out_i].center = TP_output_t_NB_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[1],_e_eq[2]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_NB_equal;
 _out_conc[_out_i].center = TP_output_t_NB_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[1],_e_eq[3]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_NM_equal;
 _out_conc[_out_i].center = TP_output_t_NM_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[1],_e_eq[4]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_NS_equal;
 _out_conc[_out_i].center = TP_output_t_NS_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[1],_e_eq[5]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_Z_equal;
 _out_conc[_out_i].center = TP_output_t_Z_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[1],_e_eq[6]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_PS_equal;
 _out_conc[_out_i].center = TP_output_t_PS_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[1],_e_eq[7]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_PM_equal;
 _out_conc[_out_i].center = TP_output_t_PM_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[1],_e_eq[8]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_PB_equal;
 _out_conc[_out_i].center = TP_output_t_PB_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[2],_e_eq[0]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_NB_equal;
 _out_conc[_out_i].center = TP_output_t_NB_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[2],_e_eq[1]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_NB_equal;
 _out_conc[_out_i].center = TP_output_t_NB_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[2],_e_eq[2]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_NB_equal;
 _out_conc[_out_i].center = TP_output_t_NB_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[2],_e_eq[3]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_NM_equal;
 _out_conc[_out_i].center = TP_output_t_NM_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[2],_e_eq[4]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_NS_equal;
 _out_conc[_out_i].center = TP_output_t_NS_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[2],_e_eq[5]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_Z_equal;
 _out_conc[_out_i].center = TP_output_t_Z_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[2],_e_eq[6]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_PS_equal;
 _out_conc[_out_i].center = TP_output_t_PS_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[2],_e_eq[7]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_PM_equal;
 _out_conc[_out_i].center = TP_output_t_PM_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[2],_e_eq[8]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_PB_equal;
 _out_conc[_out_i].center = TP_output_t_PB_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[3],_e_eq[0]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_NB_equal;
 _out_conc[_out_i].center = TP_output_t_NB_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[3],_e_eq[1]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_NB_equal;
 _out_conc[_out_i].center = TP_output_t_NB_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[3],_e_eq[2]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_NB_equal;
 _out_conc[_out_i].center = TP_output_t_NB_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[3],_e_eq[3]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_NM_equal;
 _out_conc[_out_i].center = TP_output_t_NM_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[3],_e_eq[4]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_NS_equal;
 _out_conc[_out_i].center = TP_output_t_NS_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[3],_e_eq[5]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_Z_equal;
 _out_conc[_out_i].center = TP_output_t_Z_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[3],_e_eq[6]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_PS_equal;
 _out_conc[_out_i].center = TP_output_t_PS_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[3],_e_eq[7]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_PM_equal;
 _out_conc[_out_i].center = TP_output_t_PM_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[3],_e_eq[8]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_PB_equal;
 _out_conc[_out_i].center = TP_output_t_PB_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[4],_e_eq[0]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_NB_equal;
 _out_conc[_out_i].center = TP_output_t_NB_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[4],_e_eq[1]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_NB_equal;
 _out_conc[_out_i].center = TP_output_t_NB_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[4],_e_eq[2]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_NB_equal;
 _out_conc[_out_i].center = TP_output_t_NB_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[4],_e_eq[3]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_NM_equal;
 _out_conc[_out_i].center = TP_output_t_NM_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[4],_e_eq[4]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_NS_equal;
 _out_conc[_out_i].center = TP_output_t_NS_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[4],_e_eq[5]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_Z_equal;
 _out_conc[_out_i].center = TP_output_t_Z_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[4],_e_eq[6]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_PS_equal;
 _out_conc[_out_i].center = TP_output_t_PS_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[4],_e_eq[7]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_PB_equal;
 _out_conc[_out_i].center = TP_output_t_PB_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[4],_e_eq[8]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_PB_equal;
 _out_conc[_out_i].center = TP_output_t_PB_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[5],_e_eq[0]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_PS_equal;
 _out_conc[_out_i].center = TP_output_t_PS_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[5],_e_eq[1]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_NB_equal;
 _out_conc[_out_i].center = TP_output_t_NB_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[5],_e_eq[2]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_NM_equal;
 _out_conc[_out_i].center = TP_output_t_NM_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[5],_e_eq[3]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_NS_equal;
 _out_conc[_out_i].center = TP_output_t_NS_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[5],_e_eq[4]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_Z_equal;
 _out_conc[_out_i].center = TP_output_t_Z_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[5],_e_eq[5]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_PS_equal;
 _out_conc[_out_i].center = TP_output_t_PS_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[5],_e_eq[6]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_PM_equal;
 _out_conc[_out_i].center = TP_output_t_PM_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[5],_e_eq[7]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_PB_equal;
 _out_conc[_out_i].center = TP_output_t_PB_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[5],_e_eq[8]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_PB_equal;
 _out_conc[_out_i].center = TP_output_t_PB_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[6],_e_eq[0]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_PS_equal;
 _out_conc[_out_i].center = TP_output_t_PS_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[6],_e_eq[1]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_NB_equal;
 _out_conc[_out_i].center = TP_output_t_NB_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[6],_e_eq[2]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_NM_equal;
 _out_conc[_out_i].center = TP_output_t_NM_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[6],_e_eq[3]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_NS_equal;
 _out_conc[_out_i].center = TP_output_t_NS_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[6],_e_eq[4]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_Z_equal;
 _out_conc[_out_i].center = TP_output_t_Z_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[6],_e_eq[5]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_PS_equal;
 _out_conc[_out_i].center = TP_output_t_PS_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[6],_e_eq[6]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_PM_equal;
 _out_conc[_out_i].center = TP_output_t_PM_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[6],_e_eq[7]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_PB_equal;
 _out_conc[_out_i].center = TP_output_t_PB_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[6],_e_eq[8]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_PB_equal;
 _out_conc[_out_i].center = TP_output_t_PB_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[7],_e_eq[0]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_PM_equal;
 _out_conc[_out_i].center = TP_output_t_PM_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[7],_e_eq[1]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_NB_equal;
 _out_conc[_out_i].center = TP_output_t_NB_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[7],_e_eq[2]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_NM_equal;
 _out_conc[_out_i].center = TP_output_t_NM_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[7],_e_eq[3]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_NS_equal;
 _out_conc[_out_i].center = TP_output_t_NS_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[7],_e_eq[4]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_Z_equal;
 _out_conc[_out_i].center = TP_output_t_Z_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[7],_e_eq[5]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_PS_equal;
 _out_conc[_out_i].center = TP_output_t_PS_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[7],_e_eq[6]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_PM_equal;
 _out_conc[_out_i].center = TP_output_t_PM_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[7],_e_eq[7]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_PB_equal;
 _out_conc[_out_i].center = TP_output_t_PB_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[7],_e_eq[8]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_PB_equal;
 _out_conc[_out_i].center = TP_output_t_PB_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[8],_e_eq[0]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_PB_equal;
 _out_conc[_out_i].center = TP_output_t_PB_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[8],_e_eq[1]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_NB_equal;
 _out_conc[_out_i].center = TP_output_t_NB_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[8],_e_eq[2]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_NM_equal;
 _out_conc[_out_i].center = TP_output_t_NM_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[8],_e_eq[3]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_NS_equal;
 _out_conc[_out_i].center = TP_output_t_NS_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[8],_e_eq[4]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_Z_equal;
 _out_conc[_out_i].center = TP_output_t_Z_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[8],_e_eq[5]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_PB_equal;
 _out_conc[_out_i].center = TP_output_t_PB_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[8],_e_eq[6]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_PM_equal;
 _out_conc[_out_i].center = TP_output_t_PM_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[8],_e_eq[7]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_PB_equal;
 _out_conc[_out_i].center = TP_output_t_PB_center;
 _out_i++;

 _rl = OP_fuzzy_pd_ope_And(_de_eq[8],_e_eq[8]);
 _out_degree[_out_i] = _rl;
 _out_conc[_out_i].equal = TP_output_t_PB_equal;
 _out_conc[_out_i].center = TP_output_t_PB_center;
 _out_i++;

 *out = OP_fuzzy_pd_ope_Defuz(_out);
}


/*======================================================*/
/*                   Inference Engine                   */
/*======================================================*/

void flow_fuzzy_pd_controllerInferenceEngine(double change_error, double error, double *_d_output) {
 double output;
 RL_fuzzy_pd_rules(change_error, error, &output);
 *_d_output = output;
}

