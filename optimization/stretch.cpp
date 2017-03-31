
struct Contact_pose {
    double x, y, z;
    int manip; // 0 => left foot, 1 => right foot, 2 => left hand, 3 => right hand
};


struct Force {
    double x, y, z;
};

/*
    cps: (0,0,0), (1,-1,0), ()


    QP model:

    minimize    0.8[(delta_x_1 - v_x_1)^2 + (delta_x_2 - v_x_2)^2 + (delta_x_3 - v_x_3)^2]
                + 0.2[(gamma(cp_1, cp_p) - gamma_footmin)(gamma(cp_1, cp_p) - gamma_footmax) + 
                (gamma(cp_2, cp_p) - gamma_footmin)(gamma(cp_2, cp_p) - gamma_footmax) + 
                (gamma(cp_3, cp_p) - gamma_footmin)(gamma(cp_3, cp_p) - gamma_footmax)]
    subject to  x + 2 y + 3 z >= 4
                x +   y       >= 1

   It solves it once as a continuous model, and once as an integer model.
*/

#include "gurobi_c++.h"

#include <vector>
#include <cmath>

using namespace std;

int
main(int   argc,
     char *argv[])
{
  try {
    vector<Contact_pose> poses{
        {0,0,0},
        {1,-1,0},
        {2, 0, 0}
    };

    vector<Force> forces{
        {0, 1, 0},
        {0, 1, 0},
        {1, 1, 0},
    };

    GRBEnv env = GRBEnv();

    GRBModel model = GRBModel(env);

    // Create variables

    GRBVar delta_x_mp = model.addVar(-2.0, 2.0, 0.0, GRB_CONTINUOUS, "delta_x_mp");
    GRBVar delta_y_mp = model.addVar(-2.0, 2.0, 0.0, GRB_CONTINUOUS, "delta_y_mp");
    GRBVar delta_z_mp = model.addVar(-2.0, 2.0, 0.0, GRB_CONTINUOUS, "delta_z_mp");
    GRBVar delta_theta_mp = model.addVar(-10.0, 10.0, 0.0, GRB_CONTINUOUS, "delta_theta_mp");

    GRBVar delta_si_1 = model.addVar(-2.0, 2.0, 0.0, GRB_CONTINUOUS, "delta_si_1");
    GRBVar delta_si_2 = model.addVar(-2.0, 2.0, 0.0, GRB_CONTINUOUS, "delta_si_2");
    GRBVar delta_si_3 = model.addVar(-2.0, 2.0, 0.0, GRB_CONTINUOUS, "delta_si_3");

    // Set objective

    GRBQuadExpr obj = ((delta_x_mp + delta_si_1) - forces[0].x)*((delta_x_mp + delta_si_1) - forces[0].x) +
                      (delta_y_mp - forces[0].y)*(delta_y_mp - forces[0].y) +
                      (delta_z_mp - forces[0].z)*(delta_z_mp - forces[0].z) + 

                      ((delta_x_mp - delta_theta_mp*1.414*-1 + delta_si_2 + delta_si_1) - forces[1].x)*((delta_x_mp - delta_theta_mp*1.414*-1 + delta_si_2 + delta_si_1) - forces[1].x) +
                      (delta_y_mp + delta_theta_mp*1.414*1 - forces[1].y)*(delta_y_mp + delta_theta_mp*1.414*1 - forces[1].y) +
                      (delta_z_mp - forces[1].z)*(delta_z_mp - forces[1].z) + 

                      ((delta_x_mp + delta_si_3 + delta_si_2 + delta_si_1) - forces[2].x)*((delta_x_mp + delta_si_3 + delta_si_2 + delta_si_1) - forces[2].x) +
                      (delta_y_mp + delta_theta_mp*2*1 - forces[2].y)*(delta_y_mp + delta_theta_mp*2*1 - forces[2].y)+
                      (delta_z_mp - forces[2].z)*(delta_z_mp - forces[2].z);




    // () + (x2,y,z)*(x2) + (x3)*(x2);
    model.setObjective(obj);

    // Add constraint: x + 2 y + 3 z >= 4

    // model.addConstr(x + 2 * y + 3 * z >= 4, "c0");

    // Add constraint: x + y >= 1

    // model.addConstr(x + y >= 1, "c1");

    // Optimize model

    model.optimize();

    cout << delta_x_mp.get(GRB_StringAttr_VarName) << " "
         << delta_x_mp.get(GRB_DoubleAttr_X) << endl;
    cout << delta_y_mp.get(GRB_StringAttr_VarName) << " "
         << delta_y_mp.get(GRB_DoubleAttr_X) << endl;
    cout << delta_z_mp.get(GRB_StringAttr_VarName) << " "
         << delta_z_mp.get(GRB_DoubleAttr_X) << endl;
    cout << delta_theta_mp.get(GRB_StringAttr_VarName) << " "
         << delta_theta_mp.get(GRB_DoubleAttr_X) << endl;
    cout << delta_si_1.get(GRB_StringAttr_VarName) << " "
         << delta_si_1.get(GRB_DoubleAttr_X) << endl;
    cout << delta_si_2.get(GRB_StringAttr_VarName) << " "
         << delta_si_2.get(GRB_DoubleAttr_X) << endl;
    cout << delta_si_3.get(GRB_StringAttr_VarName) << " "
         << delta_si_3.get(GRB_DoubleAttr_X) << endl;

    cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;

  } catch(GRBException e) {
    cout << "Error code = " << e.getErrorCode() << endl;
    cout << e.getMessage() << endl;
  } catch(...) {
    cout << "Exception during optimization" << endl;
  }

  return 0;
}
