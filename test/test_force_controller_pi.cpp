#include<admittance_controller/force_controller_pi.hpp>
#include<gtest/gtest.h>
#include<cmath>

TEST(PI_controller, convergence){
    double kp=0.001, ki=0.01, dt=0.01,v_max=0.15;
    double k_surface=2000.0;
    double vz=0.0;
    double f_meas=0.0,f_des=5.0,f_delta=0.0;

    //Initializing PI force controller
    admittance::ForceControllerPI pi_controller(kp,ki,dt,v_max);

    //Defining fake surface and running loop for 1000 steps
    int i=0;
    bool pass=false;
    
    while(i<1000){ 
        
        vz=pi_controller.update(f_des,f_meas);
        f_delta= - (vz*k_surface*dt);
        f_meas+=f_delta;    
        i++;
        }

    pass = std::abs(f_des - f_meas) <= 0.1;
    EXPECT_TRUE(pass);
}


TEST(PI_controller, anti_windup_recovers) {
  double kp = 0.001, ki = 0.01, dt = 0.01;
  double v_max = 0.001;            // tiny → saturates immediately
  
  admittance::ForceControllerPI pi(kp, ki, dt, v_max);

  // Drive a large constant error for many steps → forces saturation.
  for (int i = 0; i < 1000; ++i) {
    double vz = pi.update(50.0, 0.0);   // huge error, way past v_max
    EXPECT_NEAR(vz, -v_max, 1e-9);      // output pinned at the lower limit
  }

  // Flip the error sign. If the integrator had wound up, it would take
  // many steps to unwind. With anti-windup, it leaves saturation fast.
  int steps_to_recover = 0;
  for (int i = 0; i < 100; ++i) {
    double vz = pi.update(0.0, 50.0);   // error now strongly negative
    ++steps_to_recover;
    if (vz > -v_max) break;             // left the lower saturation limit
  }
  EXPECT_LT(steps_to_recover, 5);       // recovers almost immediately
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
