#include "PID.h"
#include <iostream>
#include <uWS/uWS.h>

using namespace std;

PID::PID() {

}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {

    cout<<"Intialized!!"<<endl;
    p[0] = Kp_;
    p[1] = Ki_;
    p[2] = Kd_;

    d_error = 0;
    i_error = 0;
    p_error = 0;

    //paramets for twiddle use
    counter_iterations = 0;

    best_err = 0;
    total_error = 0;

    // values to start 'twiddling' with
    dp[0] = 0.025; // p
    dp[1] = 0.0005; // i
    dp[2] = 0.25; // d

    state = 0;
    index = 0;

}


double PID::UpdateError(double cte, uWS::WebSocket<uWS::SERVER> ws) {

    double error;
    p_error = cte;
    
    i_error += cte;
    

    error = (-p[0] * cte) + (-p[1] * i_error) + (-p[2] * (cte - d_error));
    d_error = cte;
    


// uncomment if Twiddle is used
/*
    if (counter_iterations > 250){
        TotalError(cte);
    }
    counter_iterations += 1;
    if (counter_iterations == 950)
    {
        twiddle();
        cout<< "Total Error " << total_error<<endl;
        total_error =0;
        counter_iterations = 0;
        Restart(ws);
    }
*/
    return error;
}


void PID::Restart(uWS::WebSocket<uWS::SERVER> ws){
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}


void PID::TotalError(double cte) {

    total_error += (cte * cte);

}


double PID::twiddle() {
    p_error = 0;
    d_error = 0;
    i_error = 0;

    //this if condition is only run once in the beginning
    if (state == 0)
    {
        state = 1;
        best_err = total_error;

        cout<< "state = 0, total_error " << total_error<<  " PID " << index
        <<" Best Error "<< best_err<<" Ks "<< p[0]<<" "<<p[1] <<" "<<p[2] << " dp[] " << dp[index] <<endl;

        p[index] += dp[index];
        return 0;
    }

    if (state == 1)
    {


        if (total_error < best_err ){

            cout<< "state = 1(total<best), total_error " << total_error << " PID " << index <<" Best Error "<< best_err<<" Ks "<< p[0]<<" "<<p[1] <<" "<<p[2] << " dp[] " << dp[index] <<endl;

            best_err = total_error;
            dp[index] *= 1.5;

            // move on to next parameter in PID
            index += 1;
            if (index == 3) index = 0;

            p[index] += dp[index];
               
        }

        else{
            cout<< "state = 1(total>best), total_error " << total_error << " PID " << index <<" Best Error "<< best_err<<" Ks "<< p[0]<<" "<<p[1] <<" "<<p[2] << " dp[] " << dp[index] <<endl;
            p[index] -= 2*dp[index];

            state = 2;

            return 0;
        }
     }

    if (state == 2)
    {
        if (total_error < best_err ){
            best_err = total_error;

            cout<< "state = 2(total<best), total_error " << total_error << " PID " << index <<" Best Error "<< best_err<<" Ks "<< p[0]<<" "<<p[1] <<" "<<p[2] << " dp[] " << dp[index] <<endl;
            dp[index] *= 1.5;

            // move on to next parameter in PID
            index += 1;
            if (index == 3) index = 0;

            p[index] += dp[index];
            state = 1;
        }
        else{
            cout<< "state = 2(total>best), total_error " << total_error << " PID " << index <<" Best Error "<< best_err<<" Ks "<< p[0]<<" "<<p[1] <<" "<<p[2] << " dp[] " << dp[index] <<endl;
            p[index] += dp[index];
            dp[index] *= 0.5;

            // move on to next parameter in PID
            index += 1;
            if (index == 3) index = 0;

            p[index] += dp[index];
            state = 1 ;
        }
    
    }

}