#include "Regulators/PIDController.h"


using namespace pid;


PIDController::PIDController(double Kp, double Ki, double Kd) : 
    m_Kp(Kp),
    m_Ki(Ki),
    m_Kd(Kd),
    m_sum(0),
    m_prev(0)
{

}

double PIDController::run(double set_point, double pv, double delta_t)
{
    double err = set_point - pv;

    m_sum += err;

    double x_p = m_Kp * err + m_Ki * (m_sum) * delta_t + m_Kd * (err - m_prev) / delta_t;

    m_prev = err;

    return x_p;
}

void PIDController::update_pid_settings(std::vector<double> pidParams)
{
    m_Kp = pidParams[0];
    m_Ki = pidParams[1];
    m_Kd = pidParams[2];
}

std::vector<double> PIDController::get_params()
{
    return {m_Kp, m_Ki, m_Kd};
}
