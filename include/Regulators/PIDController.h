#include <vector>

namespace pid
{
    class PIDController
    {
        public:

            PIDController(double Kp, double Ki, double Kd);

            // function for calculating PID output
            double run(double set_point, double pv, double delta_t);

            // function for updating PID parameters
            void update_pid_settings(std::vector<double> pidParams);

            // function for returning current PID parameters
            std::vector<double> get_params();

            // sum of errors
            double m_sum;

            // previous error
            double m_prev;

        private:

            // Kp gain (for P part of the regulator)
            double m_Kp;

            // Ki (for I part of the regulator)
            double m_Ki;

            // Kd (for D part of the regulator)
            double m_Kd;
    };

} // namespace pid