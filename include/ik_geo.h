
#include <vector>
#include <array>


namespace lib {

    extern "C" {
        struct Robot;
        void deallocate(Robot *robot);

        Robot *gen_six_dof(const double *h, const double *p);

        void ik(const Robot *robot,
                const double *rotation,
                const double *translation,
                size_t *n_solutions,
                double *qs,
                bool *is_ls);

        Robot *irb6640();

        Robot *spherical(const double *h, const double *p);

        Robot *spherical_bot();

        Robot *spherical_two_intersecting(const double *h, const double *p);

        Robot *spherical_two_parallel(const double *h, const double *p);

        Robot *three_parallel(const double *h, const double *p);

        Robot *three_parallel_bot();

        Robot *three_parallel_two_intersecting(const double *h, const double *p);

        Robot *two_intersecting(const double *h, const double *p);

        Robot *two_parallel(const double *h, const double *p);

        Robot *two_parallel_bot();

        Robot *ur5();

    }

}


namespace ik_geo {

    struct Solution {
        std::array<double, 6> q;
        bool is_ls;
    };

    class Robot {
        lib::Robot* robot;
        Robot(lib::Robot *robot) : robot(robot) {}
    public:
        ~Robot() {
            lib::deallocate(robot);
        }
        static Robot irb66400() {
            return Robot(lib::irb6640());
        }
        static Robot spherical(const double *h, const double *p) {
            return Robot(lib::spherical(h, p));
        }
        static Robot spherical_bot() {
            return Robot(lib::spherical_bot());
        }
        static Robot spherical_two_intersecting(const double *h, const double *p) {
            return Robot(lib::spherical_two_intersecting(h, p));
        }
        static Robot spherical_two_parallel(const double *h, const double *p) {
            return Robot(lib::spherical_two_parallel(h, p));
        }
        static Robot three_parallel(const double *h, const double *p) {
            return Robot(lib::three_parallel(h, p));
        }
        static Robot three_parallel_bot() {
            return Robot(lib::three_parallel_bot());
        }
        static Robot three_parallel_two_intersecting(const double *h, const double *p) {
            return Robot(lib::three_parallel_two_intersecting(h, p));
        }
        static Robot two_intersecting(const double *h, const double *p) {
            return Robot(lib::two_intersecting(h, p));
        }
        static Robot ur5() {
            return Robot(lib::ur5());
        }

        void ik(const double *rotation, const double *translation, std::vector<Solution> &solutions) {
            const size_t MAX_SOLUTIONS = 16;
            double qs[6 * MAX_SOLUTIONS];
            bool is_ls[MAX_SOLUTIONS];
            size_t n_solutions;
            lib::ik(robot, rotation, translation, &n_solutions, qs, is_ls);

            solutions.clear();
            for (size_t i = 0; i < n_solutions; i++) {
                Solution s;
                for (size_t j = 0; j < 6; j++) {
                    s.q[j] = qs[i * 6 + j];
                }
                s.is_ls = is_ls[i];
                solutions.push_back(s);
            }
        }
    };
}
