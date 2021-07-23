//
// Created by redwan on 7/21/21.
//
#include <vector>
#include <Eigen/Dense>
#include <boost/python.hpp>
#include "robot_localization/ekf_interface.h"
using namespace std;
using namespace boost::python;
using namespace RobotLocalization;



class PoseEKF
{
public:

    PoseEKF(double processNoise, double measurementNoise, double mahalanobisThresh) {
        ekf_ = new ekf_interface(processNoise, measurementNoise, mahalanobisThresh);
    } // added constructor
    ~PoseEKF()
    {
        delete ekf_;
    }
    void set(const boost::python::list& ns) {
        Eigen::VectorXd Z(len(ns));
        for (int i = 0; i < len(ns); ++i)
        {
            Z(i) = boost::python::extract<double>(ns[i]);
        }

        ekf_->measurement_update(Z);
    }
    boost::python::list get(double ref, double dt){
        auto state = ekf_->estimate_state(ref, dt);
//        vector<double> vec(state.data(), state.data() + state.rows() * state.cols());
        boost::python::list result;
        for (int i = 0; i < state.size(); ++i) {
            result.append(state(i));
        }
        return result;
    }
    long get_ms() { return ekf_->get_ms(); }

private:
    ekf_interface * ekf_;

};


BOOST_PYTHON_MODULE(python_pose_ekf)
{
    Py_Initialize();
    class_<PoseEKF>("python_pose_ekf", init<double, double, double>())
            .def(init<double, double, double>())
            .def("get_ms", &PoseEKF::get_ms)
            .def("set", &PoseEKF::set)
            .def("get", &PoseEKF::get)
            ;
}

