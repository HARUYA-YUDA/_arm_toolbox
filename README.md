# _arm_toolbox
##Most Important 
---
__It have error of Eigen::Matrixxd__

when it start calcJacovian();
include/mikata_arm_toolbox/arm.hpp:199

It was just copied from kinematics.cpp
src/kinematics.cpp:83

I might have to learn how to calculate jacobian

##Error Message
---
```
uck_ik: /usr/include/eigen3/Eigen/src/Core/DenseCoeffsBase.h:413: Eigen::DenseCoeffsBase<Derived, 1>::Scalar& Eigen::DenseCoeffsBase<Derived, 1>::operator()(Eigen::Index) [with Derived = Eigen::Matrix<double, 3, 1>; Eigen::DenseCoeffsBase<Derived, 1>::Scalar = double; Eigen::Index = long int]: Assertion `index >= 0 && index < size()' failed.
```
