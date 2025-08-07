/*
ConstraintDescriptor:
每个描述符解决一方面的问题，描述符的定义可以是一个约束，也可以是一个代价函数。

这里设计的描述符是为了描述来自环境、的约束，主要用于约束信息的表达，不涉及代价构建。

约束可能来自于各个方面，主要包括：
1. 交通环境约束：
  - end of current lane path
  - end of path boundary
  - crosswalk descriptor
  - solid line within boundary
  - speed bump
  - standstill
  - traffic light
  - turn signal

2. 交通参与者约束:
  - leading object
  - leading groups
  - pedestrians descriptor
*/
