git clone https://github.com.cnpmjs.org/zy-cuhk/paintingrobotdemo_v2.git


the z limit should be -0.20 instead of -0.17, 3cm must be given 
self.parameterz=rospy.get_param('mat_parameterz')#0.028625
<param name="min_holding_distance" value="-0.15" />
<param name="max_holding_distance" value="0.03" />
<param name="min_climbing_distance" value="-0.17" />
<param name="max_climbing_distance" value="0.90" />
<param name="min_rotation_distance" value="-5.50" />
<param name="max_rotation_distance" value="2.36" />

the modification part includes:
1. z limit should be changed and tested 
2. planing dict about the manipulator planing part should be modified 
(1) the planning result should be tested 
(2) the painting gun control part should be added 
