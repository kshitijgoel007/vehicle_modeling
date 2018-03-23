function [yaml_struct] = yamlReadFunc(file)

yamlJohn = yamlread('johnHex.yaml');

allKeys = yamlJohn.keys;
allVals = yamlJohn.values;

%% Mass
strfind(allKeys, 'mass');
temp_index = find(~cellfun(@isempty,ans));
yaml_struct.mass_value = allVals{temp_index};

%% Inertia Initial
strfind(allKeys, 'inertia/Ixx');
temp_index = find(~cellfun(@isempty,ans));
yaml_struct.Ixx_value = allVals{temp_index};
strfind(allKeys, 'inertia/Iyy');
temp_index = find(~cellfun(@isempty,ans));
yaml_struct.Iyy_value = allVals{temp_index};
strfind(allKeys, 'inertia/Izz');
temp_index = find(~cellfun(@isempty,ans));
yaml_struct.Izz_value = allVals{temp_index};

strfind(allKeys, 'inertia/Ixy');
temp_index = find(~cellfun(@isempty,ans));
yaml_struct.Ixy_value = allVals{temp_index};
strfind(allKeys, 'inertia/Ixz');
temp_index = find(~cellfun(@isempty,ans));
yaml_struct.Ixz_value = allVals{temp_index};
strfind(allKeys, 'inertia/Iyz');
temp_index = find(~cellfun(@isempty,ans));
yaml_struct.Iyz_value = allVals{temp_index};

%% Yaw Offset and Z offset
strfind(allKeys, 'yaw_offset');
temp_index = find(~cellfun(@isempty,ans));
yaml_struct.yaw_offset = allVals{temp_index};

strfind(allKeys, 'zoffset');
temp_index = find(~cellfun(@isempty,ans));
yaml_struct.z_offset = allVals{temp_index};

%% Number of rotors
strfind(allKeys, 'num_rotors');
temp_index = find(~cellfun(@isempty,ans));
yaml_struct.num_rotors_value = allVals{temp_index};


%% Arm Length - Front, Middle, Back
strfind(allKeys, 'arm/length/front');
temp_index = find(~cellfun(@isempty,ans));
yaml_struct.arm_length_front = allVals{temp_index};
strfind(allKeys, 'arm/length/middle');
temp_index = find(~cellfun(@isempty,ans));
yaml_struct.arm_length_middle = allVals{temp_index};
strfind(allKeys, 'arm/length/back');
temp_index = find(~cellfun(@isempty,ans));
yaml_struct.arm_length_back = allVals{temp_index};

%% Arm Angle - Front, Middle, Back
strfind(allKeys, 'arm/angle/front');
temp_index = find(~cellfun(@isempty,ans));
yaml_struct.arm_angle_front = allVals{temp_index};
strfind(allKeys, 'arm/angle/middle');
temp_index = find(~cellfun(@isempty,ans));
yaml_struct.arm_angle_middle = allVals{temp_index};
strfind(allKeys, 'arm/angle/back');
temp_index = find(~cellfun(@isempty,ans));
yaml_struct.arm_angle_back = allVals{temp_index};

%% Gains Motor
strfind(allKeys, 'gains/motor');
temp_index = find(~cellfun(@isempty,ans));
yaml_struct.gains_motor_value = allVals{temp_index};

%% Gains Thrust
strfind(allKeys, 'gains/thrust');
temp_index = find(~cellfun(@isempty,ans));
yaml_struct.gains_thrust = allVals{temp_index};

%% Gains Moment_Scale
strfind(allKeys, 'gains/moment_scale');
temp_index = find(~cellfun(@isempty,ans));
yaml_struct.gains_moment_scale = allVals{temp_index};

%% cT0, cT1, cT2
strfind(allKeys, 'gains/rotor/cT0');
temp_index = find(~cellfun(@isempty,ans));
yaml_struct.cT0_value = allVals{temp_index};
strfind(allKeys, 'gains/rotor/cT1');
temp_index = find(~cellfun(@isempty,ans));
yaml_struct.cT1_value = allVals{temp_index};
strfind(allKeys, 'gains/rotor/cT2');
temp_index = find(~cellfun(@isempty,ans));
yaml_struct.cT2_value = allVals{temp_index};

%% PWM disarmed, PWM Low, PWM High
strfind(allKeys, 'pwm/disarmed');
temp_index = find(~cellfun(@isempty,ans));
yaml_struct.pwm_disarmed = allVals{temp_index};
strfind(allKeys, 'pwm/low');
temp_index = find(~cellfun(@isempty,ans));
yaml_struct.pwm_low = allVals{temp_index};
strfind(allKeys, 'pwm/high');
temp_index = find(~cellfun(@isempty,ans));
yaml_struct.pwm_high = allVals{temp_index};

%% RPM min, RPM idle, RPM max
strfind(allKeys, 'rpm/min');
temp_index = find(~cellfun(@isempty,ans));
yaml_struct.rpm_min = allVals{temp_index};
strfind(allKeys, 'rpm/idle');
temp_index = find(~cellfun(@isempty,ans));
yaml_struct.rpm_idle = allVals{temp_index};
strfind(allKeys, 'rpm/max');
temp_index = find(~cellfun(@isempty,ans));
yaml_struct.rpm_max = allVals{temp_index};

%% Battery Voltage Max
strfind(allKeys, 'battery/voltage_max');
temp_index = find(~cellfun(@isempty,ans));
yaml_struct.battery_voltage_max = allVals{temp_index};

%% Battery V0, Battery V1, Battery V2, Battery V3
strfind(allKeys, 'battery/voltage/V0');
temp_index = find(~cellfun(@isempty,ans));
yaml_struct.battery_voltage_V0 = allVals{temp_index};
strfind(allKeys, 'battery/voltage/V1');
temp_index = find(~cellfun(@isempty,ans));
yaml_struct.battery_voltage_V1 = allVals{temp_index};
strfind(allKeys, 'battery/voltage/V2');
temp_index = find(~cellfun(@isempty,ans));
yaml_struct.battery_voltage_V2 = allVals{temp_index};
strfind(allKeys, 'battery/voltage/V3');
temp_index = find(~cellfun(@isempty,ans));
yaml_struct.battery_voltage_V3 = allVals{temp_index};

%% Battery Time t1, t2, t3
strfind(allKeys, 'battery/time/t1');
temp_index = find(~cellfun(@isempty,ans));
yaml_struct.battery_time_t1 = allVals{temp_index};
strfind(allKeys, 'battery/time/t2');
temp_index = find(~cellfun(@isempty,ans));
yaml_struct.battery_time_t2 = allVals{temp_index};
strfind(allKeys, 'battery/time/t3');
temp_index = find(~cellfun(@isempty,ans));
yaml_struct.battery_time_t3 = allVals{temp_index};

%% dragModel cD, dragModel effective_tippathplane_z
strfind(allKeys, 'drag_model/cD');
temp_index = find(~cellfun(@isempty,ans));
yaml_struct.cD = allVals{temp_index};

strfind(allKeys, 'drag_model/effective_tippathplane_z');
temp_index = find(~cellfun(@isempty,ans));
yaml_struct.effective_tippathplane_z = allVals{temp_index};
end