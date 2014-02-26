#! /usr/bin/env ruby

require 'vizkit'
require 'optparse'

hostname = "localhost"
only_positive=false
override_vel_limits=0.0
no_effort=false
no_velocity=false
@driver_port_name = nil
options = OptionParser.new do |opt|
    opt.on '--host=HOSTNAME', String, 'the host we should contact to find RTT tasks' do |host|
        hostname = host
    end
    opt.on '--help', 'this help message' do
        puts opt
        exit(0)
    end
    opt.on '--only_positive_vel', "Use only positive velocities for sending" do
        only_positive=true
    end
    opt.on '--override_vel_limits=VALUE', "Oerrride velocity limits with the value given (positive and negative limits are changed to this value)" do |val|
        override_vel_limits=Float(val)
    end
    opt.on '--no_effort', "Don't generate UI elements for effort" do
        no_effort=true
    end
    opt.on '--no_velocity', "Don't generate UI elements for velocity" do
        no_velocity=true
    end
    opt.on '--driver=DRIVER_TASK_NAME', String, "BLDC driver task name" do |name|
       @driver_task_name = name 
    end
end

if(!@driver_port_name)
    raise("No driver port given")
end

Orocos::CORBA.name_service.ip = hostname
Orocos.initialize
Orocos.load_typekit('base')

ctrl_gui = Vizkit.default_loader.ControlUi
ctrl_gui.configureUi( override_vel_limits, only_positive, no_effort, no_velocity )

driver_task = Orocos::Async.name_service.get @driver_task_name

joints_config = driver_task.joints_config
jointLimits = Base::JointLimits.new
joints_config.each do |joint_config|
    jointLimits.names.push_back( joint_config.jointName )
    jointLimitRange = Base::JointLimitRange.new
    jointLimitRange.min = joint_config.positionCtrl.inputLimit.lower
    jointLimitRange.max = joint_config.positionCtrl.inputLimit.upper
    jointLimits.elements.push_back( jointLimitRange )
end

ctrl_gui.initModel( jointLimits )

@initial_data = true
driver_task.port("joints_status").on_data do |data| 
    if @initial_data
        ctrl_gui.setJointState(data)
        @initial_data = false
    end
end

@writer = driver_task.port("joints_command").command_port.writer
ctrl_gui.connect(SIGNAL('sendSignal()')) do 
    sample = ctrl_gui.getJoints()
    @writer.write(sample)
end

Vizkit.exec    
