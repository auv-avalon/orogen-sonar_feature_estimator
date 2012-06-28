require 'orocos'
require 'Qt4'
require 'vizkit'
include Orocos

#Orocos::CORBA.name_service = "192.168.128.51"
Orocos::CORBA.name_service = "127.0.0.1"
Orocos.initialize

plotter = Vizkit.default_loader.Plot2d
#plotter.setZoomAble(true)
#plotter.setRangeAble(true)
plotter.show

marker_height = 200

feature_estimator = Orocos::TaskContext.get 'sonar_feature_estimator'
feature_estimator.enable_debug_output = true

con = Vizkit.connect_port_to 'sonar_feature_estimator', 'debug_output', :pull => false, :update_frequency => 33 do |sample, name|
    plotter.update_vector sample.filteredBeam,"filteredBeam"
    plotter.update_vector sample.device_noise_distribution,"device_noise_distribution"
    plotter.update_vector sample.gaussian_distribution,"gaussian_distribution"
    plotter.update_vector sample.derivative,"derivative"
    
    if (sample.bestPos >= 0) then
        bestPos = Array.new(sample.bestPos + 1)
        bestPos[sample.bestPos] = marker_height
        plotter.update_vector bestPos,"bestPos"
    else
        bestPos = Array.new(1)
        bestPos[0] = marker_height
        plotter.update_vector bestPos,"bestPos"
    end

    if (sample.pos_surface >= 0) then
        pos_surface = Array.new(sample.pos_surface + 1)
        pos_surface[sample.pos_surface] = marker_height
        plotter.update_vector pos_surface,"pos_surface"
    else
        pos_surface = Array.new(1)
        pos_surface[0] = marker_height
        plotter.update_vector pos_surface,"pos_surface"
    end

    if (sample.pos_ground >= 0) then
        pos_ground = Array.new(sample.pos_ground + 1)
        pos_ground[sample.pos_ground] = marker_height
        plotter.update_vector pos_ground,"pos_ground"
    else
        pos_ground = Array.new(1)
        pos_ground[0] = marker_height
        plotter.update_vector pos_ground,"pos_ground"
    end
end 

Vizkit.exec 
