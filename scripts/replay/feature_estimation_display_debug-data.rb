require 'orocos'
require 'Qt4'
require 'vizkit'
include Orocos
Orocos.initialize

if ARGV[0] == nil
	puts "Please add a path to a logfile, containing sonarscans, as argument!"
	exit
end

plotter = Vizkit.default_loader.Plot2d
#plotter.setZoomAble(true)
#plotter.setRangeAble(true)
plotter.show

marker_height = 200

log = Orocos::Log::Replay.open(ARGV)
sonar_port = log.find_port("/base/samples/SonarBeam")

Orocos.run 'sonar_feature_estimator_test' do

    feature_estimator = Orocos::TaskContext.get 'sonar_feature_estimator'
    feature_estimator.enable_debug_output = true
    feature_estimator.derivative_history_length = 3
    
    sonar_port.connect_to feature_estimator.sonar_input, :type => :buffer, :size => 100
    #log.sonar.SonarScan.connect_to feature_estimator.sonar_input
    log.orientation_estimator.orientation_samples.connect_to feature_estimator.orientation_sample
    #log.pose_estimator.pose_samples.connect_to feature_estimator.orientation_sample

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

    sonar_port.connect_to do |sample, null|
        plotter.update_sonar_beam sample,"sonarBeam"
        sample
    end 

    feature_estimator.start

    Vizkit.control log
    Vizkit.exec 
end
