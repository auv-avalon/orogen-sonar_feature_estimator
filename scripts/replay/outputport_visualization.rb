require 'orocos'
require 'vizkit'
include Orocos
Orocos.initialize

if ARGV[0] == nil
	puts "Please add a path to a logfile, containing sonarscans, as argument!"
	exit
end

log = Orocos::Log::Replay.open(ARGV)

Orocos.run 'sonar_feature_estimator' do

    feature_estimator = Orocos::TaskContext.get 'sonar_feature_estimator'

    log.sonar.BaseScan.connect_to feature_estimator.sonar_input
    log.orientation_estimator.orientation_samples.connect_to feature_estimator.orientation_sample

    view3d = Vizkit.default_loader.create_widget('vizkit::Vizkit3DWidget')
    view3d.show()
    sonarfeatureviz = view3d.createPlugin('sonarfeature', 'SonarFeatureVisualization')
    auv_avalon = view3d.createPlugin('auv_avalon', 'AUVAvalonVisualization')
    auv_avalon.showDesiredModelPosition(false)

    # Connect debug port to vizkit plugins
    con = Vizkit.connect_port_to 'sonar_feature_estimator', 'features', :pull => false, :update_frequency => 33 do |sample, _|
        sonarfeatureviz.updatePointCloud(sample)
    end 

    log.orientation_estimator.orientation_samples :type => :buffer, :size => 100  do |sample|
        auv_avalon.updateRigidBodyState(sample)
        sample
    end 

    feature_estimator.start

    Vizkit.control log
    Vizkit.exec
end


