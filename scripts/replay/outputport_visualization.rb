require 'orocos'
require 'vizkit'
require 'Qt4'
include Orocos
Orocos.initialize

if ARGV[0] == nil
	puts "Please add a path to a logfile, containing sonarscans, as argument!"
	exit
end

log = Orocos::Log::Replay.open(ARGV)

Orocos.run 'sonar_feature_estimator_test' do

    feature_estimator = Orocos::TaskContext.get 'sonar_feature_estimator'
    feature_estimator.derivative_history_length = 3
    feature_estimator.enforce_line_rate = 0.5
    feature_estimator.feature_threshold = 0.6
    feature_estimator.enable_debug_output = true

    log.sonar.BaseScan.connect_to feature_estimator.sonar_input, :type => :buffer, :size => 100
    log.orientation_estimator.orientation_samples.connect_to feature_estimator.orientation_sample

    view3d = Vizkit.default_loader.create_widget('vizkit::Vizkit3DWidget')
    view3d.show()
    sonarfeatureviz = view3d.createPlugin('sonarfeature', 'SonarFeatureVisualization')
    sonarfeaturediffviz = view3d.createPlugin('sonarfeature', 'SonarFeatureVisualization')
    color = Qt::Color.new
    color.red = 255.0
    sonarfeaturediffviz.setDefaultFeatureColor(color)
    auv_avalon = view3d.createPlugin('auv_avalon', 'AUVAvalonVisualization')
    #wallviz = view3d.createPlugin('wall', 'WallVisualization')
    auv_avalon.showDesiredModelPosition(false)

    # Connect debug port to vizkit plugins
    con = Vizkit.connect_port_to 'sonar_feature_estimator', '2d_debug_output', :pull => false, :update_frequency => 33 do |sample, _|
        sonarfeatureviz.updatePointCloud(sample.point_cloud)
        #sonarfeatureviz.updatePointCloud(sample.feature_candidates)
        sonarfeaturediffviz.updatePointCloud(sample.point_cloud_force_line)
        #wallviz.updateWallData(sample.force_line_pos)
    end 

    log.orientation_estimator.orientation_samples :type => :buffer, :size => 100  do |sample|
        auv_avalon.updateRigidBodyState(sample)
        sample
    end 

    feature_estimator.start

    Vizkit.display feature_estimator
    Vizkit.control log
    Vizkit.exec
end


