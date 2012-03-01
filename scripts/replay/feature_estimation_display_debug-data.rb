require 'orocos'
require 'Qt4'
require 'qwt'
require 'vizkit'
include Orocos
Orocos.initialize

if ARGV[0] == nil
	puts "Please add a path to a logfile, containing sonarscans, as argument!"
	exit
end


plot = Qwt::Plot.new
plot.setAxisScale(0,-100,200)
plot.setAxisScale(2,0,600)
window_size = plot.minimumSize
window_size.height = 600
window_size.width = 800
plot.minimumSize = window_size
plot.show

curve = Qwt::PlotCurve.new
curve.attach(plot)
curve.show

curve2 = Qwt::PlotCurve.new
pen = Qt::Pen.new
color = pen.color
color.red = 255
pen.color = color
curve2.pen = pen
curve2.attach(plot)
curve2.show

curve3 = Qwt::PlotCurve.new
pen = Qt::Pen.new
color = pen.color
color.red = 255
color.green = 255
pen.color = color
curve3.pen = pen
curve3.attach(plot)
curve3.show

curve4 = Qwt::PlotCurve.new
pen = Qt::Pen.new
color = pen.color
color.green = 255
color.blue = 255
pen.color = color
curve4.pen = pen
curve4.attach(plot)
curve4.show

curve5 = Qwt::PlotCurve.new
pen = Qt::Pen.new
color = pen.color
color.red = 255
color.blue = 255
pen.color = color
curve5.pen = pen
curve5.attach(plot)
curve5.show

marker_f1 = Qwt::PlotMarker.new
marker_f1.lineStyle = 2
marker_f1.attach(plot)
marker_f1.show

marker_ground = Qwt::PlotMarker.new
marker_ground.lineStyle = 2
pen = Qt::Pen.new
color = pen.color
color.green = 128
pen.color = color
marker_ground.linePen=pen
marker_ground.attach(plot)
marker_ground.show

marker_surface = Qwt::PlotMarker.new
marker_surface.lineStyle = 2
pen = Qt::Pen.new
color = pen.color
color.blue = 255
pen.color = color
marker_surface.linePen=pen
marker_surface.attach(plot)
marker_surface.show


view3d = Vizkit.default_loader.create_widget('vizkit::Vizkit3DWidget')
view3d.show
sonarbeamviz = view3d.createPlugin("sonarbeam","SonarBeamVisualization")    

log = Orocos::Log::Replay.open(ARGV)
sonar_port = log.find_port("/base/samples/SonarBeam")

Orocos.run 'sonar_feature_estimator' do

    feature_estimator = Orocos::TaskContext.get 'sonar_feature_estimator'
    feature_estimator.enable_debug_output = true
    
    sonar_port.connect_to feature_estimator.sonar_input
    #log.sonar.SonarScan.connect_to feature_estimator.sonar_input
    log.orientation_estimator.orientation_samples.connect_to feature_estimator.orientation_sample
    #log.pose_estimator.pose_samples.connect_to feature_estimator.orientation_sample


    con = Vizkit.connect_port_to 'sonar_feature_estimator', 'debug_output', :pull => false, :update_frequency => 33 do |sample, name|
        a = Array.new(sample.filteredBeam.length)
        b = Array.new(sample.filteredBeam.length)
        for i in 0..sample.filteredBeam.length-1 do
            a[i] = i
            b[i] = sample.filteredBeam[i]
        end
        curve2.setData(a,b)
        
        c = Array.new(sample.device_noise_distribution.length)
        for i in 0..sample.device_noise_distribution.length-1 do
            c[i] = sample.device_noise_distribution[i]
        end
        curve3.setData(a,c)
        
        d = Array.new(sample.gaussian_distribution.length)
        for i in 0..sample.gaussian_distribution.length-1 do
            d[i] = sample.gaussian_distribution[i]
        end
        curve4.setData(a,d)
        
        e = Array.new(sample.derivative.length)
        for i in 0..sample.derivative.length-1 do
            e[i] = sample.derivative[i]
        end
        curve5.setData(a,e)
        
        if (sample.bestPos >= 0) then
            marker_f1.setValue(sample.bestPos,sample.filteredBeam[sample.bestPos])
        else
            marker_f1.setValue(-1,0)
        end
        if (sample.pos_surface >= 0) then
            marker_surface.setValue(sample.pos_surface,1)
        else
            marker_surface.setValue(-1,0)
        end
        if (sample.pos_ground >= 0) then
            marker_ground.setValue(sample.pos_ground,1)
        else
            marker_ground.setValue(-1,0)
        end
        plot.replot
    end 

    sonar_port.connect_to do |sample, null|
        plot.setAxisScale(2,0,sample.beam.length)
        a = Array.new(sample.beam.length)
        b = Array.new(sample.beam.length)
        for i in 0..sample.beam.length-1 do
            a[i] = i
            b[i] = sample.beam[i]
        end
        curve.setData(a,b)
        sonarbeamviz.updateSonarBeam(sample)
        sample
    end 

    feature_estimator.start

    Vizkit.control log
    Vizkit.exec 
end
