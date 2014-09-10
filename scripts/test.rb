require 'orocos'
include Orocos

if !ARGV[0]
    STDERR.puts "usage: test.rb <device name>"
    exit 1
end

ENV['PKG_CONFIG_PATH'] = "#{File.expand_path("..", File.dirname(__FILE__))}/build:#{ENV['PKG_CONFIG_PATH']}"

Orocos.initialize

Orocos.run "xsens_imu::Task" => 'xsens_imu' do |p|
    driver = p.task 'xsens_imu'
    Orocos.log_all_ports

    driver.port = ARGV[0]
    driver.configure
    driver.start

    reader = driver.orientation_samples.reader(:type => :buffer, :size => 100)
    loop do        
	while sample = reader.read_new
          print("#{sample.time.to_f} #{sample.orientation.x} #{sample.orientation.y} #{sample.orientation.z} #{sample.orientation.w}\r\n")
	end
	sleep 0.01
    end
end

