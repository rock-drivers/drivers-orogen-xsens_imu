require 'orocos'
include Orocos

if !ARGV[0]
    STDERR.puts "usage: test.rb <device name>"
    exit 1
end

ENV['PKG_CONFIG_PATH'] = "#{File.expand_path("..", File.dirname(__FILE__))}/build:#{ENV['PKG_CONFIG_PATH']}"

Orocos.initialize

Orocos::Process.spawn 'test_imu' do |p|
    driver = p.task 'Driver'
    Orocos.log_all_ports

    driver.port = ARGV[0]
    driver.configure
    driver.start

    reader = driver.orientation_readings.reader(:type => :buffer, :size => 10)
    loop do
	if sample = reader.read
	    print("#{sample.value.im[0]} #{sample.value.im[1]} #{sample.value.im[2]} #{sample.value.re}\r\n")
	end
	sleep 0.01
    end
end

