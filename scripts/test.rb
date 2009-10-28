require 'orocos'
include Orocos

if !ARGV[0]
    STDERR.puts "usage: test.rb <device name>"
    exit 1
end

ENV['PKG_CONFIG_PATH'] = "#{File.expand_path("..", File.dirname(__FILE__))}/build:#{ENV['PKG_CONFIG_PATH']}"

Orocos::Process.spawn 'imu' do |p|
    driver = p.task 'imu'
    Orocos.log_all_ports

    driver.port = ARGV[0]
    driver.configure
    driver.start

    reader = driver.orientation_readings.reader(:type => :buffer, :size => 5)
    Orocos.guard do
        loop do
            if sample = reader.read
            pp sample.stamp
            end
        end
    end
end

