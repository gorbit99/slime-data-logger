require 'socket'

ACCEL_SENSITIVITY = 16384.0
GYRO_SENSITIVITY = 131.072

G_TO_MS = 9.80665
DPS_TO_RAD = Math::PI / 180

def parse_imu_packet(data)
  accel = nil
  if data[1] != -32768
    accel = [
      data[1] << 4 | ((data[9] & 0xf0) >> 4),
      data[2] << 4 | ((data[10] & 0xf0) >> 4),
      data[3] << 4 | ((data[11] & 0xf0) >> 4),
    ]
    accel.map!{|value| value / ACCEL_SENSITIVITY * G_TO_MS}
  end
  gyro = [
    data[4] << 4 | (data[9] & 0x0f),
    data[5] << 4 | (data[10] & 0x0f),
    data[6] << 4 | (data[11] & 0x0f),
  ]
  gyro.map!{|value| value / GYRO_SENSITIVITY * DPS_TO_RAD}

  return [gyro, accel]
end

def handle_client(client)
  puts "Received connection"
  fileName = "log_#{Time.now.strftime('%Y-%m-%dT%H:%M:%S.%L%z')}.txt"
  logFile = File.open(fileName, 'w');
  begin
    loop do
      data = client.recv(22)
      parsed = data.unpack("CCCssssssssCCC")
      if parsed[0] == 0x01
        client.write(data)
        next
      end

      if parsed[0] == 0x03 # Error
        if parsed[1] == 0x00
          puts "FIFO Overflow!"
        elsif parsed[1] == 0x01
          puts "Dropped packets! (Packet handling too slow)"
        end
        next
      end

      gyro, accel = parse_imu_packet(parsed[2..])
      logFile.write("0;#{gyro.join(";")}\n")
      if !accel.nil?
        logFile.write("1;#{accel.join(";")}\n")
      end
    end

  rescue EOFError
    puts "Client disconnected"
  rescue => e
    puts "Error #{e.message}"
  ensure
    client.close()
  end
end

server = TCPServer.new(42069)
loop do
  Thread.start(server.accept) do |client|
    handle_client(client)
  end
end

