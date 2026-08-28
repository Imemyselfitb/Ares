#include "Sensors.h"

#include <TinyGPS++.h>

#define GPS_RX 43 // Receiver
#define GPS_TX 44 // Transmitter
#define TELEM_RX 18 // Receiver
#define TELEM_TX 17 // Transmitter

static TinyGPSPlus s_GPS;
static TinyGPSCustom s_GPSFixQuality{ s_GPS, "GPGGA", 0x6 }; // Sixth field of the GPGGA nmea sentence

struct LaunchPadDataGPS
{
  double positionECEF[3];
  double sinLat, sinLon;
  double cosLat, cosLon;
};
static LaunchPadDataGPS s_LaunchPadDataGPS;

// Sequence of 4 bits that have a hamming distance of 2
// (ie. one bit can be currupted and the system would detect that an error has happened! :D)
static constexpr uint8_t COMMAND_NIBBLES[] = { 0x0, 0x3, 0x5, 0x6, 0x9, 0xA };
#define COMMAND_ID(idx) ((COMMAND_NIBBLES[(idx / 6) + 1] << 4) | COMMAND_NIBBLES[idx % 6]) // Range: 0 < idx < 30
enum class Commands : uint8_t
{
  Begin = 0xAA, // Begin Custom Command data
  Launch = COMMAND_ID(1),
  Abort = COMMAND_ID(2),
};

void Telemetry::Init()
{
  m_SerialTelem.setRxBufferSize(2048);
  m_SerialGPS.setRxBufferSize(2048);
  m_SerialTelem.begin(115200, SERIAL_8N1, TELEM_RX, TELEM_TX);
  m_SerialGPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
}

// TODO
void Telemetry::handleIncomingCommand(uint8_t commandByte)
{
  // TODO
}

void Telemetry::ProcessIncoming()
{
  while (m_SerialTelem.available() > 0)
  {
    uint8_t rtcmByte = m_SerialTelem.read();

    if (rtcmByte == (uint8_t)Commands::Begin)
      handleIncomingCommand(m_SerialTelem.read());
    else if (rtcmByte == 0xD3)
    {
      m_SerialGPS.write(rtcmByte);

      uint8_t packetLengthBytes[2] = { m_SerialTelem.read(), m_SerialTelem.read() };
      m_SerialGPS.write(packetLengthBytes[0]);
      m_SerialGPS.write(packetLengthBytes[1]);

      uint16_t packetLength = (((packetLengthBytes[0] & 0x03) << 8) | packetLengthBytes[1]) + 6; // Add overhead bytes
      for (uint16_t i = 3; i < packetLength; i++) // Start at index 3, skipping the first 3 bytes which have already been read and sent to the GPS.
        m_SerialGPS.write(m_SerialTelem.read());
    }
  }
}

// Note: Latitude and Longitude must be in radians
static void gpsToECEF(double lat, double lon, double alt, double outECEF[3])
{
  constexpr double earthRadius = 6378137.0;
  constexpr double eccentricitySq = 6.69437999014e-3;

  const double sinLat = sin(lat);
  const double cosLat = cos(lat);

  const double curvature = earthRadius / sqrt(1.0 - eccentricitySq * sinLat * sinLat);

  outECEF[0] = (curvature + alt) * cosLat * cos(lon);
  outECEF[1] = (curvature + alt) * cosLat * sin(lon);
  outECEF[2] = (curvature * (1.0 - eccentricitySq) + alt) * sinLat;
}

static Vector3 ecefToNUE(double ecef[3])
{
  const double dx = ecef[0] - s_LaunchPadDataGPS.positionECEF[0];
  const double dy = ecef[1] - s_LaunchPadDataGPS.positionECEF[1];
  const double dz = ecef[2] - s_LaunchPadDataGPS.positionECEF[2];

  const double east = -s_LaunchPadDataGPS.sinLon * dx + s_LaunchPadDataGPS.cosLon * dy;
  const double north = -s_LaunchPadDataGPS.sinLat * (s_LaunchPadDataGPS.cosLon * dx - s_LaunchPadDataGPS.sinLon * dy) + s_LaunchPadDataGPS.cosLat * dz;
  const double up = s_LaunchPadDataGPS.cosLat * (s_LaunchPadDataGPS.cosLon * dx + s_LaunchPadDataGPS.sinLon * dy) + s_LaunchPadDataGPS.sinLat * dz;

  return Vector3{
    (float)north,
    (float)up,
    (float)east
  };
}

void Telemetry::ProcessGPS(GPSData& outGPSData, Vector3& outGPSPosition)
{
  while (m_SerialGPS.available() > 0)
  {
    s_GPS.encode(m_SerialGPS.read());
  }

  if (s_GPSFixQuality.isUpdated())
    outGPSData.Fix = (GPSFixQuality)((char)s_GPSFixQuality.value()[0] - '0');

  bool posChanged = false;
  if (s_GPS.location.isUpdated())
  {
    outGPSData.Latitude = s_GPS.location.lat();
    outGPSData.Longitude = s_GPS.location.lng();
    posChanged = true;
  }

  if (s_GPS.altitude.isUpdated())
  {
    outGPSData.Altitude = s_GPS.altitude.meters();
    posChanged = true;
  }

  if (posChanged)
  {
    double ecefPosition[3];
    gpsToECEF(outGPSData.Latitude * PI/180.0, outGPSData.Longitude * PI/180.0, outGPSData.Altitude, ecefPosition);
    outGPSPosition = ecefToNUE(ecefPosition);
  }
}

void Telemetry::EmitPacket(TelemeteryData& packet)
{
  packet.PacketID++;
  m_SerialTelem.write((uint8_t*)&packet, sizeof(TelemeteryData));
}
