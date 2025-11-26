#include<stdio.h>

#pragma pack(push,1)

#define POINTNUM 30  // Number of points per packet

/**
 * @brief Structure representing a detected point
 */
typedef struct DETECTION
{
	short rangeIdx;       // Range index
	short dopplerIdx;     // Doppler index
	short aziIdx;         // Azimuth index
	short eleIdx;         // Elevation index
	float range;          // Range value (meters)
	float doppler;        // Doppler velocity (m/s)
	float azi;            // Azimuth angle (radians)
	float ele;            // Elevation angle (radians)
	float snr;            // Signal-to-noise ratio
	int rangeVar;         // Range variance
	float dopplerVar;     // Doppler variance
	float aziVar;         // Azimuth variance
	float eleVar;         // Elevation variance
}DETECTION;

/**
 * @brief Packet header structure
 */
typedef struct PCKHEADER
{
    unsigned int  header;          // Packet header identifier (0xabcd4321)
    unsigned char version;         // Protocol version
    unsigned char mode;            // Operation mode (long or short distance)
    unsigned char reserved[2];     // Reserved bytes
    unsigned int  sec;             // Timestamp seconds
    unsigned int  nsec;            // Timestamp nanoseconds
    unsigned int  frameId;         // Frame identifier
    unsigned int  objectCount;     // Total number of objects
    unsigned short curObjInd;      // Current object index
    unsigned short curObjNum;      // Current object number
}PCKHEADER;

/**
 * @brief Section header structure
 */
typedef struct SECHEADER
{
	unsigned int sectionType;     // Section type identifier
	unsigned int sectionLength;   // Section length in bytes
}SECHEADER;

/**
 * @brief System information structure
 */
typedef struct SYSINFO
{
	float rangeRes;                    // Range resolution
	float dopplerRes;                  // Doppler resolution
	unsigned short numRangeBins;       // Number of range bins
	unsigned short numDopplerBins;     // Number of Doppler bins
	unsigned char numSensors;          // Number of sensors
	unsigned char numTxAnt;            // Number of TX antennas
	unsigned char numRxAnt;            // Number of RX antennas
	unsigned char numTxAzimuthAnt;     // Number of TX azimuth antennas
	unsigned char numTxElevationAnt;   // Number of TX elevation antennas
	unsigned char padding[3U];         // Padding bytes
} SYSINFO;

/**
 * @brief Point cloud data structure for one packet
 */
typedef struct POINTCLOUD
{
	PCKHEADER pckHeader;        // Packet header
	SYSINFO sysInfo;            // System information
	DETECTION point[POINTNUM];  // Array of detected points
}POINTCLOUD;

/**
 * @brief Simplified point cloud XYZ structure
 */
typedef struct POINTCLOUDXYZ
{
	float x;  // X coordinate
	float y;  // Y coordinate
	float z;  // Z coordinate
	float h;  // Height or Doppler velocity
	float s;  // Signal strength or RCS
	float v;  // Velocity classification (-1: away, 0: stationary, 1: approaching)
}POINTCLOUDXYZ;

#pragma pack(pop)