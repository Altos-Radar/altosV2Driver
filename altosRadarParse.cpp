#include "pointCloud.h"
#include <algorithm>
#include <arpa/inet.h>
#include <errno.h>
#include <iostream>
#include <math.h>
#include <pcl/point_cloud.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <vector>

using namespace std;

// Constants for velocity histogram calculation
#define widthSet 8000           // Maximum number of points to process
#define vrMax 60                // Maximum radial velocity (m/s)
#define vrMin -60               // Minimum radial velocity (m/s)
#define vStep 0.5               // Velocity step size for histogram
#define errThr 3                // Threshold for velocity classification
#define PI 3.1415926            // Pi constant

// Network configuration
#define GROUPIP "224.1.2.4"     // Multicast group IP address
#define GROUPPORT 4040          // Multicast port
#define LOCALIP "192.168.3.1"   // Local IP address for binding
#define UNIPORT 4041            // Unicast port
#define UNIFLAG 0               // Flag to switch between multicast(0)/unicast(1)

/**
 * @brief Calculate RCS (Radar Cross Section) value from radar measurements
 * 
 * This function computes the Radar Cross Section using the range,
 * azimuth angle, and signal-to-noise ratio, normalized by calibration data.
 * 
 * @param range Distance to target in meters
 * @param azi Azimuth angle in radians
 * @param snr Signal-to-noise ratio
 * @param rcsBuf Pointer to RCS calibration lookup table
 * @return Calculated RCS value
 */
float rcsCal(float range, float azi, float snr, float* rcsBuf) {
    // Calculate index in RCS buffer based on azimuth angle
    int ind = (azi * 180 / PI + 60.1) * 10;
    
    // Compute RCS using power law model and calibration data
    float rcs = powf32(range, 2.6) * snr / 5.0e6 / rcsBuf[ind];

    return rcs;
}

/**
 * @brief Create and configure UDP socket for receiving radar data packets
 * 
 * This function creates a UDP socket with appropriate timeouts and binds it
 * to either a multicast group or unicast address based on the UNIFLAG setting.
 * 
 * @return Socket file descriptor on success, 0 on failure
 */
int socketGen()
{
    struct sockaddr_in addr;           // Socket address structure
    struct ip_mreq req;                // IP multicast request structure
    
    // Create UDP socket
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (-1 == sockfd) {
        perror("socket");
        return 0;
    }
    
    // Set send and receive timeouts (1.3 seconds)
    struct timeval timeout = {1, 300};
    setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, (char*)&timeout,
               sizeof(struct timeval));
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout,
               sizeof(struct timeval));
    
    // Initialize address structure
    memset(&addr, 0, sizeof(addr));
    
    // Configure socket based on unicast/multicast mode
    if(UNIFLAG)
    {
        // Unicast configuration
        addr.sin_family = AF_INET;
        addr.sin_port = htons(UNIPORT);                    // Set unicast port
        addr.sin_addr.s_addr = inet_addr(LOCALIP);        // Bind to local IP
        int ret = bind(sockfd, (struct sockaddr*)&addr, sizeof(addr));
        if (-1 == ret) {
            perror("bind");
            return 0;
        }
    }
    else
    {
        // Multicast configuration
        addr.sin_family = AF_INET;
        addr.sin_port = htons(GROUPPORT);                  // Set multicast port
        addr.sin_addr.s_addr = INADDR_ANY;                 // Accept from any interface
        
        // Bind socket to multicast address
        int ret = bind(sockfd, (struct sockaddr*)&addr, sizeof(addr));
        if (-1 == ret) {
            perror("bind");
            return 0;
        }

        // Join multicast group
        req.imr_multiaddr.s_addr = inet_addr(GROUPIP);    // Multicast group address
        req.imr_interface.s_addr = inet_addr(LOCALIP);    // Local interface address
        ret = setsockopt(sockfd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &req, sizeof(req));
        if (ret < 0) {
            perror("setsockopt");
            return 0;
        }
    }

    return sockfd;
}

/**
 * @brief Calculate velocity histogram for motion estimation
 * 
 * This function creates a histogram of radial velocities from Doppler measurements
 * and returns the most likely velocity based on the histogram peak.
 * 
 * @param pointCloudVec Vector containing point cloud data
 * @param histBuf Buffer to store histogram values
 * @param step Velocity step size for histogram bins
 * @return Estimated velocity corresponding to histogram peak
 */
float hist(vector<POINTCLOUD> pointCloudVec, float* histBuf, float step) {
    int ind = 0;                        // Histogram bin index
    float vr = 0;                       // Radial velocity

    // Process all points in the point cloud vector
    for (int i = 0; i < pointCloudVec.size(); i++) {
        for (int j = 0; j < 30; j++) {
            // Only process valid points (non-zero range)
            if (abs(pointCloudVec[i].point[j].range) > 0) {
                // Calculate radial velocity from Doppler measurement
                // Dividing by cos(azi) converts radial velocity to actual velocity
                vr = pointCloudVec[i].point[j].doppler / cos(pointCloudVec[i].point[j].azi);
                
                // Calculate histogram bin index
                ind = (vr - vrMin) / step;
                
                // Filter out invalid or out-of-range velocities
                if (vr > 60 || vr < -60 || isnan(vr)) {
                    continue;
                }
                
                // Only count negative or zero velocities (approaching targets)
                if (vr <= 0) {
                    histBuf[ind]++;
                }
            }
        }
    }
    
    // Find the histogram bin with maximum count and convert back to velocity
    return float(
               (max_element(histBuf, histBuf + (int((vrMax - vrMin) / step))) -
                histBuf)) *
               step +
           vrMin;
}

/**
 * @brief Process point cloud data and calculate Cartesian coordinates and velocities
 * 
 * This function converts spherical coordinates (range, azimuth, elevation) to
 * Cartesian coordinates (x, y, z), calculates RCS values, and classifies points
 * based on their relative velocity.
 * 
 * @param pointCloudVec Vector of input point clouds to process
 * @param cloud Output array for processed points
 * @param installFlag Installation orientation flag (-1 or 1)
 * @param rcsBuf RCS calibration lookup table
 * @param step Velocity step size for histogram
 * @param histBuf Buffer for velocity histogram calculation
 * @param pointNumPerPack Number of points in each packet
 */
void calPoint(vector<POINTCLOUD> pointCloudVec, POINTCLOUDXYZ *cloud, int installFlag, 
              float *rcsBuf, float step, float *histBuf, int pointNumPerPack)
{
    POINTCLOUDXYZ cloudPoint;           // Temporary point structure
    int pointIndex = 0;                 // Index for output array
    
    // Convert spherical coordinates to Cartesian coordinates for all points
    for(int i = 0; i < pointCloudVec.size(); i++)
    {
        for(int j = 0; j < pointNumPerPack; j++)
        {
            // Process only valid points (non-zero range)
            if(abs(pointCloudVec[i].point[j].range) > 0)
            {
                // Apply installation orientation correction to elevation angle
                pointCloudVec[i].point[j].ele = installFlag * (pointCloudVec[i].point[j].ele);

                // Convert from spherical to Cartesian coordinates:
                // x = range * cos(azimuth) * cos(elevation)
                // y = range * sin(azimuth) * cos(elevation) 
                // z = range * sin(elevation)
                cloudPoint.x = (pointCloudVec[i].point[j].range) * 
                               cos(pointCloudVec[i].point[j].azi) * 
                               cos(pointCloudVec[i].point[j].ele); 
                               
                cloudPoint.y = (pointCloudVec[i].point[j].range) * 
                               sin(pointCloudVec[i].point[j].azi) * 
                               cos(pointCloudVec[i].point[j].ele);
                               
                cloudPoint.z = (pointCloudVec[i].point[j].range) * 
                               sin(pointCloudVec[i].point[j].ele);
                               
                // Store Doppler velocity
                cloudPoint.h = pointCloudVec[i].point[j].doppler; 
                
                // Calculate and store RCS value
                cloudPoint.s = rcsCal(pointCloudVec[i].point[j].range, 
                                      pointCloudVec[i].point[j].azi,
                                      pointCloudVec[i].point[j].snr, 
                                      rcsBuf);
                                      
                // Add point to output array
                cloud[pointIndex++] = cloudPoint;
            }
        }
    }
    
    // Reset histogram buffer for velocity estimation
    memset(histBuf, 0, sizeof(float) * int((vrMax - vrMin) / step));
    
    // Estimate dominant velocity from histogram
    float vrEst = hist(pointCloudVec, histBuf, step);
    float tmp;  // Temporary variable for velocity comparison
    
    // Classify each point based on its relative velocity
    for (int i = 0; i < pointCloudVec.size(); i++) {
        for (int j = 0; j < pointNumPerPack; j++) {
            if (abs(pointCloudVec[i].point[j].range) > 0) {
                // Calculate relative velocity (measured velocity - estimated background velocity)
                tmp = cloud[i * pointNumPerPack + j].h -
                      vrEst * cos(pointCloudVec[i].point[j].azi);
                      
                // Classify point based on relative velocity:
                if (tmp < -errThr) {
                    cloud[i * pointNumPerPack + j].v = -1;  // Moving away (receding)
                } else if (tmp > errThr) {
                    cloud[i * pointNumPerPack + j].v = 1;   // Moving toward (approaching)
                } else {
                    cloud[i * pointNumPerPack + j].v = 0;   // Stationary relative to background
                }
            }
        }
    }
}

/**
 * @brief Main function - receives and processes radar point cloud data
 * 
 * The main function initializes resources, creates a socket for receiving
 * radar data packets, processes incoming point clouds, and saves results.
 * It runs an infinite loop to continuously receive and process data.
 * 
 * @param argc Number of command line arguments
 * @param argv Array of command line argument strings
 * @return 0 on successful execution
 */
int main(int argc, char** argv) {
    // Load RCS calibration data from file
    float* rcsBuf = (float*)malloc(1201 * sizeof(float));
    FILE* fp_rcs = fopen("data//rcs.dat", "rb");
    if (fp_rcs == NULL) {
        perror("Cannot open rcs.dat");
        return -1;
    }
    fread(rcsBuf, sizeof(float), 1201, fp_rcs);
    fclose(fp_rcs);

    // Allocate memory for processed point cloud data
    POINTCLOUDXYZ *cloud = (POINTCLOUDXYZ*)malloc(sizeof(POINTCLOUDXYZ) * widthSet);
    
    // Initialize socket for receiving radar data
    struct sockaddr_in from;           // Sender address structure
    socklen_t len = sizeof(from);      // Length of address structure
    int sockfd = socketGen();          // Create and configure socket
    if (sockfd == 0) {
        cerr << "Failed to create socket" << endl;
        return -1;
    }
    
    // Variables for point cloud reception and processing
    vector<POINTCLOUD> pointCloudVec;  // Vector to store received point clouds
    POINTCLOUD pointCloudBuf;          // Buffer for single point cloud packet
    char* recvBuf = (char*)&pointCloudBuf;  // Raw buffer pointer for receiving
    int installFlag = -1;              // Installation orientation flag
    int pointNumPerPack = 30;          // Number of points per packet
    int pointSizeByte = 44;            // Size of each point in bytes
    int recvFrameLen = 0;              // Received frame length (not used)
    int frameNum = 0;                  // Frame counter
    int frameId[2] = {0, 0};           // Frame IDs for different modes
    int cntPointCloud[2] = {0, 0};     // Point counters for different modes
    float vrEst = 0;                   // Estimated velocity
    unsigned short curObjInd;          // Current object index in packet
    unsigned char mode;                // Packet mode (0: normal, 1: last packet)
    float* histBuf = (float*)malloc(sizeof(float) * int((vrMax - vrMin) / vStep));  // Histogram buffer

    // Create data file with timestamp
    char filePath[1024];               // Buffer for file path
    struct timeval tv;                 // Time value structure
    struct tm tm;                      // Broken-down time structure
    gettimeofday(&tv, NULL);           // Get current time
    localtime_r(&tv.tv_sec, &tm);      // Convert to local time
    
    // Generate filename with timestamp
    sprintf(filePath, "data//%d_%d_%d_%d_%d_%d_altos.dat", tm.tm_year + 1900,
            tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
            
    FILE* fp = fopen(filePath, "wb");  // Open file for binary writing
    if (fp == NULL) {
        perror("Cannot create output file");
        return -1;
    }

    // Main loop for receiving and processing radar data
    while(true)
    {
        // Clear receive buffer
        memset(recvBuf, 0, sizeof(POINTCLOUD));
        
        // Receive UDP packet
        int ret = recvfrom(sockfd, recvBuf, sizeof(POINTCLOUD), 0, 
                           (struct sockaddr *)&from, &len);
                           
        if (ret > 0)
        {
            // Save raw data to file
            fwrite(recvBuf, 1, ret, fp);
            
            // Extract packet header information
            curObjInd = pointCloudBuf.pckHeader.curObjInd;  // Current object index
            mode = pointCloudBuf.pckHeader.mode;            // Packet mode
            cntPointCloud[mode] = pointCloudBuf.pckHeader.objectCount;  // Object count
            pointCloudVec.push_back(pointCloudBuf);         // Add to point cloud vector
            
            // Process complete frame when last packet is received
            if ((mode == 1 && (curObjInd + 1) * pointNumPerPack >= pointCloudBuf.pckHeader.objectCount)) {
                // Process all points in the frame
                calPoint(pointCloudVec, cloud, installFlag, rcsBuf, vStep,
                         histBuf, pointNumPerPack);
                         
                // Print frame information
                printf("0 pointNum of %d frame: %d\n",
                       pointCloudBuf.pckHeader.frameId,
                       cntPointCloud[0] + cntPointCloud[1]);
                       
                // Reset for next frame
                pointCloudVec.clear();
                memset(cloud, 0, sizeof(POINTCLOUDXYZ) * widthSet);
                cntPointCloud[0] = 0;
                cntPointCloud[1] = 0;
            }
        } else {
            // Handle receive timeout or error
            printf("recv failed (timeOut)   %d\n", ret);
        }
    }

    // Cleanup resources (note: this code is unreachable due to infinite loop)
    close(sockfd);
    free(histBuf);
    free(cloud);
    free(rcsBuf);
    fclose(fp);
    
    return 0;
}