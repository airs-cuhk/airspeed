#include <algorithm>
#include <iostream>
#include <vector>
#include <string>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "MocapApi.h"

#define VERIFY(r)                                                    \
  if (auto r_ = (r); r_ != Error_None) {                             \
    std::cout << "\'" << #r "\' return Error : " << r_ << std::endl; \
    throw std::runtime_error("");                                    \
  }

using namespace MocapApi;

// Function to send data to the client
void sendDataToClient(int sock, const std::string& message) {
  if (sock >= 0) {
    send(sock, message.c_str(), message.length(), 0);
  }
}

void printJointPosture(MCPJointHandle_t jointHandle, int sock) {
  IMCPJoint* mcpJoint = nullptr;
  VERIFY(MCPGetGenericInterface(IMCPJoint_Version, reinterpret_cast<void**>(&mcpJoint)));
  const char* szJointName = nullptr;
  VERIFY(mcpJoint->GetJointName(&szJointName, jointHandle));
  std::string message = "\t" + std::string(szJointName) + ":";

  float px, py, pz;
  VERIFY(mcpJoint->GetJointLocalPosition(&px, &py, &pz, jointHandle));
  message += "(" + std::to_string(px) + "," + std::to_string(py) + "," + std::to_string(pz) + ")";

  float rx, ry, rz;
  VERIFY(mcpJoint->GetJointLocalRotationByEuler(&rx, &ry, &rz, jointHandle));
  message += "(" + std::to_string(rx) + "," + std::to_string(ry) + "," + std::to_string(rz) + ")\n";
  sendDataToClient(sock, message);

  uint32_t unSizeOfJointHandle = 0;
  VERIFY(mcpJoint->GetJointChild(nullptr, &unSizeOfJointHandle, jointHandle));
  if (unSizeOfJointHandle > 0) {
    std::vector<MCPJointHandle_t> vJointHandles;
    vJointHandles.resize(unSizeOfJointHandle);
    VERIFY(mcpJoint->GetJointChild(vJointHandles.data(), &unSizeOfJointHandle, jointHandle));
    std::for_each(vJointHandles.begin(), vJointHandles.end(), [sock](MCPJointHandle_t childJointHandle) {
      printJointPosture(childJointHandle, sock);
    });
  }
}

void handleAvatarUpdated(const MCPEvent_MotionData_t& motionData, int sock) {
  IMCPAvatar* mcpvatar = nullptr;
  VERIFY(MCPGetGenericInterface(IMCPAvatar_Version, reinterpret_cast<void**>(&mcpvatar)));
  const char* szAvatarName = nullptr;
  VERIFY(mcpvatar->GetAvatarName(&szAvatarName, motionData.avatarHandle));
  std::string message = "Avatar \'" + std::string(szAvatarName) + "\' updated\n";
  sendDataToClient(sock, message);

  MCPJointHandle_t rootJointHandle = 0;
  VERIFY(mcpvatar->GetAvatarRootJoint(&rootJointHandle, motionData.avatarHandle));
  printJointPosture(rootJointHandle, sock);
}

void handleEvent(const MCPEvent_t& ev, int sock) {
  std::string errorMessage; // 提前声明和初始化
  switch (ev.eventType) {
    case MCPEvent_AvatarUpdated:
      handleAvatarUpdated(ev.eventData.motionData, sock);
      break;
    case MCPEvent_Error:
      errorMessage = "Error occur " + std::to_string(ev.eventData.systemError.error) + "\n";
      sendDataToClient(sock, errorMessage);
      break;
    default:
      std::string unhandledMessage = "Unhandled event\n";
      sendDataToClient(sock, unhandledMessage);
      break;
  }
}

int main() {
  std::cout << "MocapApi Version:" << MCPGetMocapApiVersionString() << std::endl;

  // Initialize socket
  int server_sock = socket(AF_INET, SOCK_STREAM, 0);
  if (server_sock == -1) {
    perror("Socket creation failed");
    return 1;
  }

  sockaddr_in server_addr;
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = INADDR_ANY;
  server_addr.sin_port = htons(8089); // Use port 8089 for the server

  if (bind(server_sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
    perror("Bind failed");
    return 1;
  }

  listen(server_sock, 5);
  std::cout << "Waiting for connection..." << std::endl;

  int client_sock = -1; // Initialize client socket as invalid

  IMCPSettings* mcpSettings = nullptr;
  VERIFY(MocapApi::MCPGetGenericInterface(
      MocapApi::IMCPSettings_Version, reinterpret_cast<void**>(&mcpSettings)));
  MCPSettingsHandle_t mcpSettingsHandle = 0;
  VERIFY(mcpSettings->CreateSettings(&mcpSettingsHandle));
  VERIFY(mcpSettings->SetSettingsUDP(8088, mcpSettingsHandle));
  VERIFY(mcpSettings->SetSettingsBvhData(MocapApi::BvhDataType_Binary, mcpSettingsHandle));
  VERIFY(mcpSettings->SetSettingsBvhTransformation(MocapApi::BvhTransformation_Enable, mcpSettingsHandle));
  VERIFY(mcpSettings->SetSettingsBvhRotation(MocapApi::BvhRotation_YXZ, mcpSettingsHandle));

  IMCPApplication* mcpApplication = nullptr;
  VERIFY(MCPGetGenericInterface(MocapApi::IMCPApplication_Version, reinterpret_cast<void**>(&mcpApplication)));
  MCPApplicationHandle_t appHandle = 0;
  VERIFY(mcpApplication->CreateApplication(&appHandle));
  VERIFY(mcpApplication->SetApplicationSettings(mcpSettingsHandle, appHandle));
  VERIFY(mcpSettings->DestroySettings(mcpSettingsHandle));
  VERIFY(mcpApplication->OpenApplication(appHandle));

  for (;;) {
    // Accept new client connections
    if (client_sock < 0) {
      struct sockaddr_in client_addr;
      socklen_t addr_len = sizeof(client_addr);
      client_sock = accept(server_sock, (struct sockaddr*)&client_addr, &addr_len);
      if (client_sock >= 0) {
        std::cout << "Connected to client" << std::endl;
      } else {
        perror("Accept failed");
        continue;
      }
    }

    // Process mocap events
    std::vector<MCPEvent_t> vEvents;
    uint32_t unEvent = 0;
    VERIFY(mcpApplication->PollApplicationNextEvent(nullptr, &unEvent, appHandle));
    vEvents.resize(unEvent);
    std::for_each(vEvents.begin(), vEvents.end(), [](MCPEvent_t& ev) { ev.size = sizeof(MCPEvent_t); });
    auto mcpError = mcpApplication->PollApplicationNextEvent(vEvents.data(), &unEvent, appHandle);
    if (mcpError == Error_None) {
      vEvents.resize(unEvent);
      std::for_each(vEvents.begin(), vEvents.end(), [client_sock](const MCPEvent_t& ev) { handleEvent(ev, client_sock); });
    } else if (mcpError != Error_MoreEvent) {
      std::string errorMessage = "\'mcpApplication->PollApplicationNextEvent(nullptr, &unEvent, appHandle)\' return Error : " +
                                 std::to_string(mcpError) + "\n";
      sendDataToClient(client_sock, errorMessage);
      throw std::runtime_error(errorMessage);
    }

    // Check if the client has disconnected
    char buffer[1];
    ssize_t bytes_received = recv(client_sock, buffer, 1, MSG_PEEK | MSG_DONTWAIT);
    if (bytes_received == 0) { // Client disconnected
      std::cout << "Client disconnected" << std::endl;
      close(client_sock);
      client_sock = -1;
    }
  }

  close(server_sock);

  VERIFY(mcpApplication->CloseApplication(appHandle));
  VERIFY(mcpApplication->DestroyApplication(appHandle));
  return 0;
}