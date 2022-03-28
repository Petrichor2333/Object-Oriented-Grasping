#pragma once

#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <vector>
#include <winsock2.h>
#include <Ws2tcpip.h>
#include "pch.h"

constexpr auto PI = 3.141592653589793;

#define NDEBUG

using namespace std;

#pragma comment(lib, "ws2_32.lib")