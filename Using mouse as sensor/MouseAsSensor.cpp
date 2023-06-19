#include <windows.h> // Header for Windows API
#include <iostream> // Header for input/output stream
#include <ctime> // Header for date and time functions
#include <fstream> // Header for file stream operations
#include <map> // Header for std::map container
#include <vector> // Header for std::vector container

HHOOK hhook = NULL; // Global variable for the mouse hook
std::map<std::string, int> timeCounts; // Map to store time counts
std::string currentTimeStr; // Current time string
std::ofstream outputFile; // Output file stream

// Callback function for mouse hook
LRESULT CALLBACK mouseHookProc(int nCode, WPARAM wParam, LPARAM lParam) {
    if (nCode >= 0) {
        MSLLHOOKSTRUCT* pMouseStruct = (MSLLHOOKSTRUCT*)lParam; // Retrieve mouse information
        time_t currentTime = time(NULL); // Get current time
        char timeStr[26];
        ctime_s(timeStr, sizeof timeStr, &currentTime); // Convert current time to string
        std::string newTimeStr(timeStr); // Create a new string from the time string
        newTimeStr.erase(newTimeStr.length() - 1); // Remove the newline character at the end of the string

        if (newTimeStr != currentTimeStr) { // If the new time string is different from the previous time string
            if (!currentTimeStr.empty()) {
                std::cout << "Time change: " << currentTimeStr << " repeated " << timeCounts[currentTimeStr] << " times" << std::endl;
            }
            currentTimeStr = newTimeStr; // Update the current time string
            timeCounts[currentTimeStr] = 1; // Initialize the count for the new time string
        }
        else {
            ++timeCounts[currentTimeStr]; // Increment the count for the same time string
        }

        outputFile.open("mouse_movements.csv", std::ios::app); // Open the output file in append mode
        outputFile << pMouseStruct->pt.x << "," << pMouseStruct->pt.y << "," << newTimeStr << std::endl; // Write mouse coordinates and time to the file
        outputFile.close(); // Close the output file

        std::cout << "Mouse moved: x=" << pMouseStruct->pt.x << ", y=" << pMouseStruct->pt.y << " at time: " << newTimeStr << std::endl;
    }
    return CallNextHookEx(hhook, nCode, wParam, lParam); // Call the next hook procedure in the hook chain
}

int main() {
    outputFile.open("mouse_movements.csv", std::ios::app); // Open the output file in append mode
    outputFile << "New Experiment" << "," << "New Experiment" << "," << "New Experiment" << std::endl; // Write experiment information to the file
    outputFile.close(); // Close the output file

    hhook = SetWindowsHookEx(WH_MOUSE_LL, mouseHookProc, NULL, 0); // Set the mouse hook
    if (hhook == NULL) {
        std::cerr << "Failed to install mouse hook" << std::endl;
        return 1;
    }

    MSG msg;
    while (GetMessage(&msg, NULL, 0, 0)) { // Enter the message loop
        TranslateMessage(&msg); // Translate virtual-key messages into character messages
        DispatchMessage(&msg); // Dispatch the message to the appropriate window procedure
    }

    UnhookWindowsHookEx(hhook); // Unhook the mouse hook
