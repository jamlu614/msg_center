#include <iostream>
#include <thread>
#include <signal.h>
#include "gac_ipc_metro.hpp"

bool g_quit;

static void exit_handler(int sig) {
    g_quit = true;
    return;
}

void write_thread(void) {
    printf("write init\n");
    if (-1 == gac_ipc_metro_init()) {
        printf("gac_ipc_metro_init error.\n");
        return;
    }
    printf("write\n");

    GacIpcMetro data;

    while(!g_quit) {
        data.algorithm_type++;
        gac_ipc_metro_write(data);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    gac_ipc_metro_exit();
    return;
}

int main(int argc, char **argv) {
    signal(SIGINT, exit_handler);
    signal(SIGTERM, exit_handler);
    signal(SIGCHLD, exit_handler);

    std::thread write_tid(write_thread);
    write_tid.join();
    return 0;
}
