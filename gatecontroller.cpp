#include <iostream>
#include <thread>
#include <mutex>
#include <csignal>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
using namespace std;

void error(const char *msg)
{
    perror(msg);
    exit(1);
}

bool done = false;

void finish(int sig) {
    done = true;
    cout << "\nFinishing up" << endl;
     exit(0);
}

class GateController {

    const size_t bufferLength = 24;
    const string key = "123456";

    std::mutex gate_operations_lock;
    void open_gate() {cout << "opening gate" << endl;}
    void close_gate() {cout << "closing gate" << endl;}


    void password_challenge(int socketfd) {
         char message[bufferLength] = {'\0'};
         int n = read(socketfd, message, bufferLength);
         if (n < 0) {
            cout << "Error reading from socket: " << socketfd << endl;
            close(socketfd);
            return;
         }
        cout << "Got data: " << string(message, n) << endl;

         if(string(message, n) == "Please\n") { // They match
             n = write(socketfd, key.c_str(), key.size());
             if (n < 0) {
                cout << "Error writing to socket: " << socketfd << endl;
                close(socketfd);
                return;
             }

             for(int i = 0; i < bufferLength; i++)
                    message[i] = 0;

             cout << "Ping" << endl;
             n = read(socketfd, message, bufferLength);
             cout << "Ping2" << endl;
             if (n < 0) {
                cout << "Error reading from socket: " << socketfd << endl;
                close(socketfd);
                return;
             }
             cout << "Ping1" << endl;

             if(string(message) == string(key+"Please\n")) {
                close(socketfd);
                gate_operations_lock.lock();

                open_gate();
                std::this_thread::sleep_for(4s);
                close_gate();

                gate_operations_lock.unlock();
             }
         } else {
            cout << "Failed to say please" << endl;
         }

         close(socketfd);
    }

    public:

    GateController(int port) {
            int sockfd, newsockfd;
            socklen_t clilen;

            struct sockaddr_in serv_addr, cli_addr;

            sockfd = socket(AF_INET, SOCK_STREAM, 0);
            if (sockfd < 0)
                error("ERROR opening socket");

            int optval = 1;
            if (setsockopt(sockfd, SOL_SOCKET,SO_REUSEADDR, &optval, sizeof optval) != 0)
                error("setsockopt") ;



            bzero((char *) &serv_addr, sizeof(serv_addr));

            serv_addr.sin_family = AF_INET;
            serv_addr.sin_addr.s_addr = INADDR_ANY;
            serv_addr.sin_port = htons(port);
            if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
                error("ERROR on binding");

            listen(sockfd, 5);

            while(!done) {
                clilen = sizeof(cli_addr);
                newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
                if (newsockfd < 0)
                    error("ERROR on accept");
                else {
                    cout << "Accepted a socket" << endl;
                    std::thread ChallengeThread(&GateController::password_challenge, this, newsockfd);
                    ChallengeThread.detach();
                }


            }

            close(sockfd);
        }

};



int main()
{
    signal(2, finish);
    GateController NewController(1024);
    return 0;
}
