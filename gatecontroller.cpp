#include <iostream>
#include <thread>
#include <mutex>
#include <csignal>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>    /* File control definitions */
#include <errno.h>    /* Error number definitions */
#include <termios.h>  /* POSIX terminal control definitions */
#include <stdint.h>   /* definitions of 8&16 bits ints*/
#include <sys/signal.h>  /* serial port uses interrupt*/
#include <stdlib.h>
#include <sys/ioctl.h>

using namespace std;

void signal_handler(int sig);

class GateController {

    const size_t bufferLength = 24;
    bool done = false;
    int ser_port;
    const string key = "123456";
	const string portname = "/dev/ttyACM0"; 
	

    std::mutex gate_operations_lock;
    bool gate_state = false;
    
    void open_gate() {cout << "\topening gate" << endl; write_serial('o');}
    void close_gate() {cout << "\tclosing gate" << endl; write_serial('c');}

    void password_challenge(int socketfd) {
         char message[bufferLength] = {'\0'};
         int n = read(socketfd, message, bufferLength);
         if (n < 0) {
            cout << "Error reading from socket: " << socketfd << endl;
            close(socketfd);
            return;
         }
	  cout << "\tPI first message: " << string(message, n) << endl;

         if(string(message, n).find("Please") != string::npos) { // they sent something containing 'Please'
             n = write(socketfd, key.c_str(), key.size()); // Give them the key
             if (n < 0) {
                cout << "Error writing to socket: " << socketfd << endl;
                close(socketfd);
                return;
             }

             for(int i = 0; i < bufferLength; i++)
                    message[i] = '\0'; // Zero out message buffer
       
             n = read(socketfd, message, bufferLength); // Get the response password from the pi
             if (n < 0) {
                cout << "Error reading from socket: " << socketfd << endl;
                close(socketfd);
                return;
             }
             
	     cout << "\tPI sent password: " << string(message) << endl;
             if(string(message).find( key ) != string::npos){
                close(socketfd); // The pie has sent the right key in there. Just end their connection so they can continue.
		
		gate_operations_lock.lock();	
		
		if(gate_state) {
			cout << "\tGate is already open" << endl;
			gate_operations_lock.unlock();
			return;
		}

		//The gate is closed. Open it
                open_gate();
		gate_state = true;
		gate_operations_lock.unlock();
		
			std::this_thread::sleep_for(4s);
		
		gate_operations_lock.lock();
		
			close_gate();
			gate_state = false;
		
		gate_operations_lock.unlock();

             }
         } else {
            cout << "\tDidnt say \"Please\"!" << endl;
         }

         close(socketfd);
    }

	int set_interface_attribs (int ser_port, int speed, int parity)
	{
			struct termios tty;
			memset (&tty, 0, sizeof tty);
			if (tcgetattr (ser_port, &tty) != 0)
			{
			  cout << "Error "<< errno << " from tcgetattr." << endl;
			  return -1;
			}

			cfsetospeed (&tty, speed);
			cfsetispeed (&tty, speed);

			tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
			// disable IGNBRK for mismatched speed tests; otherwise receive break
			// as \000 chars
			tty.c_iflag &= ~IGNBRK;         // ignore break signal
			tty.c_lflag = 0;                // no signaling chars, no echo,
											// no canonical processing
			tty.c_oflag = 0;                // no remapping, no delays
			tty.c_cc[VMIN]  = 0;            // read doesn't block
			tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

			tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

			tty.c_cflag |= (CLOCAL | CREAD);    // ignore modem controls,
												// enable reading
			tty.c_cflag &= ~(PARENB | PARODD);    // shut off parity
			tty.c_cflag |= parity;
			tty.c_cflag &= ~CSTOPB;
			tty.c_cflag &= ~CRTSCTS;

			if (tcsetattr (ser_port, TCSANOW, &tty) != 0)
			{
			  cout << "Error " << errno <<" from tcsetattr." << endl;
			   return -1;
			}
			return 0;
	}


	int open_serial()
	{
	   ser_port = open (portname.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
	   if (ser_port < 0)
	   {
		 cout << "Error " << errno << " opening " <<  portname.c_str() << ": "<<  portname.c_str()<< endl;
		 return -1;
	   }

	   set_interface_attribs (ser_port, B9600, 0);  // set speed to 9600 bps, 8n1 (no parity)
	   sleep(2); //required to make flush work, for some reason http://stackoverflow.com/questions/13013387/clearing-the-serial-ports-buffer
	   tcflush(ser_port,TCIOFLUSH); // does not work anyway - buffer filled with 0s, read blocking
	   
	   return 1;
	}


	int write_serial(char c)
	{
	   int n_bytes = write(ser_port,&c,1);
	   if (n_bytes!=1)
	   {
		  cout << "Writing to serial port stopped working. Please restart this program." << endl;
		  raise(SIGINT);
		  return -1;
	   }
	   return 1;
	}

    public:
    
    void start(int port) {

	    if(open_serial() != 1) {
			raise(SIGINT);
		}

	    write_serial('c'); //Close gate
	    
	   
		 signal(2, signal_handler);
        

         int sockfd = socket(AF_INET, SOCK_STREAM, 0);
         if (sockfd < 0)
               perror("ERROR creating server listening socket");

         int optval = 1;
         if (setsockopt(sockfd, SOL_SOCKET,SO_REUSEADDR, &optval, sizeof optval) != 0)
                perror("setsockopt encountered and error") ;


		struct sockaddr_in serv_addr, cli_addr;
		
         bzero((char *) &serv_addr, sizeof(serv_addr));

         serv_addr.sin_family = AF_INET;
         serv_addr.sin_addr.s_addr = INADDR_ANY;
         serv_addr.sin_port = htons(port);
         if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
                perror("ERROR on binding server socket");

         listen(sockfd, 5);

         while(!done) {
                socklen_t clilen = sizeof(cli_addr);
                int newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
                if (newsockfd < 0)
                    perror("ERROR on accepting new connection");
                else {
					
					if(cli_addr.sin_family == AF_INET) {
						cout << "Accepted new connection, ip: " << inet_ntoa(cli_addr.sin_addr) << endl; 
					} else {
						cout << "Accepted new connection." << endl;
					}
					
                    std::thread ChallengeThread(&GateController::password_challenge, this, newsockfd);
                    ChallengeThread.detach();
                }

            }

            close(sockfd);
        }
        
        
	void finish() {
		done = true;
		cout << "\nFinishing up" << endl;
			exit(0);
	}

};

GateController Controller; // Has to be defined globally so signal handling can work for clean shutdown

void signal_handler(int sig) {
		Controller.finish();
}


int main()
{	
	Controller.start(1024);
    return 0;
}
