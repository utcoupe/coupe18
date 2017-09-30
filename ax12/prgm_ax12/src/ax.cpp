#include "ax.h"


std::mutex ax::lockSerial;
std::mutex ax::lockCout;
using namespace std;
void ax::executeAction(int finalPos, int idOrder){

     std::lock_guard<std::mutex> lck (lockAction);
     //int finalPos = positions[pos];
     int currentPos[2];
     int current = 1, lastValue =0;
     currentPos[0] = 10000; currentPos[1] = 10000;
     // finalPos = toAxValue(finalPos);
     //std::cout << "ok thread, position =" << finalPos << ", id=" << id <<", reg=" << P_GOAL_POSITION_L<< std::endl;
     lockSerial.lock();
     dxl_write_word(id, 34, 1023);
     dxl_write_word(id, P_GOAL_POSITION_L, finalPos);
     lockSerial.unlock();
     //std::this_thread::sleep_for(std::chrono::milliseconds(100));
     time_t start    = time(0);
     time_t last = start;
     do{
         lockSerial.lock();
         dxl_read_word(id, 36);
         std::this_thread::sleep_for(std::chrono::milliseconds(10));
         currentPos[0] = dxl_get_rxpacket_parameter(0);
         currentPos[1] = dxl_get_rxpacket_parameter(1) << 8;
         lockSerial.unlock();
         current = currentPos[0] | currentPos[1];
         // std::cout << "Position actuelle: " << current  << ", position visee :" << finalPos << std::endl;
        //  cerr <<"kill:" <<killAction << endl;

         if(killAction == true){
             //cout << "Action killed" << endl;
             goto end;
         }
         std::this_thread::sleep_for(std::chrono::milliseconds(100));
         if (lastValue != current){
             last = time(0);
         }
         if(time(0) - start > vals::timeOut || time(0) - last > 1){
              lockSerial.lock();
              dxl_write_byte(id, 24, 0);
              lockSerial.unlock();
             cout << idOrder << ";"  << "0;" << endl;
             return;
         }
         lastValue = current;

     }while(current > finalPos+3 || current < finalPos - 3);
     //while(current > finalPos+3 || current < finalPos - 3);
     end:
     cout << idOrder << ";"  << endl;
     //string message = (string)idOrder + "action done";;
     //lockAction.unlock();

 }

 void ax::initialize(){
    /* for (std::map<int, int>::iterator it=vals::paramValues.begin(); it!=vals::paramValues.end(); ++it){
 		dxl_write_word(id, it->first, it->second);
 	}*/
 }


 void ax::goTo(int pos, int idOrder){
     if (actions.size() > 0){
         std::thread& lastThread = actions.back();
         if(lastThread.joinable() == true){
             killAction = true;
             lastThread.join();
             killAction = false;
        }
    }
    // killAction = false;


     actions.push_back(std::thread(&ax::executeAction, this, positions[pos], idOrder));

 }
