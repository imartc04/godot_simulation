#pragma once

#include <thread>



/**
 * Auxiliar class to manage the creation and execution of a gRPC server
 * in a new thread
*/
class CServerThreadManager
{

public:
    CServerThreadManager(/* args */);
    ~CServerThreadManager();

    void init();

protected:

    /**
     * Method to be overriden by child class
     * In the method the child is supossed  just to call 
     * grcp server wait method
    */
    virtual void run_server() = 0;

private:

    std::thread m_server_thread;

    bool m_exit_thread = false;


    void server_thread();

};

CServerThreadManager::CServerThreadManager(/* args */)
{
}

CServerThreadManager::~CServerThreadManager()
{
}
