
#include "server_thread_manager.hpp"


CServerThreadManager::CServerThreadManager(/* args */)
{
}

CServerThreadManager::~CServerThreadManager()
{
}


void CServerThreadManager::init()
{
    m_server_thread = std::thread(&CServerThreadManager::server_thread, this);
}

void CServerThreadManager::server_thread()
{
    while (!m_exit_thread)
    {
        run_server();
    }
}