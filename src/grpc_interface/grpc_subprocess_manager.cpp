
#include "grpc_subprocess_manager.hpp"

using namespace godot_grpc;

namespace bp = ::boost::process;

CGRPCSubprocess::~CGRPCSubprocess()
{
    if(m_thread.joinable())
    {
        set_exit_thread(true);

        //Force process to terminate
        m_process->terminate();

        m_thread.join();
    }
    else
    {
        //Check if process is running
        if(m_process)
        {
            //Force process to terminate
            m_process->terminate();
        }
    }
}

void CGRPCSubprocess::config(const CGRPCSubprocessConfig &config)
{
    m_config = config;
}

void CGRPCSubprocess::init()
{
    if (m_config.run_in_new_thread)
    {
        m_thread = std::thread(&CGRPCSubprocess::run, this);
    }
    else
    {
        run();
    }
}

//implement set_exit_thread
void CGRPCSubprocess::set_exit_thread(bool f_exit_thread)
{
    std::lock_guard<std::mutex> l_lock(m_exit_thread_mtx);
    exit_thread = f_exit_thread;
}

//implement get_exit_thread
bool CGRPCSubprocess::get_exit_thread()
{
    std::lock_guard<std::mutex> l_lock(m_exit_thread_mtx);
    return exit_thread;
}

void CGRPCSubprocess::run()
{
    if (m_process)
    {
        m_process->terminate();
    }

    int l_exit_code = 0;
    do
    {
        auto env = boost::this_process::environment();

        // Access the PATH variable
        std::string pathVar = env["GODOT_GEN_BIN_DIR"].to_string();

        // Create a boost::process::child with the command line
        m_process = std::make_unique<bp::child>(pathVar + "/" + m_config.exec_name, bp::args({m_config.server_address.c_str(), m_config.server_port.c_str()}), bp::std_out > stdout, bp::std_err > stderr, bp::std_in < stdin);

        m_process->wait();

        // Get return code
        l_exit_code = m_process->exit_code();
    } while (l_exit_code == 2 && !get_exit_thread());

    if (l_exit_code == 1 && !get_exit_thread())
    {
        throw std::runtime_error("Error running process");
    }

}
