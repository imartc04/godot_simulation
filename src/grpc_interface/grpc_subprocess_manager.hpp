#pragma once

#include <boost/process.hpp>
#include <string>
#include <thread>

namespace godot_grpc
{

    struct CGRPCSubprocessConfig
    {
        bool enable = false;

        /**
         * Name of the executable to run
         */
        std::string exec_name;

        /**
         * String of type server
         */
        std::string server_address;

        /**
         * String of type port
         */
        std::string server_port;

        /**
         * Flag to indicate if process creatino and management should
         * be exectued in new thread
         */
        bool run_in_new_thread = true;
    };

    /**
     * Class to maange the creattion and execution of 
     * a gRPC server. By default a custom thread is used to 
     * run the server.
    */
    class CGRPCSubprocess
    {

    public:

        ~CGRPCSubprocess();

        void config(const CGRPCSubprocessConfig &config);

        void init();

    private:
        void run();

        std::unique_ptr<boost::process::child> m_process;

        // config
        CGRPCSubprocessConfig m_config;

        void set_exit_thread(bool f_exit_thread);
        bool get_exit_thread();

        std::thread m_thread;

        bool exit_thread = false;
        std::mutex m_exit_thread_mtx;
    };

} // namespace godot_grpc
