// Copyright 2021 VorteX-co
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "../../../include/VDVL/executor.hpp"
#include <boost/thread.hpp>
#include <iostream>
void VDVL::Executor::Worker(boost::asio::io_service & ios)
{
  /* *************************************************
   * Worker function invoked by each new worker thread
   * Internally each thread  calls the run() method of
   * the boost::asio::io_service object.
   * The more threads that launch this method the more
   * workers are available for servicing.
   *************************************************** */
  while (true) {
    try {
      boost::system::error_code ec;
      // each call to ios.run from a thread makes the respective thread
      // available as a worker for the I/O service.
      ios.run(ec);
      if (ec && OnError) {
        OnError(ios, ec);
      }
      break;
    } catch (const std::exception & ex) {
      if (OnException) {
        OnException(ios, ex);
      }
    }
  }
}
void VDVL::Executor::Run(unsigned int numThreads)
{
  if (OnRun) {
    OnRun(_io);
  }
  /* ***********************************************
   * The Run method starts the the execution process by
   * creating a thread pool with supplied number of threads
   * if not supplied it set to the number of threads to
   * the number of physical execution units
   * ************************************************ */
  boost::thread_group workers;
  numThreads == static_cast<unsigned int>(-1) ?
  boost::thread::hardware_concurrency() :
  numThreads;
  for (unsigned int i = 0; i < numThreads; ++i) {
    // boost::bind() used to attach a function with it's arguments
    // to each created thread to be executed later.
    workers.create_thread(
      boost::bind(&VDVL::Executor::Worker, this, boost::ref(_io)));
  }
  // here we block untill the termination of each worker thread.
  workers.join_all();
}
boost::asio::io_service & VDVL::Executor::GetIO() {return boost::ref(_io);}
